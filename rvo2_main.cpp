#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <vector>
#include "csv/CSVparser.hpp"
#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <utility>
#include "vectoreuc.h"
#include <chrono>
#include "trajectory.h"
#include "curve.h"
#include "hyperplane.h"
#include "obstacle.h"
#include "cxxopts.hpp"
#include "json.hpp"
#include <random>
#include "utility.h"
#include "occupancy_grid.h"
// #include "discrete_search.hpp"

#include <RVO.h>

namespace fs = std::experimental::filesystem::v1;
using namespace std;


typedef chrono::high_resolution_clock Time;
typedef chrono::milliseconds ms;
typedef chrono::duration<float> fsec;

int main(int argc, char** argv) {

  string config_path;
  cxxopts::Options options("Trajectory Replanner", "trajectory replanner");
  options.add_options()
    ("cfg", "Config file", cxxopts::value<std::string>()->default_value("../config.json"))
    ("help", "Display help page");

  auto result = options.parse(argc, argv);

  if(result.count("help")>0) {
    cout << options.help() << endl;
    return 0;
  }

  config_path = result["cfg"].as<string>();

  ifstream cfg(config_path);

  nlohmann::json jsn = nlohmann::json::parse(cfg);

  string initial_trajectories_path = jsn["trajectories"];
  string obstacles_path = jsn["obstacles"];
  double dt = jsn["replan_period"];
  int problem_dimension = jsn["problem_dimension"];
  int ppc = jsn["points_per_curve"];
  int max_continuity = jsn["continuity_upto_degree"];
  bool set_max_time = jsn["set_max_time_as_replan_period"];
  double hor = jsn["planning_horizon"];
  int curve_count = jsn["plan_for_curves"];
  double time_per_curve = hor / curve_count;
  string outputfile = jsn["output_file"];
  const double robot_radius = jsn["robot_radius"];
  const double cell_size = jsn["cell_size"];
  const double v_max = jsn["v_max"];
  const double a_max = jsn["a_max"];

  bool enable_voronoi = jsn["enable_voronoi"];

  string alg = jsn["algorithm"];

  nlohmann::json output_json;

  vector<trajectory> original_trajectories;
  ofstream out(outputfile);

  for (auto & p : fs::directory_iterator(initial_trajectories_path)) {
    csv::Parser file(p.path().string());
    trajectory trj;
    for(int i=0; i<file.rowCount(); i++) {
      double duration = stod(file[i][0]);
      curve crv(duration, problem_dimension);
      for(int j=1; j<=problem_dimension * ppc; j+=problem_dimension) {
        vectoreuc cp(problem_dimension);
        for(int u=0; u<problem_dimension; u++) {
          cp[u] = stod(file[i][j+u]);
        }
        crv.add_cpt(cp);
      }
      trj.add_curve(crv);
    }
    original_trajectories.push_back(trj);
  }


  vector<trajectory> trajectories(original_trajectories.size());

  for(int i=0; i<trajectories.size(); i++) {
    for(int j=0; j<curve_count; j++) {
      trajectories[i].add_curve(original_trajectories[i][min(j, original_trajectories[i].size() -1)]);
      trajectories[i][j].duration = time_per_curve;
    }
  }

  vector<obstacle2D> obstacles;
  int obs_idx = 0;
  for(auto & p : fs::directory_iterator(obstacles_path)) {
    csv::Parser file(p.path().string());
    obstacle2D o;
    for(int i=0; i<file.rowCount(); i++) {
      vectoreuc pt(problem_dimension);
      for(int u=0; u<problem_dimension; u++)
        pt[u] = stod(file[i][u]);
      output_json["obstacles"][obs_idx].push_back(pt.crds);
      o.add_pt(pt);
    }
    o.convex_hull();
    o.ch_planes(/*shift*/ 0);
    obstacles.push_back(o);
    obs_idx++;
  }

  OG og(cell_size, -10, 10, -10, 10, obstacles);
  output_json["cell_size"] = cell_size;

  vector<obstacle2D> cell_based_obstacles;
  for (size_t i = 0; i < og.max_i(); ++i) {
    for (size_t j = 0; j < og.max_j(); ++j) {
      OG::index idx(i, j);
      if (og.idx_occupied(idx)) {
        pair<double, double> coord = og.get_coordinates(idx);
        // std::cout << "occ: " << coord.first << "," << coord.second << std::endl;
        output_json["occupied_cells"]["x"].push_back(coord.first);
        output_json["occupied_cells"]["y"].push_back(coord.second);

        obstacle2D o;
        vectoreuc pt(problem_dimension);
        pt[0] = coord.first - cell_size / 2.0;
        pt[1] = coord.second - cell_size / 2.0;
        o.add_pt(pt);
        pt[0] = coord.first + cell_size / 2.0;
        pt[1] = coord.second - cell_size / 2.0;
        o.add_pt(pt);
        pt[0] = coord.first + cell_size / 2.0;
        pt[1] = coord.second + cell_size / 2.0;
        o.add_pt(pt);
        pt[0] = coord.first - cell_size / 2.0;
        pt[1] = coord.second + cell_size / 2.0;
        o.add_pt(pt);
        o.convex_hull();
        o.ch_planes(/*shift*/0);
        cell_based_obstacles.push_back(o);
      }
    }
  }

  vector<double> total_times(original_trajectories.size());
  double total_t = 0;
  for(int i=0; i<original_trajectories.size(); i++) {
    double tt = 0;
    for(int j=0; j<original_trajectories[i].size(); j++) {
      tt += original_trajectories[i][j].duration;
    }
    total_t = max(total_t, tt);
    total_times[i] = tt;
  }

  double printdt = jsn["print_dt"];

  output_json["dt"] = dt;//printdt;
  output_json["robot_radius"] = robot_radius;
  output_json["number_of_robots"] = original_trajectories.size();
  int steps_per_cycle = (dt / printdt)+0.5;
  int output_iter = 0;

  for(int i=0; i<original_trajectories.size(); i++) {
    for(double t=0; t<=total_t; t+=printdt) {
      vectoreuc eu = original_trajectories[i].eval(t);
      output_json["originals"][i]["x"].push_back(eu[0]);
      output_json["originals"][i]["y"].push_back(eu[1]);
    }
  }

  // RVO2
  RVO::RVOSimulator sim;
  sim.setTimeStep(dt);
  sim.setAgentDefaults(
    /*neighborDist*/ 4 * robot_radius,
    /*maxNeighbors*/ original_trajectories.size() - 1,
    /*timeHorizon*/ hor, // or hor?
    /*timeHorizonObst*/ hor, // or hor?
    /*radius*/ robot_radius,
    /*maxSpeed*/ v_max);

  // add agents
  for(int i=0; i<original_trajectories.size(); i++) {
    vectoreuc startPos = original_trajectories[i].eval(0);
    sim.addAgent(RVO::Vector2(startPos[0], startPos[1]));
  }

  // add obstacles
  // Add (polygonal) obstacle(s), specifying vertices in counterclockwise order.
  for (auto& obs : cell_based_obstacles) {
    std::vector<RVO::Vector2> vertices;
    for (auto& pt : obs.pts) {
      vertices.push_back(RVO::Vector2(pt[0], pt[1]));
    }
    sim.addObstacle(vertices);
  }

  // Process obstacles so that they are accounted for in the simulation.
  sim.processObstacles();

  // main loop
  for(double ct = 0; ct <= total_t /*+ 5*/ /*!everyone_reached*/ ; ct+=dt) {
    for(int i=0; i<trajectories.size(); i++) {
      RVO::Vector2 pos = sim.getAgentPosition(i);
      vectoreuc curPos(2);
      curPos[0] = pos.x();
      curPos[1] = pos.y();
      // Visualization
      output_json["points"][output_iter].push_back(curPos.crds);

      // set prefered velocity
      vectoreuc goalPos = original_trajectories[i].neval(min(ct, total_times[i]), 0);
      vectoreuc goalVel = original_trajectories[i].neval(min(ct, total_times[i]), 1);

      if ((goalPos - curPos).L2norm() < 0.01) {
        // if we are close to our target position, just use the planned velocity
        sim.setAgentPrefVelocity(i, RVO::Vector2(goalVel[0], goalVel[1]));
      } else {
        // otherwise, use a velocity that reaches the target position in the next time step
        goalPos = original_trajectories[i].neval(min(ct+dt, total_times[i]), 0);
        vectoreuc vel = (goalPos - curPos) / dt;
        sim.setAgentPrefVelocity(i, RVO::Vector2(vel[0], vel[1]));
      }

    }
    output_iter++;
    sim.doStep();
  }

  // write output file
  out << std::setw(2) << output_json << endl;
  return 0;

}
