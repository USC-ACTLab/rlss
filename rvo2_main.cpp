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
#include "discrete_search.hpp"

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


  nlohmann::json compareStats;
  nlohmann::json jsn = nlohmann::json::parse(cfg);

  string initial_trajectories_path = jsn["trajectories"];
  string obstacles_path = jsn["obstacles"];
  double dt = jsn["replan_period"];
  dt/=10;
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
  const double additional_time = jsn["additional_time"];

  const bool enable_discrete = jsn["rvo2_enable_discrete"];

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
        // This function sorts the points internally => do not call!
        // o.convex_hull();
        // o.ch_planes(/*shift*/0);
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
  compareStats["number_of_robots"] = original_trajectories.size();
  int steps_per_cycle = (dt / printdt)+0.5;
  int output_iter = 0;

  for(int i=0; i<original_trajectories.size(); i++) {
    for(double t=0; t<=total_t; t+=printdt) {
      vectoreuc eu = original_trajectories[i].eval(t);
      output_json["originals"][i]["x"].push_back(eu[0]);
      output_json["originals"][i]["y"].push_back(eu[1]);
    }
  }


  vector<double> max_opt_times(original_trajectories.size(), -1);
  vector<double> average_opt_times(original_trajectories.size(), 0);
  vector<int> opt_counts(original_trajectories.size(), 0);
  vector<int> obstacle_crash_counts(original_trajectories.size(), 0);
  vector<int> obstacle_no_crash_counts(original_trajectories.size(), 0);
  vector<int> robot_crash_counts(original_trajectories.size(), 0);
  vector<int> robot_no_crash_counts(original_trajectories.size(), 0);

  // RVO2
  RVO::RVOSimulator sim;
  sim.setTimeStep(dt);
  sim.setAgentDefaults(
    /*neighborDist*/ 4 * robot_radius,
    /*maxNeighbors*/ original_trajectories.size() - 1,
    /*timeHorizon*/ 1.5 * dt, // or hor?
    /*timeHorizonObst*/ 1.5 * dt, // or hor?
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

  std::chrono::time_point<std::chrono::high_resolution_clock> t_start, t_start_a_star, t_end_a_star, t_start_svm, t_end_svm, t_start_qp, t_end_qp;
  // main loop
  for(double ct = 0; ct <= total_t + additional_time /*+ 5*/ /*!everyone_reached*/ ; ct+=dt) {
    for(int i=0; i<trajectories.size(); i++) {
      t_start = Time::now();
      std::cout << "ct: " << ct << ", i: " << i << std::endl;
      RVO::Vector2 pos = sim.getAgentPosition(i);
      vectoreuc curPos(2);
      curPos[0] = pos.x();
      curPos[1] = pos.y();
      // Visualization
      output_json["points"][output_iter].push_back(curPos.crds);

      // set prefered velocity
      vectoreuc goalPos = original_trajectories[i].neval(min(ct, total_times[i]), 0);
      vectoreuc goalVel = original_trajectories[i].neval(min(ct, total_times[i]), 1);

      // check if original trajectory is occupied
      bool original_traj_occupied = og.occupied(original_trajectories[i], robot_radius, ct, min(ct+hor, total_times[i]));

      bool discretePath = false;
      if (enable_discrete && original_traj_occupied) {
        // attempt to follow discrete path

        std::vector< std::pair<double, double> > otherRobots;
        for (int j = 0; j < original_trajectories.size(); ++j) {
          if (i != j) {
            RVO::Vector2 otherPos = sim.getAgentPosition(j);
            otherRobots.emplace_back(std::make_pair(otherPos.x(), otherPos.y()));
          }
        }

        // the goal is the next point on the original trajectory that is not occupied
        discreteSearch::State goal(-1, -1, OG::direction::NONE);
        double arrivalTime = min(ct+hor, total_times[i]);
        vectoreuc goalPos;
        double discrete_horizon = 0;
        for (double t = arrivalTime; t <= total_times[i]; t += 0.01) {
          goalPos = original_trajectories[i].neval(t, 0);
          OG::index goalIdx = og.get_index(goalPos[0], goalPos[1]);
          std::pair<double, double> coord = og.get_coordinates(goalIdx);
          if (!og.occupied(coord.first, coord.second, robot_radius)) {

            // check of occupied by another robot
            bool occupiedByOtherRobot = false;

            for (const auto& otherRobot : otherRobots) {
              double distSq = pow(otherRobot.first - coord.first, 2) + pow(otherRobot.second - coord.second, 2);
              if (distSq < pow(2 * robot_radius, 2)) {
                occupiedByOtherRobot = true;
                break;
              }
            }

            if (!occupiedByOtherRobot) {
              goal.x = goalIdx.i;
              goal.y = goalIdx.j;
              discrete_horizon = t - ct;
              break;
            }
          }
        }

        if (goal.x == -1) {
          std::cerr << "Couldn't find unoccupied space on original trajectory!" << ct << std::endl;
          // throw std::runtime_error("Couldn't find unoccupied space on original trajectory!");
        }
        std::cout << "discrete horizon: " << discrete_horizon << std::endl;

        OG::index startIdx = og.get_index(pos.x(), pos.y());
        discreteSearch::State start(startIdx.i, startIdx.j, OG::direction::NONE);

        discreteSearch::Environment env(og, otherRobots, robot_radius, goal);

        libSearch::AStar<discreteSearch::State, discreteSearch::Action, int, discreteSearch::Environment> astar(env);
        libSearch::PlanResult<discreteSearch::State, discreteSearch::Action, int> solution;

        // t_start_a_star = Time::now();
        bool success = astar.search(start, solution);
        // t_end_a_star = Time::now();

        if (success) {
          std::cout << "discrete planning successful! Total cost: " << solution.cost << std::endl;
          // for (size_t i = 0; i < solution.actions.size(); ++i) {
          //   std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i].first << "(cost: " << solution.actions[i].second << ")" << std::endl;
          // }
          // std::cout << solution.states.back().second << ": " << solution.states.back().first << std::endl;

          // for (const auto& s : solution.states) {
          //   const discreteSearch::State& state = s.first;
          //   OG::index idx(state.x, state.y);
          //   pair<double, double> coord = og.get_coordinates(idx);
          //   std::cout << coord.first << "," << coord.second << std::endl;
          // }

          // combine results to lines (stored in corners vector; contains at least 2 elements)
          std::vector<pair<double, double>> corners;
          const discreteSearch::State& state = solution.states.front().first;
          OG::index idx1(state.x, state.y);
          // corners.emplace_back(og.get_coordinates(idx1));
          // double startx = positions[i][0];
          // double starty = positions[i][1];
          corners.emplace_back(std::make_pair(pos.x(), pos.y()));

          corners.emplace_back(og.get_coordinates(idx1));


          for (size_t j = 0; j < solution.actions.size(); ++j) {
            if (solution.actions[j].first != discreteSearch::Action::Forward) {
              const discreteSearch::State& state2 = solution.states[j].first;
              OG::index idx2(state2.x, state2.y);
              corners.emplace_back(og.get_coordinates(idx2));
            }
          }
          const discreteSearch::State& state3 = solution.states.back().first;
          OG::index idx3(state3.x, state3.y);
          corners.emplace_back(og.get_coordinates(idx3));
          corners.emplace_back(std::make_pair(goalPos[0], goalPos[1]));

          for (const auto& corner : corners) {
            std::cout << "corner: " << corner.first << "," << corner.second << std::endl;
            output_json["discrete_plan"][output_iter][i]["x"].push_back(corner.first);
            output_json["discrete_plan"][output_iter][i]["y"].push_back(corner.second);
          }

          double total_discrete_path_length = 0.0;
          for (size_t j = 0; j < corners.size() - 1; ++j) {
            total_discrete_path_length += sqrt(pow(corners[j].first - corners[j+1].first, 2) +
                                               pow(corners[j].second - corners[j+1].second, 2));
          }

          // compute preferred velocities
          double current_discrete_path_length = 0.0;
          for (size_t j = 1; j < corners.size(); ++j) {
            vectoreuc pt(2);
            pt[0] = corners[j].first;
            pt[1] = corners[j].second;
            current_discrete_path_length += sqrt(pow(corners[j].first - corners[j-1].first, 2) +
                                                 pow(corners[j].second - corners[j-1].second, 2));

            if ((pt - curPos).L2norm() > cell_size / sqrt(2)) {
              // attempt to reach next goal point by the desired time
              double timeToReach = current_discrete_path_length / total_discrete_path_length * discrete_horizon;
              vectoreuc vel = (pt - curPos) / timeToReach;
              std::cout << "pt " << pt << " curPos: " << curPos << " timeToReach: " << timeToReach <<
              " tdpl " << total_discrete_path_length << " cdpl " << current_discrete_path_length << " dh " << discrete_horizon <<
              std::endl;
              sim.setAgentPrefVelocity(i, RVO::Vector2(vel[0], vel[1]));
              std::cout << " prefvel: " << vel << std::endl;
              break;
            }
          }


          discretePath = true;

        } else {
          std::cerr << "discrete planning NOT successful!" << ct << std::endl;
        }
      }

      if (!discretePath) {
        // attempt to follow original trajectory

        if ((goalPos - curPos).L2norm() < 0.01) {
          // if we are close to our target position, just use the planned velocity
        /*  if(goalVel.L2norm() > v_max) {
            goalVel = goalVel * (v_max/goalVel.L2norm());
          }*/
          sim.setAgentPrefVelocity(i, RVO::Vector2(goalVel[0], goalVel[1]));
        } else {
          // otherwise, use a velocity that reaches the target position in the next time step
          goalPos = original_trajectories[i].neval(min(ct+dt, total_times[i]), 0);
          vectoreuc vel = (goalPos - curPos) / dt;
        /*  if(vel.L2norm() > v_max) {
            vel = vel * (v_max/vel.L2norm());
          }*/

          sim.setAgentPrefVelocity(i, RVO::Vector2(vel[0], vel[1]));
        }
      }


      /*
       * Perturb a little to avoid deadlocks due to perfect symmetry.
       */
      float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
      float dist = std::rand() * 0.0001f / RAND_MAX;

      sim.setAgentPrefVelocity(i, sim.getAgentPrefVelocity(i) +
                                dist * RVO::Vector2(std::cos(angle), std::sin(angle)));

      fsec fs = Time::now() - t_start;
      ms d = chrono::duration_cast<ms>(fs);
      average_opt_times[i]+=d.count();
      opt_counts[i]++;
      max_opt_times[i] = max((double)d.count(), max_opt_times[i]);
    }
    output_iter++;

    sim.doStep();
    for(int i=0; i<original_trajectories.size(); i++) {
      RVO::Vector2 pos = sim.getAgentPosition(i);
      vectoreuc vec(2);
      vec[0] = pos.x();
      vec[1] = pos.y();

      bool crashed = false;
      for(int p=0; p<cell_based_obstacles.size(); p++) {
        obstacle2D& obs = cell_based_obstacles[p];
        if(obs.point_inside(vec, robot_radius)) {
          crashed = true;
          break;
        }
      }

      if(crashed) {
        obstacle_crash_counts[i]++;
      } else {
        obstacle_no_crash_counts[i]++;
      }

      crashed = false;
      for(int p = 0; p<original_trajectories.size(); p++) {
        if(p==i) continue;
        vectoreuc vec2(2);
        vec2[0] = sim.getAgentPosition(p).x();
        vec2[1] = sim.getAgentPosition(p).y();
        if((vec2-vec).L2norm() < 2*robot_radius) {
          crashed = true;
          break;
        }
      }

      if(crashed) {
        robot_crash_counts[i]++;
      } else {
        robot_no_crash_counts[i]++;
      }

    }
  }

  // write output file
  out << std::setw(2) << output_json << endl;


    for(int i=0; i<average_opt_times.size(); i++) {
      average_opt_times[i] /= opt_counts[i];
    }

  compareStats["average_cycle_times"] = average_opt_times;
  compareStats["maximum_cycle_times"] = max_opt_times;
  compareStats["robot_crash_counts"] = robot_crash_counts;
  compareStats["robot_no_crash_counts"] = robot_no_crash_counts;
  compareStats["obstacle_crash_counts"] = obstacle_crash_counts;
  compareStats["obstacle_no_crash_counts"] = obstacle_no_crash_counts;


  vector<bool> robots_reached(original_trajectories.size());
  for(int i=0; i<original_trajectories.size(); i++) {
    vectoreuc pos(2);
    pos[0] = sim.getAgentPosition(i).x();
    pos[1] = sim.getAgentPosition(i).y();
    double diff = (original_trajectories[i].eval(total_times[i]) - pos).L2norm();
    robots_reached[i] = (diff <= 0.4);
  }

  compareStats["robots_reached"] = robots_reached;

  ofstream compfile("compareStats_rvo2.json");
  compfile << std::setw(2) << compareStats << endl;
  compfile.close();

  return 0;

}
