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
#include <chrono>
#include <random>
#include "svmoptimization.h"
#include "utility.h"
#include "occupancy_grid.h"
#include "svm_seperator.h"
#include "discrete_search.hpp"

#define QP_SOLVER_QPOASES 0
#define QP_SOLVER_OSQP    1
#define QP_SOLVER QP_SOLVER_QPOASES
// #define QP_SOLVER QP_SOLVER_OSQP

#include "qp_optimize.h"

#if QP_SOLVER == QP_SOLVER_QPOASES
  #include <qpOASES.hpp>
#endif
#if QP_SOLVER == QP_SOLVER_OSQP
  #include "osqp.h"
#endif

#define PATHREPLAN_EPSILON 0.00001

namespace fs = std::experimental::filesystem::v1;
using namespace std;


typedef chrono::high_resolution_clock Time;
typedef chrono::milliseconds ms;
typedef chrono::duration<float> fsec;

double linearInterpolation(double start, double end, size_t idx, size_t count)
{
  return start + (end - start) * idx / (count - 1);
}

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
  double integral_stopval = jsn["objective_stop_value"];
  double relative_integral_stopval = jsn["objective_relative_stop_value"];
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

  vector<double> continuity_tols;
  if(max_continuity >= 0) {
    continuity_tols.resize(max_continuity+1);
    for(int i=0; i<=max_continuity; i++) {
      continuity_tols[i] = jsn["continuity_tolerances"][i];
    }
  }

  int max_initial_point_degree = jsn["initial_point_constraints_upto_degree"];
  vector<double> initial_point_tols;
  if(max_initial_point_degree >= 0) {
    initial_point_tols.resize(max_initial_point_degree+1);
    for(int i=0; i<=max_initial_point_degree; i++) {
      initial_point_tols[i] = jsn["initial_point_tolerances"][i];
    }
  }

  double obstacle_tolerance = jsn["obstacle_tolerance"];

  bool enable_voronoi = jsn["enable_voronoi"];

  vector<int> dynamic_constraint_degrees = jsn["dynamic_constraint_degrees"];
  vector<double> dynamic_constraint_max_values = jsn["dynamic_constraint_max_value_squares"];

  string alg = jsn["algorithm"];


  nlohmann::json output_json;


  srand(time(NULL));


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

  vector<vectoreuc> pos_diffs(trajectories.size());
  vectoreuc zerovec(2);
  zerovec.zero();
  for(int i=0; i<trajectories.size(); i++) {
    pos_diffs[i] = zerovec;
  }

/*
  cout << "Number of trajectories: " << trajectories.size() << endl;

  for(int i=0; i<trajectories.size(); i++) {
    cout << "\ttraj#" << i << endl;
    cout << "\t\tnumber of curves: " << trajectories[i].size() << endl;
  }
*/


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
    o.ch_planes();
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
        pt[0] = coord.first - cell_size / 2.0;
        pt[1] = coord.second + cell_size / 2.0;
        o.add_pt(pt);
        pt[0] = coord.first + cell_size / 2.0;
        pt[1] = coord.second + cell_size / 2.0;
        o.add_pt(pt);
        pt[0] = coord.first + cell_size / 2.0;
        pt[1] = coord.second - cell_size / 2.0;
        o.add_pt(pt);
        o.convex_hull();
        o.ch_planes();
        cell_based_obstacles.push_back(o);
      }
    }
  }
  SvmSeperator svm(&cell_based_obstacles);

/*
  for(double x=-9; x<9; x+=0.01) {
    for(double y = -9; y<9; y+=0.01) {
      OG::index idx = og.get_index(x,y);
      vector<OG::index> neighs = og.neighbors(idx);
      if(neighs.size() != 4) {
        cout << x << " , " << y << ": " << neighs.size() << endl;
      }
    }
  }

  int p; cin >> p;
*/

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


  vector<vectoreuc> positions(trajectories.size());
  vector<vectoreuc> velocities(trajectories.size());
  vector<vectoreuc> accelerations(trajectories.size());

  double printdt = jsn["print_dt"];

  output_json["dt"] = printdt;
  output_json["number_of_robots"] = original_trajectories.size();
  int steps_per_cycle = (dt / printdt)+0.5;
  int output_iter = 0;


  double total_time_for_opt = 0;
  int total_count_for_opt = 0;



  for(int i=0; i<trajectories.size(); i++) {
    positions[i] = trajectories[i].eval(0);
    cout << "init pos: " << positions[i]<< endl;
    velocities[i] = trajectories[i].neval(0, 1);
    cout << "init vel: " << velocities[i]<< endl;
    accelerations[i] = trajectories[i].neval(0, 2);
    cout << "init acc: " << accelerations[i]<< endl;
  }

  for(int i=0; i<original_trajectories.size(); i++) {
    for(double t=0; t<=total_t; t+=printdt) {
      vectoreuc eu = original_trajectories[i].eval(t);
      output_json["originals"][i]["x"].push_back(eu[0]);
      output_json["originals"][i]["y"].push_back(eu[1]);
    }
  }

  ObjectiveBuilder::Init();
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,0.00);

  ofstream outStats("stats.csv");
  outStats << "t";
  for (size_t i = 0; i < original_trajectories.size(); ++i) {
    outStats << ",total" << i << ",astar" << i << ",svm" << i << ",qp" << i;
  }
  outStats << std::endl;

  std::chrono::time_point<std::chrono::high_resolution_clock> t_start, t_start_a_star, t_end_a_star, t_start_svm, t_end_svm, t_start_qp, t_end_qp;

  double everyone_reached = false;

  for(double ct = 0; ct <= total_t + 5 /*!everyone_reached*/ ; ct+=dt) {

    outStats << ct;

    cerr << ct << " / " << total_t << endl;

    for(int i=0; i<original_trajectories.size(); i++ ) {
      cout << "traj " << i << " start " << ct << " / " << total_times[i] << endl;
      t_start = Time::now();

      // if (ct > 7 && i == 0) {
      //   continue;
      // }

#if 0
      // update cellBasedObstacles with robot positions, such that svmSeparator take other robots into account
      for (int j = 0; j < original_trajectories.size(); ++j) {
        if (i != j) {
          // og.set_occupied(positions[j][0], positions[j][1]);

          obstacle2D o;
          vectoreuc pt(problem_dimension);
          pt[0] = positions[j][0] - robot_radius;
          pt[1] = positions[j][1] - robot_radius;
          o.add_pt(pt);
          pt[0] = positions[j][0] - robot_radius;
          pt[1] = positions[j][1] + robot_radius;
          o.add_pt(pt);
          pt[0] = positions[j][0] + robot_radius;
          pt[1] = positions[j][1] + robot_radius;
          o.add_pt(pt);
          pt[0] = positions[j][0] + robot_radius;
          pt[1] = positions[j][1] - robot_radius;
          o.add_pt(pt);
          o.convex_hull();
          o.ch_planes();
          cell_based_obstacles.push_back(o);
        }
      }
#endif

      /*calculate voronoi hyperplanes for robot i*/
      vector<hyperplane> voronoi_hyperplanes;
      if(enable_voronoi) {
        voronoi_hyperplanes = voronoi(positions,i);
      }

      for(int j=0; j<voronoi_hyperplanes.size(); j++) {
        output_json["voronois"][output_iter][i][j].push_back(voronoi_hyperplanes[j].normal[0]); //normal first
        output_json["voronois"][output_iter][i][j].push_back(voronoi_hyperplanes[j].normal[1]);//normal seconds
        output_json["voronois"][output_iter][i][j].push_back(voronoi_hyperplanes[j].distance); //distance
        output_json["voronois"][output_iter][i][j].push_back(ct);
        output_json["voronois"][output_iter][i][j].push_back(ct+dt);
      }

      // if (ct >= total_times[i] - 3 * dt) {
      //   continue;
      // }


      unsigned varcount = curve_count * ppc * problem_dimension;

      // shift last trajectories
      // for(int p=0; p<problem_dimension; p++) {
      //   double whiteNoise = distribution(generator);
      //   for(int j=0; j<trajectories[i].size(); j++) {
      //     for(int k=0; k<ppc; k++) {
      //       trajectories[i][j][k][p] += pos_diffs[i][p] + whiteNoise;
      //     }
      //   }
      // }

      // Create objective (min energy + reach goal)
      std::vector<double> pieceDurations(curve_count, hor / curve_count);

      // y are our control points (decision variable)
      // Vector y(numVars);
      Matrix y(problem_dimension, 8 * curve_count);

      // initialize y with previous solution
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            y(p, j * ppc + k) = trajectories[i][j][k][p];
          }
        }
      }

      struct endCloseToData
      {
        size_t piece;
        double lambda;
        Vector value;
      };
      std::vector<endCloseToData> endCloseToObjectives;

      struct hyperplaneData
      {
        size_t piece;
        Vector normal;
        double dist;
      };
      std::vector<hyperplaneData> hyperplaneConstraints;


      // check if those trajectories are collision-free w/ respect to the environment
      bool planned_traj_occupied = og.occupied(trajectories[i]);
      vectoreuc goalPos = original_trajectories[i].neval(min(ct+hor, total_times[i]), 0);

      // only considers static obstacles, not other robots (intentionally)
      bool goal_occupied = og.occupied(goalPos[0], goalPos[1]);

      // check if the first curve will violate voronoi constraints
      bool voronoi_violated = false;
      for(int j=0; j<voronoi_hyperplanes.size() && !voronoi_violated; j++) {
        hyperplane& plane = voronoi_hyperplanes[j];

        Vector normal(problem_dimension);
        normal << plane.normal[0], plane.normal[1];

        for(int k=0; k<ppc; k++) {
          vectoreuc pt(2);
          pt[0] = y(0, k);
          pt[1] = y(1, k);
          if (pt.dot(plane.normal) > plane.distance) {
            voronoi_violated = true;
            break;
          }
        }
      }

      bool discretePath = false;
      if (planned_traj_occupied || goal_occupied || voronoi_violated) {
        // the trajectory is now on top of an obstacle => discrete re-planning!


        std::vector< std::pair<double, double> > otherRobots;
        for (int j = 0; j < original_trajectories.size(); ++j) {
          if (i != j) {
            otherRobots.emplace_back(std::make_pair(positions[j][0], positions[j][1]));
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
          if (!og.idx_occupied(goalIdx)) {

            // check of occupied by another robot
            bool occupiedByOtherRobot = false;
            std::pair<double, double> coord = og.get_coordinates(goalIdx);
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

        OG::index startIdx = og.get_index(positions[i][0], positions[i][1]);
        discreteSearch::State start(startIdx.i, startIdx.j, OG::direction::NONE);

        discreteSearch::Environment env(og, otherRobots, robot_radius, goal);

        libSearch::AStar<discreteSearch::State, discreteSearch::Action, int, discreteSearch::Environment> astar(env);
        libSearch::PlanResult<discreteSearch::State, discreteSearch::Action, int> solution;

        t_start_a_star = Time::now();
        bool success = astar.search(start, solution);
        t_end_a_star = Time::now();

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
            corners.emplace_back(std::make_pair(positions[i][0], positions[i][1]));

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

            // update cellBasedObstacles with robot positions, such that svmSeparator take other robots into account
            for (int j = 0; j < original_trajectories.size(); ++j) {
              if (i != j) {
                // og.set_occupied(positions[j][0], positions[j][1]);

                obstacle2D o;
                vectoreuc pt(problem_dimension);
                pt[0] = positions[j][0] - robot_radius;
                pt[1] = positions[j][1] - robot_radius;
                o.add_pt(pt);
                pt[0] = positions[j][0] - robot_radius;
                pt[1] = positions[j][1] + robot_radius;
                o.add_pt(pt);
                pt[0] = positions[j][0] + robot_radius;
                pt[1] = positions[j][1] + robot_radius;
                o.add_pt(pt);
                pt[0] = positions[j][0] + robot_radius;
                pt[1] = positions[j][1] - robot_radius;
                o.add_pt(pt);
                o.convex_hull();
                o.ch_planes();
                cell_based_obstacles.push_back(o);
              }
            }

            // find separating hyperplanes between those lines and all (nearby) obstacles
            // the first line is given by points corners[0] and corners[1]; second line by corners[1] and corners[2] etc.

            // if there are not enough corners for the curves, repeat the last corners multiple times
            // i.e., the constraints are the same for the last curves
            int discrete_curve_count = corners.size() - 1;
            // while (curve_count > corners.size() - 1) {
            //   corners.push_back(corners[corners.size() - 2]);
            // }
            t_start_svm = Time::now();
            size_t hpidx = 0;
            for (size_t j = 0; j < discrete_curve_count && j < curve_count; ++j) {
              svm.reset_pts();
              vectoreuc pt(2);
              pt[0] = corners[j].first;
              pt[1] = corners[j].second;
              svm.add_pt(pt);

              pt[0] = corners[j+1].first;
              pt[1] = corners[j+1].second;
              svm.add_pt(pt);

              // gives us one hyperplane per obstacle
              vector<hyperplane> hyperplanes = svm._2_4_seperate();
              std::cout << "hyperplanes: " << hyperplanes.size() << std::endl;

              do {
                // add hyperplane constraints
                for (auto& plane : hyperplanes) {
                  Vector normal(problem_dimension);
                  normal << plane.normal[0], plane.normal[1];
                  // cb.addHyperplane(j, normal, plane.distance);
                  hyperplaneConstraints.push_back({j, normal, plane.distance});

                  output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[0]); //normal first
                  output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[1]);//normal seconds
                  output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.distance); //distance
                  output_json["hyperplanes"][output_iter][i][hpidx].push_back(j);
                  ++hpidx;
                }
                if (j >= discrete_curve_count - 1) {
                  ++j;
                }
              } while (j >= discrete_curve_count && j < curve_count);
            }
            t_end_svm = Time::now();

            // uniformily distribute the control points on the line segments as initial guess

            // e.g., if curve_count = 3 and discrete_curve_count = 1, then last_discrete_curve_occupancy = 3,
            //       because the discrete curve is shared between 3 continuous curves
            int last_discrete_curve_occupancy = curve_count - std::min(discrete_curve_count, curve_count) + 1;
            for (size_t j = 0; j < curve_count; ++j) {
              if (j < discrete_curve_count - 1) {
                // single occupation
                for(int k=0; k<ppc; k++) {
                  y(0, j * ppc + k) = linearInterpolation(corners[j].first, corners[j+1].first, k, ppc);
                  y(1, j * ppc + k) = linearInterpolation(corners[j].second, corners[j+1].second, k, ppc);
                }
                double curve_length = sqrt(pow(corners[j].first - corners[j+1].first, 2) +
                                           pow(corners[j].second - corners[j+1].second, 2));
                pieceDurations[j] = std::max(curve_length / total_discrete_path_length * discrete_horizon, 1.5 * dt);
              } else {
                // double occupation
                int cornersIdx = discrete_curve_count - 1;
                int idx = j - discrete_curve_count + 1;
                for(int k=0; k<ppc; k++) {
                  y(0, j * ppc + k) = linearInterpolation(corners[cornersIdx].first, corners[cornersIdx+1].first, k + idx * ppc, ppc * last_discrete_curve_occupancy);
                  y(1, j * ppc + k) = linearInterpolation(corners[cornersIdx].second, corners[cornersIdx+1].second, k + idx * ppc, ppc * last_discrete_curve_occupancy);
                }
                double curve_length = sqrt(pow(corners[cornersIdx].first - corners[cornersIdx+1].first, 2) +
                                           pow(corners[cornersIdx].second - corners[cornersIdx+1].second, 2)) / last_discrete_curve_occupancy;
                pieceDurations[j] = std::max(curve_length / total_discrete_path_length * discrete_horizon, 1.5 * dt);
              }
            }

            // force end of trajectory to be at discrete end point
            // TODO: factor?
            Vector endPosition(2);
            endPosition << y(0, 8 * curve_count - 1) , y(1, 8 * curve_count - 1);

            endCloseToObjectives.push_back({(size_t)curve_count - 1, 100, endPosition});
            // ob.endCloseTo(curve_count - 1, 100, endPosition);
            // cb.addConstraintEnd(curve_count - 1, 0, endPosition);

            for (int j = 0; j < original_trajectories.size(); ++j) {
              if (i != j) {
                cell_based_obstacles.pop_back();
              }
            }

            discretePath = true;

          } else {
            std::cerr << "discrete planning NOT successful!" << ct << std::endl;
          }

      }

      if (!discretePath) {
        // The old trajectories will not collide with a (static) obstacle

        // TODO 3: * Split last trajectory up to horizon into uniform pieces (curve_count pieces)
        //         * find separating hyperplanes between those trajectories and all obstacles
        //         * add hyperplanes as constraints (per piece)
#if 1
        t_start_a_star = t_end_a_star = Time::now();

        t_start_svm = Time::now();
        size_t hpidx = 0;
        for (size_t j = 0; j < curve_count ; ++j) {
          svm.reset_pts();

          for(int k=0; k<ppc; k++) {
            svm.add_pt(trajectories[i][j][k]);
          }

          vector<hyperplane> hyperplanes = svm._8_4_seperate();
          std::cout << "hyperplanes2: " << hyperplanes.size() << std::endl;

          for (auto& plane : hyperplanes) {
            Vector normal(problem_dimension);
            normal << plane.normal[0], plane.normal[1];
            hyperplaneConstraints.push_back({j, normal, plane.distance});
            // cb.addHyperplane(j, normal, plane.distance);

            output_json["hyperplanes"][output_iter][i][hpidx].clear();
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[0]); //normal first
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[1]);//normal seconds
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.distance); //distance
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(j);
            ++hpidx;
          }
        }
        t_end_svm = Time::now();
#endif

        double factor = 1.0f / curve_count;
        for (size_t j = 1; j < curve_count; ++j) {
          vectoreuc OBJPOS = original_trajectories[i].neval(min(ct+hor * (j+1)/(double)curve_count, total_times[i]), 0);
          Vector targetPosition(problem_dimension);
          targetPosition << OBJPOS[0], OBJPOS[1];
          // ob.endCloseTo(j, factor * (j+1), targetPosition);
          endCloseToObjectives.push_back({j, factor * (j+1), targetPosition});
        }

      }

      output_json["controlpoints_guessed"][output_iter][i].clear();
      for (size_t j = 0; j < curve_count; ++j) {
        for(int k=0; k<ppc; k++) {
          std::vector<double> crds;
          crds.push_back(y(0, j * ppc + k));
          crds.push_back(y(1, j * ppc + k));
          output_json["controlpoints_guessed"][output_iter][i].push_back(crds);
        }
      }

      t_start_qp = Time::now();

      do { // loop for temporal scaling

      // construct QP matrices
      ObjectiveBuilder ob(problem_dimension, pieceDurations);

      double remaining_time = std::max(total_times[i] - ct, 3 * dt * 1.5);
      double factor = 1.0f / remaining_time;
      ob.minDerivativeSquared(1 * factor, 0 * factor, 5e-3 * factor, 0 * factor);

      for (const auto& o : endCloseToObjectives) {
        ob.endCloseTo(o.piece, o.lambda, o.value);
      }

      // lower and upper bound for decision variables (i.e., workspace)
      const size_t numVars = problem_dimension * 8 * curve_count;
      Vector lb(numVars);
      lb.setConstant(-10);
      Vector ub(numVars);
      ub.setConstant(10);


      // constraint matrix A
      ConstraintBuilder cb(problem_dimension, pieceDurations);

      // initial point constraints

        // position (with added noise)
      if(max_initial_point_degree >= 0) {
        Vector value(problem_dimension);
        value << positions[i][0] + distribution(generator), positions[i][1] + distribution(generator);
        cb.addConstraintBeginning(0, 0, value); // Position
      }

        // higher order constraints
      if(max_initial_point_degree >= 1) {
        Vector value(problem_dimension);
        value << velocities[i][0], velocities[i][1];
        cb.addConstraintBeginning(0, 1, value); // Velocity
      }

      if(max_initial_point_degree >= 2) {
        Vector value(problem_dimension);
        value << accelerations[i][0], accelerations[i][1];
        cb.addConstraintBeginning(0, 2, value); // Acceleration
      }

      // for (size_t d = 1; d <= max_initial_point_degree; ++d) {
      //   vectoreuc val = trajectories[i].neval(dt, d);
      //   Vector value(problem_dimension);
      //   value << val[0], val[1];
      //   cb.addConstraintBeginning(0, d, value);
      // }

      // continuity constraints
      for (size_t i = 0; i < curve_count - 1; ++i) {
        for (size_t c = 0; c <= max_continuity; ++c) {
          cb.addContinuity(i, c);
        }
      }

      // if time is nearly up, make sure we stop at the end
      // if (ct + hor >= total_times[i]) {
      //   Vector zeroVec(problem_dimension);
      //   zeroVec.setZero();
      //   for (size_t c = 1; c <= max_continuity; ++c) {
      //     cb.addConstraintEnd(curve_count - 1, c, zeroVec);
      //   }
      // }

      // voronoi constraints (for the first curve only)
      for(int j=0; j<voronoi_hyperplanes.size(); j++) {
        hyperplane& plane = voronoi_hyperplanes[j];

        Vector normal(problem_dimension);
        normal << plane.normal[0], plane.normal[1];
        cb.addHyperplane(0, normal, plane.distance);
      }

      // hyperplane constraints

      for (const auto& hpc : hyperplaneConstraints) {
        cb.addHyperplane(hpc.piece, hpc.normal, hpc.dist);
      }

      const size_t numConstraints = cb.A().rows();

      std::cout << "piece durations: ";
      for (const auto& pd : pieceDurations) {
        std::cout << pd << ",";
      }
      std::cout << std::endl;


#if QP_SOLVER == QP_SOLVER_OSQP
      ////
      // Problem settings
      OSQPSettings settings;
      osqp_set_default_settings(&settings);
      // settings.polish = 1;
      // settings.eps_abs = 1.0e-4;
      // settings.eps_rel = 1.0e-4;
      // settings.max_iter = 20000;
      // settings.verbose = false;

      // Structures
      typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixCM;
      MatrixCM H_CM = ob.H();
      std::vector<c_int> rowIndicesH(numVars * numVars);
      std::vector<c_int> columnIndicesH(numVars + 1);
      for (size_t c = 0; c < numVars; ++c) {
        for (size_t r = 0; r < numVars; ++r) {
          rowIndicesH[c * numVars + r] = r;
        }
        columnIndicesH[c] = c * numVars;
      }
      columnIndicesH[numVars] = numVars * numVars;

      MatrixCM A_CM = cb.A();
      std::vector<c_int> rowIndicesA(numConstraints * numVars);
      std::vector<c_int> columnIndicesA(numVars + 1);
      for (size_t c = 0; c < numVars; ++c) {
        for (size_t r = 0; r < numConstraints; ++r) {
          rowIndicesA[c * numConstraints + r] = r;
        }
        columnIndicesA[c] = c * numConstraints;
      }
      columnIndicesA[numVars] = numConstraints * numVars;


      OSQPData data;
      data.n = numVars;
      data.m = numConstraints;
      data.P = csc_matrix(numVars, numVars, numVars * numVars, H_CM.data(), rowIndicesH.data(), columnIndicesH.data());
      data.q = const_cast<double*>(ob.g().data());
      data.A = csc_matrix(numConstraints, numVars, numConstraints * numVars, A_CM.data(), rowIndicesA.data(), columnIndicesA.data());
      data.l = const_cast<double*>(cb.lbA().data());
      data.u = const_cast<double*>(cb.ubA().data());

      OSQPWorkspace* work = osqp_setup(&data, &settings);

      // Solve Problem
      osqp_warm_start_x(work, y.data());
      osqp_solve(work);

      if (work->info->status_val != OSQP_SOLVED) {
        std::stringstream sstr;
        sstr << "Couldn't solve QP! @ " << ct << " robot " << i;
        // throw std::runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }

      // work->solution->x
      for (size_t i = 0; i < numVars; ++i) {
        y(i) = work->solution->x[i];
      }
      // for(int j=0; j<trajectories[i].size(); j++) {
      //   for(int k=0; k<ppc; k++) {
      //     for(int p=0; p<problem_dimension; p++) {
      //       trajectories[i][j][k][p] = work->solution->x[p * numVars + j * ppc + k];
      //     }
      //   }
      // }

      // Clean workspace
      osqp_cleanup(work);
#endif
#if QP_SOLVER == QP_SOLVER_QPOASES
      // const size_t numVars = cb.A().columns();

      qpOASES::QProblem qp(numVars, numConstraints, qpOASES::HST_SEMIDEF);

      qpOASES::Options options;
      options.setToMPC(); // maximum speed; others: setToDefault() and setToReliable()
      // options.setToDefault();
      // options.setToReliable();
      options.printLevel = qpOASES::PL_LOW; // only show errors
      // options.boundTolerance =  1e5;//1e-6;
      qp.setOptions(options);

      // The  integer  argument nWSR specifies  the  maximum  number  of  working  set
      // recalculations to be performed during the initial homotopy (on output it contains the number
      // of  working  set  recalculations  actually  performed!)
      qpOASES::int_t nWSR = 10000;

      // std::cout << "H: " << ob.H() << std::endl;
      // std::cout << "A: " << cb.A() << std::endl;

      // auto t0 = Time::now();
      qpOASES::returnValue status = qp.init(
        ob.H().data(),
        ob.g().data(),
        cb.A().data(),
        lb.data(),
        ub.data(),
        cb.lbA().data(),
        cb.ubA().data(),
        nWSR,
        /*cputime*/ NULL,
        y.data());

      qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(status);
      if (simpleStatus != 0) {
        std::stringstream sstr;
        sstr << "Couldn't solve QP! @ " << ct << " robot " << i;

        Eigen::Map<Matrix> yVec(y.data(), numVars, 1);

        auto constraints = cb.A() * yVec;
        // std::cout << "constraints: " << constraints << std::endl;
        for (size_t c = 0; c < numConstraints; ++c) {
          if (cb.lbA()(c) > constraints(c) || constraints(c) > cb.ubA()(c)) {
            std::cout << "Constraint " << c << " violated!" << std::endl;
          }
        }
        // throw std::runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;

        y.setZero();

        continue;
      }

      qp.getPrimalSolution(y.data());

      std::cout << "status: " << status << std::endl;
      std::cout << "objective: " << qp.getObjVal() << std::endl;
      // std::cout << "y: " << y << std::endl;
#endif
      t_end_qp = Time::now();
      // Update trajectories with our solution
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            trajectories[i][j][k][p] = y(p, j * ppc + k);
          }
        }
      }

      // temporal scaling

      // // double desired_time_per_curve = hor / curve_count;
      // remaining_time = std::max(total_times[i] - ct, 3 * dt * 1.5);

      // for (size_t j = 0; j < curve_count; ++j) {
      //   trajectories[i][j].duration = /*std::max(*/std::min(hor, remaining_time) / curve_count;//, 1.5 * dt);
      // }
      // For robot-robot safety make sure that the first piece (with voronoi constraints) lasts until the next planning cycle
      // trajectories[i][0].duration = std::max(trajectories[i][0].duration, 1.5 * dt);

      // scale until slow enough

      for (size_t j = 0; j < curve_count; ++j) {
        trajectories[i][j].duration = pieceDurations[j];
        std::cout << "traj " << j << " " << trajectories[i][j].duration << std::endl;
      }

      double max_velocity = -1;
      double max_acceleration = -1;
      for (double t = 0; t < trajectories[i].duration(); t+=dt/10.0) {
        double velocity = trajectories[i].neval(t, 1).L2norm();
        double acceleration = trajectories[i].neval(t, 2).L2norm();
        max_velocity = std::max(max_velocity, velocity);
        max_acceleration = std::max(max_acceleration, acceleration);
      }
      std::cout << "max_vel " << max_velocity << std::endl;
      std::cout << "max_acc " << max_acceleration << std::endl;
      if (   max_velocity > 2.0
          || max_acceleration > 4.0) {
        for (auto& pd : pieceDurations) {
          pd *= 1.2;
        }
        std::cerr << "SCALING " << pieceDurations[0] << " " << accelerations[i] << std::endl;
      } else {
          break;
      }

      // for (size_t j = 0; j < curve_count; ++j) {
      //   std::cout << "traj " << j << " " << trajectories[i][j].duration << std::endl;
      // }




      } while (true); // temporal scaling


      auto t1 = Time::now();
      fsec fs = t1 - t_start;
      ms d = chrono::duration_cast<ms>(fs);
      total_time_for_opt += d.count();
      total_count_for_opt++;
      cout << "optimization time: " << d.count() << "ms" << endl;
      outStats << "," << d.count()
               << "," << chrono::duration_cast<ms>(t_end_a_star - t_start_a_star).count()
               << "," << chrono::duration_cast<ms>(t_end_svm - t_start_svm).count()
               << "," << chrono::duration_cast<ms>(t_end_qp - t_start_qp).count();

      for(double t = 0; t<=trajectories[i].duration(); t+=printdt) {
        vectoreuc ev = trajectories[i].eval(t);
        //cout << ev << endl;
        output_json["planned_trajs"][output_iter][i]["x"].push_back(ev[0]);
        output_json["planned_trajs"][output_iter][i]["y"].push_back(ev[1]);
      }

      for(int j=0; j<curve_count; j++) {
        for(int p=0; p<ppc; p++) {
          output_json["controlpoints"][output_iter][i].push_back(trajectories[i][j][p].crds);
          //cout << trajectories[i][j][p] << endl;
        }
      }
#if 0
      for (int j = 0; j < original_trajectories.size(); ++j) {
        if (i != j) {
          cell_based_obstacles.pop_back();
        }
      }
#endif
    }
    //cout << "------" << endl;
    vectoreuc vec;
    for(int v = 0; v < steps_per_cycle; v++) {
      for(int i=0; i<trajectories.size(); i++) {
        vec = trajectories[i].neval(v*printdt, 0);
        //cout << vec << endl;
        output_json["points"][output_iter].push_back(vec.crds);

        vec = trajectories[i].neval(v*printdt, 1);
        output_json["velocities"][output_iter].push_back(vec.crds);

        vec = trajectories[i].neval(v*printdt, 2);
        output_json["accelerations"][output_iter].push_back(vec.crds);
      }
      output_iter++;
    }



    for(int i=0; i<trajectories.size(); i++) {
      pos_diffs[i] = positions[i];
      positions[i] = trajectories[i].neval(dt, 0);
      pos_diffs[i] = positions[i] - pos_diffs[i];
      velocities[i] = trajectories[i].neval(dt, 1);
      //cout << "vel wanted: " << velocities[i] << endl;
      accelerations[i] = trajectories[i].neval(dt, 2);
    }

    everyone_reached = true;
    for(int i=0; i<trajectories.size(); i++) {
      double diff = (original_trajectories[i].eval(total_times[i]) - positions[i]).L2norm();
      if(diff > 0.1) {
        everyone_reached = false;
        break;
      }
    }

    outStats << std::endl;

  }



  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;
  out << std::setw(2) << output_json << endl;

  out.close();
  return 0;

}
