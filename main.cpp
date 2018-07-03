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

#include "qp_optimize.h"

// different solvers
#include <qpOASES.hpp>
#include "osqp.h"
#include <bspline.h>

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

  nlohmann::json compareStats;

  nlohmann::json jsn = nlohmann::json::parse(cfg);

  string initial_trajectories_path = jsn["trajectories"];
  float scale_traj = jsn["scale_traj"];
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
  const double lambda_hyperplanes = jsn["lambda_hyperplanes"];
  const double lambda_min_der = jsn["lambda_min_der"];
  const double lambda_min_der_vel = jsn["lambda_min_der_vel"];
  const double lambda_min_der_acc = jsn["lambda_min_der_acc"];
  const double lambda_min_der_jerk = jsn["lambda_min_der_jerk"];
  const double lambda_min_der_snap = jsn["lambda_min_der_snap"];
  const double scaling_multiplier = jsn["scaling_multiplier"];
  const double additional_time = jsn["additional_time"];


  bool enable_voronoi = jsn["enable_voronoi"];

  string alg = jsn["algorithm"];

  nlohmann::json output_json;


  srand(time(NULL));

  const double stoppingDistance = v_max * v_max / (2 * a_max);
  std::cout << "stoppingDistance: " << stoppingDistance << std::endl;


  vector<trajectory> original_trajectories;


  for (auto & p : fs::directory_iterator(initial_trajectories_path)) {
    csv::Parser file(p.path().string());
    trajectory trj;
    for(int i=0; i<file.rowCount(); i++) {
      double duration = stod(file[i][0]);
      curve crv(duration * scale_traj, problem_dimension);
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


  vector<splx::BSpline> trajectories;

  for(int i=0; i<original_trajectories.size(); i++) {
    std::vector<splx::Vec> cpts;
    splx::Vec cpt(problem_dimension);
    cpt(0) = original_trajectories[i][0][0][0];
    cpt(1) = original_trajectories[i][0][0][1];
    for(int j=0; j<curve_count; j++) {
      for(int k = 0; k < original_trajectories[i][j].size(); k++) {
        cpts.push_back(cpt);
      }
    }
    trajectories.push_back(splx::BSpline(max_continuity+1, problem_dimension, 0, hor, cpts));
  }

  /*vector<vectoreuc> pos_diffs(trajectories.size());*/
  vectoreuc zerovec(2);
  zerovec.zero();
  /*for(int i=0; i<trajectories.size(); i++) {
    pos_diffs[i] = zerovec;
  }*/

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
        o.ch_planes(/*shift*/0);
        cell_based_obstacles.push_back(o);
      }
    }
  }
  cout << "obs count: " << cell_based_obstacles.size() << endl;
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


  vector<splx::Vec> positions(trajectories.size());
  vector<splx::Vec> velocities(trajectories.size());
  vector<splx::Vec> accelerations(trajectories.size());
  // vector<vectoreuc> jerks(trajectories.size());
  // vector<vectoreuc> snaps(trajectories.size());

  double printdt = jsn["print_dt"];

  output_json["dt"] = printdt;
  output_json["robot_radius"] = robot_radius;
  output_json["number_of_robots"] = original_trajectories.size();
  compareStats["number_of_robots"] = original_trajectories.size();
  int steps_per_cycle = (dt / printdt)+0.5;
  int output_iter = 0;


  double total_time_for_opt = 0;
  int total_count_for_opt = 0;



  for(int i=0; i<trajectories.size(); i++) {
    positions[i] = trajectories[i].eval(0, 0);
    cout << "init pos: " << positions[i]<< endl;
    velocities[i] = trajectories[i].eval(0, 1);
    cout << "init vel: " << velocities[i]<< endl;
    accelerations[i] = trajectories[i].eval(0, 2);
    cout << "init acc: " << accelerations[i]<< endl;
    // jerks[i] = trajectories[i].neval(0, 3);
    // cout << "init jerk: " << jerks[i]<< endl;
    // snaps[i] = trajectories[i].neval(0, 4);
    // cout << "init snap: " << snaps[i]<< endl;
  }

  // add some pertubation at the beginning for first robot
  // positions[0][0] += 0.2;
  // positions[0][1] -= 0.4;

  for(int i=0; i<original_trajectories.size(); i++) {
    for(double t=0; t<=total_t; t+=printdt) {
      vectoreuc eu = original_trajectories[i].eval(t);
      output_json["originals"][i]["x"].push_back(eu[0]);
      output_json["originals"][i]["y"].push_back(eu[1]);
    }
  }

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

  vector<double> max_opt_times(original_trajectories.size(), -1);
  vector<double> average_opt_times(original_trajectories.size(), 0);
  vector<int> opt_counts(original_trajectories.size(), 0);
  vector<int> obstacle_crash_counts(original_trajectories.size(), 0);
  vector<int> obstacle_no_crash_counts(original_trajectories.size(), 0);
  vector<int> robot_crash_counts(original_trajectories.size(), 0);
  vector<int> robot_no_crash_counts(original_trajectories.size(), 0);

  for(double ct = 0; ct <= total_t + additional_time/*+ 5*/ && !everyone_reached ; ct+=dt) {

    outStats << ct;

    cerr << ct << " / " << total_t << endl;

    for(int i=0; i<original_trajectories.size(); i++ ) {
      cerr << "traj " << i << " start " << ct << " / " << total_times[i] << endl;
      trajectories[i].m_b = hor;
      trajectories[i].generateUniformKnotVector();
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
        voronoi_hyperplanes = voronoi(positions,i, robot_radius/* + stoppingDistance*/);
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
      double planning_horizon = std::max(hor, curve_count * dt * 1.5);
      std::vector<double> pieceDurations(curve_count, planning_horizon / curve_count);

      // y are our control points (decision variable)
      // Vector y(numVars);


      struct endCloseToData
      {
        double time;
        double lambda;
        Vector value;
      };
      std::vector<endCloseToData> endCloseToObjectives;

      struct hyperplaneData
      {
        unsigned int from_pt;
        unsigned int to_pt;
        Vector normal;
        double dist;
      };
      std::vector<hyperplaneData> hyperplaneConstraints;


      // check if those trajectories are collision-free w/ respect to the environment
      bool planned_traj_occupied = og.occupied(trajectories[i], robot_radius);
      vectoreuc goalPos = original_trajectories[i].neval(min(ct+hor, total_times[i]), 0);

      // only considers static obstacles, not other robots (intentionally)
      // bool goal_occupied = og.occupied(goalPos[0], goalPos[1], robot_radius);

      // check if the first curve will violate voronoi constraints
      bool voronoi_violated = false;
      auto aff = trajectories[i].affectingPoints(0, dt);
      unsigned int jl = aff.first;
      unsigned int jr = aff.second;
      for(int j=0; j<voronoi_hyperplanes.size() && !voronoi_violated; j++) {
        hyperplane& plane = voronoi_hyperplanes[j];

        Vector normal(problem_dimension);
        normal << plane.normal[0], plane.normal[1];

        for(int k=jl; k<=jr; k++) {
          const splx::Vec& vec = trajectories[i].getCP(k);
          vectoreuc pt(2);
          pt[0] = vec(0);
          pt[1] = vec(1);
          if (pt.dot(plane.normal) > plane.distance) {
            voronoi_violated = true;
            break;
          }
        }
      }

      // check if original trajectory is occupied
      bool original_traj_occupied = og.occupied(original_trajectories[i], robot_radius, ct, min(ct+hor, total_times[i]));

      bool discretePath = false;
      if (planned_traj_occupied || /*goal_occupied ||*/ voronoi_violated || original_traj_occupied) {
        // the trajectory is now on top of an obstacle => discrete re-planning!
        bool discretePlanningNeeded = true;

        std::vector< std::pair<double, double> > otherRobots;
        for (int j = 0; j < original_trajectories.size(); ++j) {
          if (i != j) {
            otherRobots.emplace_back(std::make_pair(positions[j](0), positions[j](1)));
          }
        }

        for (const auto& otherRobot : otherRobots) {

          if (   fabs(positions[i](0) - otherRobot.first) <= 2 * robot_radius
              && fabs(positions[i](1) - otherRobot.second) <= 2 * robot_radius) {
            std::cerr << "Other robot too close! Skip discrete planning" << std::endl;
            discretePlanningNeeded = false;
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
              // double distSq = pow(otherRobot.first - coord.first, 2) + pow(otherRobot.second - coord.second, 2);
              // if (distSq < pow(2 * robot_radius, 2)) {
              //   occupiedByOtherRobot = true;
              //   break;
              // }
              if (   fabs(positions[i](0) - otherRobot.first) <= 2 * robot_radius
                  && fabs(positions[i](1) - otherRobot.second) <= 2 * robot_radius) {
                occupiedByOtherRobot = true;
                break;
              }

              if (   fabs(coord.first - otherRobot.first) <= 2 * robot_radius
                  && fabs(coord.second - otherRobot.second) <= 2 * robot_radius) {
                occupiedByOtherRobot = true;
                break;
              }
            }

            bool occupiedByObstacle = og.occupied(goalPos[0], goalPos[1], robot_radius);


            if (!occupiedByObstacle && !occupiedByOtherRobot) {
              goal.x = goalIdx.i;
              goal.y = goalIdx.j;
              discrete_horizon = t - ct;
              break;
            }
          }
        }

        if (goal.x == -1) {
          std::cerr << "Couldn't find unoccupied space on original trajectory!" << ct << std::endl;
          discretePlanningNeeded = false;
        }

        if (discretePlanningNeeded) {
          std::cout << "discrete horizon: " << discrete_horizon << std::endl;

          OG::index startIdx = og.get_index(positions[i](0), positions[i](1));
          discreteSearch::State start(startIdx.i, startIdx.j, OG::direction::NONE);

          // otherRobots.clear();
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
                pt[0] = positions[j](0) - robot_radius;
                pt[1] = positions[j](1) - robot_radius;
                o.add_pt(pt);
                pt[0] = positions[j](0) - robot_radius;
                pt[1] = positions[j](1) + robot_radius;
                o.add_pt(pt);
                pt[0] = positions[j](0) + robot_radius;
                pt[1] = positions[j](1) + robot_radius;
                o.add_pt(pt);
                pt[0] = positions[j](0) + robot_radius;
                pt[1] = positions[j](1) - robot_radius;
                o.add_pt(pt);
                o.convex_hull();
                o.ch_planes(/*shift*/0);
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


            trajectories[i].clearControlPoints();

            size_t hpidx = 0;
            for (size_t j = 0; j < discrete_curve_count && j < curve_count; ++j) {
              splx::Vec fromvec(2);
              fromvec(0) = corners[j].first;
              fromvec(1) = corners[j].second;
              splx::Vec tovec(2);
              tovec(0) = corners[j+1].first;
              tovec(1) = corners[j+1].second;
              std::pair<unsigned int, unsigned int> effectrange = trajectories[i]
                    .interpolateEndAtTo(fromvec, tovec, max(5, ppc - (int)trajectories[i].m_degree));
            }

            trajectories[i].generateUniformKnotVector();

            for(unsigned int j = 0; j < trajectories[i].m_controlPoints.size() - trajectories[i].m_degree; j++) {
              svm.reset_pts();
              vectoreuc vec(2);
              for(unsigned int k = j; k <= j+trajectories[i].m_degree; k++) {
                vec[0] = trajectories[i].m_controlPoints[k](0) - robot_radius;
                vec[1] = trajectories[i].m_controlPoints[k](1) - robot_radius;
                svm.add_pt(vec);
                vec[0] = trajectories[i].m_controlPoints[k](0) - robot_radius;
                vec[1] = trajectories[i].m_controlPoints[k](1) + robot_radius;
                svm.add_pt(vec);
                vec[0] = trajectories[i].m_controlPoints[k](0) + robot_radius;
                vec[1] = trajectories[i].m_controlPoints[k](1) - robot_radius;
                svm.add_pt(vec);
                vec[0] = trajectories[i].m_controlPoints[k](0) + robot_radius;
                vec[1] = trajectories[i].m_controlPoints[k](1) + robot_radius;
                svm.add_pt(vec);
              }
              cout << "discrete svm" << endl;
              vector<hyperplane> sep = svm._16_4_seperate();
              for(unsigned int k = 0; k < sep.size(); k++) {
                auto& plane = sep[k];
                Vector normal(problem_dimension);
                double min_dist = std::numeric_limits<double>::infinity();
                for(unsigned int p = j; p <= j + trajectories[i].m_degree; p++) {
                  vectoreuc vv(2);
                  vv[0] = trajectories[i].getCP(p)(0);
                  vv[1] = trajectories[i].getCP(p)(1);
                  min_dist = min(min_dist, plane.dist(vv));
                }

                double shift_amount = min(robot_radius * sqrt(2), min_dist - 0.00001);

                normal << plane.normal[0], plane.normal[1];
                output_json["hyperplanes"][output_iter][i][hpidx] = vector<double>();
                output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[0]);
                output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[1]);
                output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.distance - robot_radius);
                hpidx++;
                hyperplaneConstraints.push_back({j, j+trajectories[i].m_degree, normal, plane.distance - robot_radius});
              }
            }
            t_end_svm = Time::now();


            Vector crnr(2);
            crnr <<corners[min((int)corners.size()-1, curve_count)].first, corners[min((int)corners.size()-1, curve_count)].second;
            endCloseToObjectives.push_back({trajectories[i].m_b, 100, crnr});
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

      }

      if (!discretePath) {
        // The old trajectories will not collide with a (static) obstacle

        // TODO 3: * Split last trajectory up to horizon into uniform pieces (curve_count pieces)
        //         * find separating hyperplanes between those trajectories and all obstacles
        //         * add hyperplanes as constraints (per piece)
        t_start_a_star = t_end_a_star = Time::now();

        t_start_svm = Time::now();
        size_t hpidx = 0;
        for (size_t j = 0; j < trajectories[i].m_controlPoints.size() - trajectories[i].m_degree ; ++j) {
          svm.reset_pts();

          vectoreuc pt(2);
          for(int k=j; k<=j+trajectories[i].m_degree; k++) {
            pt[0] = trajectories[i].getCP(k)(0) - robot_radius;
            pt[1] = trajectories[i].getCP(k)(1) - robot_radius;
            svm.add_pt(pt);
            pt[0] = trajectories[i].getCP(k)(0) - robot_radius;
            pt[1] = trajectories[i].getCP(k)(1) + robot_radius;
            svm.add_pt(pt);
            pt[0] = trajectories[i].getCP(k)(0) + robot_radius;
            pt[1] = trajectories[i].getCP(k)(1) - robot_radius;
            svm.add_pt(pt);
            pt[0] = trajectories[i].getCP(k)(0) + robot_radius;
            pt[1] = trajectories[i].getCP(k)(1) + robot_radius;
            svm.add_pt(pt);
          }

          cout << "continuous svm" << endl;
          vector<hyperplane> hyperplanes = svm._16_4_seperate();

          for (auto& plane : hyperplanes) {
            Vector normal(problem_dimension);

            double min_dist = std::numeric_limits<double>::infinity();
            for(unsigned int p = j; p <= j + trajectories[i].m_degree; p++) {
              vectoreuc vv(2);
              vv[0] = trajectories[i].getCP(p)(0);
              vv[1] = trajectories[i].getCP(p)(1);
              min_dist = min(min_dist, plane.dist(vv));
            }

            double shift_amount = min(robot_radius * sqrt(2), min_dist - 0.01);

            normal << plane.normal[0], plane.normal[1];
            output_json["hyperplanes"][output_iter][i][hpidx] = vector<double>();
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[0]);
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.normal[1]);
            output_json["hyperplanes"][output_iter][i][hpidx].push_back(plane.distance - shift_amount);
            hpidx++;
            hyperplaneConstraints.push_back({j, j+trajectories[i].m_degree, normal, plane.distance - shift_amount});
            // cb.addHyperplane(j, normal, plane.distance);
          }
        }
        t_end_svm = Time::now();

        const double step_u = (trajectories[i].m_b - trajectories[i].m_a) / curve_count;
        double factor = 0.5f / curve_count;
        for (size_t j = 1; j <= curve_count; ++j) {
          vectoreuc OBJPOS = original_trajectories[i].neval(min(ct+j*step_u, total_times[i]), 0);
          Vector targetPosition(problem_dimension);
          targetPosition << OBJPOS[0], OBJPOS[1];
          // ob.endCloseTo(j, factor * (j+1), targetPosition);
          endCloseToObjectives.push_back({trajectories[i].m_a + j*step_u, factor * (j+1), targetPosition});
        }

      }

      output_json["controlpoints_guessed"][output_iter][i].clear();
      for(int k=0; k<trajectories[i].m_controlPoints.size(); k++) {
        std::vector<double> crds(2);
        crds[0] = trajectories[i].m_controlPoints[k](0);
        crds[1] = trajectories[i].m_controlPoints[k](1);
        output_json["controlpoints_guessed"][output_iter][i].push_back(crds);
      }

      t_start_qp = Time::now();


      do { // loop for temporal scaling

      // construct QP matrices

      splx::QPMatrices qpm = trajectories[i].getQPMatrices();

      trajectories[i].extendQPDecisionConstraint(qpm, -10, 10);

      double remaining_time = std::max(total_times[i] - ct, curve_count * dt * 10.1);
      double factor = lambda_min_der / remaining_time;
      cout << factor*lambda_min_der_vel << endl;
      trajectories[i].extendQPIntegratedSquaredDerivative(qpm, 1, factor*lambda_min_der_vel);
      trajectories[i].extendQPIntegratedSquaredDerivative(qpm, 2, factor*lambda_min_der_acc);
      trajectories[i].extendQPIntegratedSquaredDerivative(qpm, 3, factor*lambda_min_der_jerk);
      trajectories[i].extendQPIntegratedSquaredDerivative(qpm, 4, factor*lambda_min_der_snap);

      for (const auto& o : endCloseToObjectives) {
        trajectories[i].extendQPPositionAt(qpm, o.time, o.value, o.lambda);
      }



      if(max_continuity >= 0) {
        Vector value(problem_dimension);
        value << positions[i](0) + distribution(generator), positions[i](1) + distribution(generator);
        trajectories[i].extendQPBeginningConstraint(qpm, 0, value);
      }

        // higher order constraints
      if(max_continuity >= 1) {
        Vector value(problem_dimension);
        value << velocities[i](0), velocities[i](1);
        trajectories[i].extendQPBeginningConstraint(qpm, 1, value);
      }

      if(max_continuity >= 2) {
        Vector value(problem_dimension);
        value << accelerations[i](0), accelerations[i](1);
        trajectories[i].extendQPBeginningConstraint(qpm, 2, value);
      }

      // voronoi constraints (for the first curve only)
      std::pair<unsigned int, unsigned int> voronoipoints = trajectories[i].affectingPoints(0, 2 * dt);
      for(int j=0; j<voronoi_hyperplanes.size(); j++) {
        hyperplane& plane = voronoi_hyperplanes[j];

        Vector normal(problem_dimension);
        normal << plane.normal[0], plane.normal[1];
        splx::Hyperplane hplane(2);
        hplane.normal() = normal;
        hplane.offset() = plane.distance;

        trajectories[i].extendQPHyperplaneConstraint(qpm, voronoipoints.first, voronoipoints.second, hplane);
      }

      // hyperplane constraints

      for (const auto& hpc : hyperplaneConstraints) {
        splx::Hyperplane hplane(2);
        hplane.normal() = hpc.normal;
        hplane.offset() = hpc.dist;
        trajectories[i].extendQPHyperplaneConstraint(qpm, hpc.from_pt, hpc.to_pt, hplane);
        //trajectories[i].extendQPHyperplanePenalty(qpm, hpc.from_pt, hpc.to_pt, hplane, 0.00001);
      }


      const size_t numConstraints = qpm.A.rows();
      const size_t numVars = qpm.x.rows();
      if (alg == "QPOASES") {
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
        Eigen::LLT<Eigen::MatrixXd> lltofH(qpm.H);
        // cout << qpm.H << endl;
        if(lltofH.info() == Eigen::NumericalIssue) {
          cout << "non psd" << endl;
        } else
          cout << "psd" << endl;
      qpOASES::real_t cputime = dt;
        qpOASES::returnValue status = qp.init(
          qpm.H.data(),
          qpm.g.data(),
          qpm.A.data(),
          qpm.lbX.data(),
          qpm.ubX.data(),
          qpm.lbA.data(),
          qpm.ubA.data(),
          nWSR,
          set_max_time ? &cputime : NULL,
          qpm.x.data());

          qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(status);
        if(simpleStatus != 0) {
          std::stringstream sstr;
          sstr << "Couldn't solve QP! @ " << ct << " robot " << i << " status: " << simpleStatus;
          //std::cerr << "reliable falied" << endl;
          // throw std::runtime_error(sstr.str());
          std::cerr << sstr.str() << std::endl;

        }


        qp.getPrimalSolution(qpm.x.data());

        std::cout << "status: " << status << std::endl;
        std::cout << "objective: " << qp.getObjVal() << std::endl;
        // std::cout << "y: " << y << std::endl;
      } else if (alg == "OSQP") {
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
        MatrixCM H_CM = qpm.H;
        std::vector<c_int> rowIndicesH(numVars * numVars);
        std::vector<c_int> columnIndicesH(numVars + 1);
        for (size_t c = 0; c < numVars; ++c) {
          for (size_t r = 0; r < numVars; ++r) {
            rowIndicesH[c * numVars + r] = r;
          }
          columnIndicesH[c] = c * numVars;
        }
        columnIndicesH[numVars] = numVars * numVars;

        MatrixCM A_CM = qpm.A;
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
        data.q = const_cast<double*>(qpm.g.data());
        data.A = csc_matrix(numConstraints, numVars, numConstraints * numVars, A_CM.data(), rowIndicesA.data(), columnIndicesA.data());
        data.l = const_cast<double*>(qpm.lbA.data());
        data.u = const_cast<double*>(qpm.ubA.data());

        OSQPWorkspace* work = osqp_setup(&data, &settings);

        // Solve Problem
        osqp_warm_start_x(work, qpm.x.data());
        osqp_solve(work);

        if (work->info->status_val != OSQP_SOLVED) {
          std::stringstream sstr;
          sstr << "Couldn't solve QP! @ " << ct << " robot " << i;
          // throw std::runtime_error(sstr.str());
          std::cerr << sstr.str() << std::endl;
        }

        // work->solution->x
        for (size_t i = 0; i < numVars; ++i) {
          qpm.x(i) = work->solution->x[i];
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
      } else {
        throw std::runtime_error("Unknown algorithms!");
      }

      t_end_qp = Time::now();
      // Update trajectories with our solution
      trajectories[i].loadControlPoints(qpm);

      // temporal scaling

      // // double desired_time_per_curve = hor / curve_count;
      // remaining_time = std::max(total_times[i] - ct, 3 * dt * 1.5);

      // for (size_t j = 0; j < curve_count; ++j) {
      //   trajectories[i][j].duration = /*std::max(*/std::min(hor, remaining_time) / curve_count;//, 1.5 * dt);
      // }
      // For robot-robot safety make sure that the first piece (with voronoi constraints) lasts until the next planning cycle
      // trajectories[i][0].duration = std::max(trajectories[i][0].duration, 1.5 * dt);

      // scale until slow enough


      double max_velocity = -1;
      double max_acceleration = -1;
      for (double t = trajectories[i].m_a; t < trajectories[i].m_b; t+=dt/10.0) {
        double velocity = trajectories[i].eval(t, 1).norm();
        double acceleration = trajectories[i].eval(t, 2).norm();
        max_velocity = std::max(max_velocity, velocity);
        max_acceleration = std::max(max_acceleration, acceleration);
      }
      std::cout << "max_vel " << max_velocity << std::endl;
      std::cout << "max_acc " << max_acceleration << std::endl;
      if (   max_velocity > v_max
          || max_acceleration > a_max) {
            trajectories[i].m_b*=scaling_multiplier;
            trajectories[i].generateUniformKnotVector();
            for (auto& o : endCloseToObjectives) {
              o.time *= scaling_multiplier;
            }
        std::cerr << "SCALING " << max_velocity << " " << max_acceleration << std::endl;
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
      average_opt_times[i]+=d.count();
      opt_counts[i]++;
      max_opt_times[i] = max((double)d.count(), max_opt_times[i]);
      outStats << "," << d.count()
               << "," << chrono::duration_cast<ms>(t_end_a_star - t_start_a_star).count()
               << "," << chrono::duration_cast<ms>(t_end_svm - t_start_svm).count()
               << "," << chrono::duration_cast<ms>(t_end_qp - t_start_qp).count();

      for(double t = trajectories[i].m_a; t<=trajectories[i].m_b; t+=printdt) {
        splx::Vec ev = trajectories[i].eval(t, 0);
        //cout << ev << endl;
        output_json["planned_trajs"][output_iter][i]["x"].push_back(ev(0));
        output_json["planned_trajs"][output_iter][i]["y"].push_back(ev(1));
      }

      for(int p=0; p<trajectories[i].m_controlPoints.size(); p++) {
        std::vector<double> crds(2);
        crds[0] = trajectories[i].m_controlPoints[p](0);
        crds[1] = trajectories[i].m_controlPoints[p](1);
        output_json["controlpoints"][output_iter][i].push_back(crds);
          //cout << trajectories[i][j][p] << endl;
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
    vectoreuc vec(2);
    for(int v = 0; v < steps_per_cycle; v++) {
      for(int i=0; i<trajectories.size(); i++) {
        splx::Vec vecc = trajectories[i].eval(v*printdt, 0);
        vec[0] = vecc(0);
        vec[1] = vecc(1);

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
        for(int p = 0; p<trajectories.size(); p++) {
          if(p==i) continue;
          splx::Vec vecc2 = trajectories[p].eval(v*printdt, 0);
          vectoreuc vec2(2);
          vec2[0] = vecc2(0);
          vec2[1] = vecc2(1);
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
        //cout << vec << endl;
        output_json["points"][output_iter].push_back(vec.crds);

        splx::Vec huehue;
        huehue = trajectories[i].eval(v*printdt, 1);
        std::vector<double> crds(2);
        crds[0] = huehue(0);
        crds[1] = huehue(1);
        output_json["velocities"][output_iter].push_back(crds);

        huehue = trajectories[i].eval(v*printdt, 2);
        crds[0] = huehue(0);
        crds[1] = huehue(1);
        output_json["accelerations"][output_iter].push_back(crds);
      }
      output_iter++;
    }



    for(int i=0; i<trajectories.size(); i++) {
      positions[i] = trajectories[i].eval(dt, 0);
      velocities[i] = trajectories[i].eval(dt, 1);
      accelerations[i] = trajectories[i].eval(dt, 2);
      // jerks[i] = trajectories[i].neval(dt, 3);
      // snaps[i] = trajectories[i].neval(dt, 4);
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
    double diff = (original_trajectories[i].eval(total_times[i]) - positions[i]).L2norm();
    robots_reached[i] = (diff <= 0.4);
  }

  compareStats["robots_reached"] = robots_reached;

  cout << compareStats << endl;

  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;

  ofstream out(outputfile);
  out << std::setw(2) << output_json << endl;

  out.close();

  ofstream compfile("compareStats_main.json");
  compfile << std::setw(2) << compareStats << endl;
  compfile.close();
  return 0;

}
