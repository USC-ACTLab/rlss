#include "ros/ros.h"
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
#include "create2_controller/TrajectoryState2D.h"
#include "create2_controller/Vector2D.h"
#include <tf/transform_listener.h>

#include "qp_optimize.h"

// different solvers
#include <qpOASES.hpp>
#include "osqp.h"

using namespace std;

namespace fs = std::experimental::filesystem::v1;
using namespace std;


typedef chrono::high_resolution_clock Time;
typedef chrono::milliseconds ms;
typedef chrono::duration<float> fsec;

double linearInterpolation(double start, double end, size_t idx, size_t count)
{
  return start + (end - start) * idx / (count - 1);
}

vector<vectoreuc> positions;

void getPositions(const tf::TransformListener& listener, int number_of_robots) {
  tf::StampedTransform transform;

  for(int i=1; i<=number_of_robots; i++) {
    listener.lookupTransform("/world", "/create" + std::to_string(i), ros::Time(0), transform);
    positions[i-1][0] = transform.getOrigin().x();
    positions[i-1][1] = transform.getOrigin().y();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;


  create2_controller::TrajectoryState2D desired_state_msg;
  ros::Publisher desired_state_publisher = nh.advertise<create2_controller::TrajectoryState2D>("desired_state", 1000);

  tf::TransformListener transformlistener;

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

  string initial_trajectory_path = jsn["trajectory"];
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
  const double scaling_multiplier = jsn["scaling_multiplier"];
  const int number_of_robots = jsn["number_of_robots"];
  const int robot_id = jsn["robot_id"];


  bool enable_voronoi = jsn["enable_voronoi"];

  string alg = jsn["algorithm"];



  srand(time(NULL));

  const double stoppingDistance = v_max * v_max / (2 * a_max);
  std::cout << "stoppingDistance: " << stoppingDistance << std::endl;


  ros::Rate rate((int)(1.0/dt + 0.5));

  trajectory original_trajectory;


  csv::Parser file(initial_trajectory_path);
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
    original_trajectory.add_curve(crv);
  }


  trajectory traj;

  for(int j=0; j<curve_count; j++) {
    traj.add_curve(original_trajectory[min(j, original_trajectory.size() -1)]);
    traj[j].duration = time_per_curve;
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
      o.add_pt(pt);
    }
    o.convex_hull();
    o.ch_planes(/*shift*/ 0);
    obstacles.push_back(o);
    obs_idx++;
  }

  OG og(cell_size, -10, 10, -10, 10, obstacles);

  vector<obstacle2D> cell_based_obstacles;
  for (size_t i = 0; i < og.max_i(); ++i) {
    for (size_t j = 0; j < og.max_j(); ++j) {
      OG::index idx(i, j);
      if (og.idx_occupied(idx)) {
        pair<double, double> coord = og.get_coordinates(idx);
        // std::cout << "occ: " << coord.first << "," << coord.second << std::endl;

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
  SvmSeperator svm(&cell_based_obstacles);


  double total_t = 0;
  for(int j=0; j<original_trajectory.size(); j++) {
    total_t += original_trajectory[j].duration;
  }


  positions.resize(number_of_robots);
  positions[robot_id] = traj.eval(0);
  vectoreuc velocitydes = traj.neval(0,1);
  vectoreuc accelerationdes = traj.neval(0,2);



  double total_time_for_opt = 0;
  int total_count_for_opt = 0;




  ObjectiveBuilder::Init();


  std::chrono::time_point<std::chrono::high_resolution_clock> t_start, t_start_a_star, t_end_a_star, t_start_svm, t_end_svm, t_start_qp, t_end_qp;

  double everyone_reached = false;


  for(double ct = 0; ct <= total_t && ros::ok() ; ct+=dt) {
    ros::spinOnce();

    getPositions(transformlistener, number_of_robots);

    cerr << ct << " / " << total_t << endl;


    t_start = Time::now();

    // if (ct > 7 && i == 0) {
    //   continue;
    // }

    /*calculate voronoi hyperplanes for robot i*/
    vector<hyperplane> voronoi_hyperplanes;
    if(enable_voronoi) {
      voronoi_hyperplanes = voronoi(positions,robot_id, robot_radius);
    }


    unsigned varcount = curve_count * ppc * problem_dimension;

    double planning_horizon = std::max(hor, curve_count * dt * 1.5);
    std::vector<double> pieceDurations(curve_count, planning_horizon / curve_count);

    // y are our control points (decision variable)
    // Vector y(numVars);
    Matrix y(problem_dimension, 8 * curve_count);

    // initialize y with previous solution
    for(int j=0; j<traj.size(); j++) {
      for(int k=0; k<ppc; k++) {
        for(int p=0; p<problem_dimension; p++) {
          y(p, j * ppc + k) = traj[j][k][p];
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
    bool planned_traj_occupied = og.occupied(traj, robot_radius);
    vectoreuc goalPos = original_trajectory.neval(min(ct+hor, total_t), 0);

    // only considers static obstacles, not other robots (intentionally)
    // bool goal_occupied = og.occupied(goalPos[0], goalPos[1], robot_radius);

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

    // check if original trajectory is occupied
    bool original_traj_occupied = og.occupied(original_trajectory, robot_radius, ct, min(ct+hor, total_t));

    bool discretePath = false;
    if (planned_traj_occupied /*|| goal_occupied*/ || voronoi_violated || original_traj_occupied) {
      // the trajectory is now on top of an obstacle => discrete re-planning!
      bool discretePlanningNeeded = true;

      std::vector< std::pair<double, double> > otherRobots;
      for(int j=0; j<number_of_robots; j++) {
        if(j != robot_id) {
          otherRobots.emplace_back(std::make_pair(positions[j][0], positions[j][1]));
        }
      }

      for (const auto& otherRobot : otherRobots) {

        if (   fabs(positions[robot_id][0] - otherRobot.first) <= 2 * robot_radius
            && fabs(positions[robot_id][1] - otherRobot.second) <= 2 * robot_radius) {
          std::cerr << "Other robot too close! Skip discrete planning" << std::endl;
          discretePlanningNeeded = false;
        }
      }

      // the goal is the next point on the original trajectory that is not occupied
      discreteSearch::State goal(-1, -1, OG::direction::NONE);
      double arrivalTime = min(ct+hor, total_t);
      vectoreuc goalPos;
      double discrete_horizon = 0;
      for (double t = arrivalTime; t <= total_t; t += 0.01) {
        goalPos = original_trajectory.neval(t, 0);
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
            if (   fabs(positions[robot_id][0] - otherRobot.first) <= 2 * robot_radius
                && fabs(positions[robot_id][1] - otherRobot.second) <= 2 * robot_radius) {
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
        discretePlanningNeeded = false;
      }

      if (discretePlanningNeeded) {
        OG::index startIdx = og.get_index(positions[robot_id][0], positions[robot_id][1]);
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
          corners.emplace_back(std::make_pair(positions[robot_id][0], positions[robot_id][1]));

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


          double total_discrete_path_length = 0.0;
          for (size_t j = 0; j < corners.size() - 1; ++j) {
            total_discrete_path_length += sqrt(pow(corners[j].first - corners[j+1].first, 2) +
                                               pow(corners[j].second - corners[j+1].second, 2));
          }

          // update cellBasedObstacles with robot positions, such that svmSeparator take other robots into account
          for (int j = 0; j < number_of_robots; ++j) {
            if (robot_id != j) {
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
          size_t hpidx = 0;
          for (size_t j = 0; j < discrete_curve_count && j < curve_count; ++j) {
            svm.reset_pts();
            vectoreuc pt(2);
            pt[0] = corners[j].first - robot_radius;
            pt[1] = corners[j].second - robot_radius;
            svm.add_pt(pt);
            pt[0] = corners[j].first - robot_radius;
            pt[1] = corners[j].second + robot_radius;
            svm.add_pt(pt);
            pt[0] = corners[j].first + robot_radius;
            pt[1] = corners[j].second + robot_radius;
            svm.add_pt(pt);
            pt[0] = corners[j].first + robot_radius;
            pt[1] = corners[j].second - robot_radius;
            svm.add_pt(pt);

            pt[0] = corners[j+1].first - robot_radius;
            pt[1] = corners[j+1].second - robot_radius;
            svm.add_pt(pt);
            pt[0] = corners[j+1].first - robot_radius;
            pt[1] = corners[j+1].second + robot_radius;
            svm.add_pt(pt);
            pt[0] = corners[j+1].first + robot_radius;
            pt[1] = corners[j+1].second + robot_radius;
            svm.add_pt(pt);
            pt[0] = corners[j+1].first + robot_radius;
            pt[1] = corners[j+1].second - robot_radius;
            svm.add_pt(pt);

            // gives us one hyperplane per obstacle
            vector<hyperplane> hyperplanes = svm._8_4_seperate();


            do {
              // add hyperplane constraints
              // for (auto& plane : hyperplanes) {
              for (size_t hidx = 0; hidx < hyperplanes.size(); ++hidx) {
                auto& plane = hyperplanes[hidx];

                Vector normal(problem_dimension);
                normal << plane.normal[0], plane.normal[1];

                hyperplaneConstraints.push_back({j, normal, plane.distance - robot_radius});
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
              pieceDurations[j] = std::max(curve_length / total_discrete_path_length * discrete_horizon, 1.1 * dt);
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
              pieceDurations[j] = std::max(curve_length / total_discrete_path_length * discrete_horizon, 1.1 * dt);
            }
          }

          // force end of trajectory to be at discrete end point
          // TODO: factor?
          Vector endPosition(2);
          endPosition << y(0, 8 * curve_count - 1) , y(1, 8 * curve_count - 1);

          endCloseToObjectives.push_back({(size_t)curve_count - 1, 100, endPosition});
          // ob.endCloseTo(curve_count - 1, 100, endPosition);
          // cb.addConstraintEnd(curve_count - 1, 0, endPosition);

          for (int j = 0; j < number_of_robots; ++j) {
            if (robot_id != j) {
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
      for (size_t j = 0; j < curve_count ; ++j) {
        svm.reset_pts();

        vectoreuc pt(2);
        for(int k=0; k<ppc; k++) {
          pt[0] = traj[j][k][0] - robot_radius;
          pt[1] = traj[j][k][1] - robot_radius;
          svm.add_pt(pt);
          pt[0] = traj[j][k][0] - robot_radius;
          pt[1] = traj[j][k][1] + robot_radius;
          svm.add_pt(pt);
          pt[0] = traj[j][k][0] + robot_radius;
          pt[1] = traj[j][k][1] - robot_radius;
          svm.add_pt(pt);
          pt[0] = traj[j][k][0] + robot_radius;
          pt[1] = traj[j][k][1] + robot_radius;
          svm.add_pt(pt);
        }

        vector<hyperplane> hyperplanes = svm._32_4_seperate();
        std::cout << "hyperplanes2: " << hyperplanes.size() << std::endl;

        for (auto& plane : hyperplanes) {
          Vector normal(problem_dimension);
          normal << plane.normal[0], plane.normal[1];
          hyperplaneConstraints.push_back({j, normal, plane.distance - robot_radius});
          // cb.addHyperplane(j, normal, plane.distance);

          ++hpidx;
        }
      }
      t_end_svm = Time::now();


      double factor = 0.5f / curve_count;
      for (size_t j = 1; j < curve_count; ++j) {
        vectoreuc OBJPOS = original_trajectory.neval(min(ct+planning_horizon * (j+1)/(double)curve_count, total_t), 0);
        Vector targetPosition(problem_dimension);
        targetPosition << OBJPOS[0], OBJPOS[1];
        // ob.endCloseTo(j, factor * (j+1), targetPosition);
        endCloseToObjectives.push_back({j, factor * (j+1), targetPosition});
      }

    }

    for (size_t j = 0; j < curve_count; ++j) {
      for(int k=0; k<ppc; k++) {
        std::vector<double> crds;
        crds.push_back(y(0, j * ppc + k));
        crds.push_back(y(1, j * ppc + k));
      }
    }

    t_start_qp = Time::now();

    do { // loop for temporal scaling

    // construct QP matrices
    ObjectiveBuilder ob(problem_dimension, pieceDurations);

    double remaining_time = std::max(total_t - ct, curve_count * dt * 1.1);
    double factor = 1.0f / remaining_time;
    ob.minDerivativeSquared(1 * factor, 0 * factor, 5e-3 * factor, 0 * factor);

    for (const auto& o : endCloseToObjectives) {
      ob.endCloseTo(o.piece, o.lambda, o.value);
    }

    // const double magic = 0.0001;
    // voronoi (for the first curve only)
    for(int j=0; j<voronoi_hyperplanes.size(); j++) {
      hyperplane& plane = voronoi_hyperplanes[j];

      Vector normal(problem_dimension);
      normal << plane.normal[0], plane.normal[1];

      ob.maxHyperplaneDist(0, lambda_hyperplanes, normal, plane.distance);
    }

    // hyperplane

    for (const auto& hpc : hyperplaneConstraints) {
      ob.maxHyperplaneDist(hpc.piece, lambda_hyperplanes, hpc.normal, hpc.dist);
    }


    const size_t numVars = problem_dimension * 8 * curve_count;

    // constraint matrix A
    ConstraintBuilder cb(problem_dimension, pieceDurations);

    // initial point constraints

      // position (with added noise)
    if(max_continuity >= 0) {
      Vector value(problem_dimension);
      value << positions[robot_id][0], positions[robot_id][1];
      cb.addConstraintBeginning(0, 0, value); // Position
    }

      // higher order constraints
    if(max_continuity >= 1) {
      Vector value(problem_dimension);
      value << velocitydes[0], velocitydes[1];
      cb.addConstraintBeginning(0, 1, value); // Velocity
    }

    if(max_continuity >= 2) {
      Vector value(problem_dimension);
      value << accelerationdes[0], accelerationdes[1];
      cb.addConstraintBeginning(0, 2, value); // Acceleration
    }


    // continuity constraints
    for (size_t i = 0; i < curve_count - 1; ++i) {
      for (size_t c = 0; c <= max_continuity; ++c) {
        cb.addContinuity(i, c);
      }
    }



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

    // lower and upper bound for decision variables (i.e., workspace)
    const double margin = 0.5;
    for(int j=0; j<traj.size(); j++) {
      Vector minVec(2);
      minVec << std::numeric_limits<double>::max(), std::numeric_limits<double>::max();
      Vector maxVec(2);
      maxVec << std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest();
      for(int p=0; p<problem_dimension; p++) {
        for(int k=0; k<ppc; k++) {
          minVec(p) = std::min(y(p, j * ppc + k), minVec(p));
          maxVec(p) = std::max(y(p, j * ppc + k), maxVec(p));
        }
        minVec(p) -= margin;
        maxVec(p) += margin;
      }
      cb.addBounds(j, minVec, maxVec);
    }

    const size_t numConstraints = cb.A().rows();

    std::cout << "piece durations: ";
    for (const auto& pd : pieceDurations) {
      std::cout << pd << ",";
    }
    std::cout << std::endl;

    if (alg == "OSQP") {
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
        sstr << "Couldn't solve QP! @ " << ct;
        // throw std::runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }

      // work->solution->x
      for (size_t i = 0; i < numVars; ++i) {
        y(i) = work->solution->x[i];
      }


      // Clean workspace
      osqp_cleanup(work);
    } else if (alg == "QPOASES") {
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
    qpOASES::real_t cputime = dt;
      qpOASES::returnValue status = qp.init(
        ob.H().data(),
        ob.g().data(),
        cb.A().data(),
        cb.lb().data(),
        cb.ub().data(),
        cb.lbA().data(),
        cb.ubA().data(),
        nWSR,
        set_max_time ? &cputime : NULL,
        y.data());

      qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(status);
      if (simpleStatus != 0) {

        //std::cerr << "MPC failed" << endl;
        qpOASES::QProblem qp2(numVars, numConstraints, qpOASES::HST_SEMIDEF);
        qpOASES::Options opts2;
        opts2.setToReliable();
        opts2.printLevel = qpOASES::PL_LOW;
        qp2.setOptions(opts2);
        status = qp2.init(
          ob.H().data(),
          ob.g().data(),
          cb.A().data(),
          cb.lb().data(),
          cb.ub().data(),
          cb.lbA().data(),
          cb.ubA().data(),
          nWSR,
          set_max_time ? &cputime : NULL,
          y.data());

        simpleStatus = qpOASES::getSimpleStatus(status);

        if(simpleStatus != 0) {
          std::stringstream sstr;
          sstr << "Couldn't solve QP! @ " << ct;
          //std::cerr << "reliable falied" << endl;
          // throw std::runtime_error(sstr.str());
          std::cerr << sstr.str() << std::endl;

          Eigen::Map<Matrix> yVec(y.data(), numVars, 1);

          auto constraints = cb.A() * yVec;
          // std::cout << "constraints: " << constraints << std::endl;
          for (size_t c = 0; c < numConstraints; ++c) {
            if (cb.lbA()(c) > constraints(c) || constraints(c) > cb.ubA()(c)) {
              std::cerr << "Constraint " << cb.info(c) << " violated!" << " value: " << constraints(c) << " lb: " << cb.lbA()(c) << " ub: " << cb.ubA()(c) << std::endl;
            }
          }
          std::cerr << "vel: " << velocitydes << "acc: " << accelerationdes << std::endl;
        }

        // y.setZero();

        // continue;
      }

      qp.getPrimalSolution(y.data());

      std::cout << "status: " << status << std::endl;
      std::cout << "objective: " << qp.getObjVal() << std::endl;
      // std::cout << "y: " << y << std::endl;
    } else {
      throw std::runtime_error("Unknown algorithms!");
    }

    t_end_qp = Time::now();
    // Update trajectories with our solution
    for(int j=0; j<traj.size(); j++) {
      for(int k=0; k<ppc; k++) {
        for(int p=0; p<problem_dimension; p++) {
          traj[j][k][p] = y(p, j * ppc + k);
        }
      }
    }

    // temporal scaling
    // scale until slow enough

    for (size_t j = 0; j < curve_count; ++j) {
      traj[j].duration = pieceDurations[j];
    }

    double max_velocity = -1;
    double max_acceleration = -1;
    for (double t = 0; t < traj.duration(); t+=dt/10.0) {
      double velocity = traj.neval(t, 1).L2norm();
      double acceleration = traj.neval(t, 2).L2norm();
      max_velocity = std::max(max_velocity, velocity);
      max_acceleration = std::max(max_acceleration, acceleration);
    }
    std::cout << "max_vel " << max_velocity << std::endl;
    std::cout << "max_acc " << max_acceleration << std::endl;
    if (   max_velocity > v_max
        || max_acceleration > a_max) {
      for (auto& pd : pieceDurations) {
        pd *= scaling_multiplier;
      }
      std::cerr << "SCALING " << max_velocity << " " << max_acceleration << std::endl;
    } else {
        break;
    }





    } while (true); // temporal scaling


    auto t1 = Time::now();
    fsec fs = t1 - t_start;
    ms d = chrono::duration_cast<ms>(fs);
    total_time_for_opt += d.count();
    total_count_for_opt++;
    cout << "optimization time: " << d.count() << "ms" << endl;


    vectoreuc despos = traj.neval(dt, 0);
    velocitydes = traj.neval(dt, 1);
    accelerationdes = traj.neval(dt, 2);

    desired_state_msg.position.x = despos[0];
    desired_state_msg.position.x = despos[1];
    desired_state_msg.velocity.x = velocitydes[0];
    desired_state_msg.velocity.x = velocitydes[1];
    desired_state_msg.acceleration.x = accelerationdes[0];
    desired_state_msg.acceleration.x = accelerationdes[1];

    desired_state_publisher.publish(desired_state_msg);


    rate.sleep();
  }



  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;

  return 0;

}
