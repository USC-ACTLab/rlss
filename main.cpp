#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <Eigen/StdVector>
#include "csv/CSVparser.hpp"
#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <utility>
#include <chrono>
#include "cxxopts.hpp"
#include "json.hpp"
#include <chrono>
#include <random>
#include "PointCloud.h"
#include "OccupancyGrid3D.h"
#include "utility.h"
#include "spline.h"
#include "bezier.h"
#include "SVM.h"
#include <utility>
#include "discrete_search.hpp"
#include <memory>
#include "qpOASES.hpp"
#include "outpututils.hpp"


namespace fs = std::experimental::filesystem;
using std::string;
using std::cout;
using std::vector;
using std::endl;
using std::ifstream;
using namespace ACT;
using namespace splx;
using std::max;
using std::min;
using std::pair;
using std::ofstream;
using nlohmann::json;





int main(int argc, char** argv) {
  string config_path;

  cxxopts::Options options("Trajectory Replanner", "trajectory replanner");
  options.add_options()
    ("cfg", "Config file", cxxopts::value<std::string>()->default_value("../config.json"))
    ("help", "Display help page");

  auto result = options.parse(argc, argv);

  if(result.count("help") > 0) {
    cout << options.help() << endl;
    return 0;
  }
  config_path = result["cfg"].as<string>();


  ifstream cfg(config_path);


  json jsn = nlohmann::json::parse(cfg);


  const bool enable_voronoi = jsn["enable_voronoi"];
  const bool set_max_time = jsn["set_max_time_as_replan_period"];
  const bool enable_svm = jsn["enable_svm"];
  const unsigned int max_continuity = jsn["continuity_upto_degree"];
  const unsigned int curve_count = jsn["plan_for_curves"];
  const unsigned int points_per_curve = jsn["points_per_curve"];
  const string outputfile = jsn["output_file"];
  const string initial_trajectories_path = jsn["trajectories"];
  const string obstacles_path = jsn["obstacles"];
  const double scale_traj = jsn["scale_traj"];
  const double dt = jsn["replan_period"];
  const double robot_radius = jsn["robot_radius"];
  const double cell_size = jsn["cell_size"];
  const double desired_horizon = jsn["planning_horizon"];
  const vector<double> max_derivative_magnitudes = jsn["max_derivative_magnitudes"];
  const vector<double> lambdas_integrated_squared_derivative
    = jsn["lambdas_integrated_squared_derivative"];
  const double scaling_multiplier = jsn["scaling_multiplier"];
  const double additional_time = jsn["additional_time"];
  const double output_frame_dt = jsn["output_frame_dt"];

  const double step = 0.01; // general purpose step



  using VectorDIM = Eigen::Matrix<double, 3U, 1U>;
  using Hyperplane = Eigen::Hyperplane<double, 3U>;
  using AlignedBox = PointCloud<double, 3U>::AlignedBox;
  using QPMatrices = Spline<double, 3U>::QPMatrices;


  /* Get obstacles and construct occupancy grid */
  std::vector<PointCloud<double, 3U> > OBSTACLES;
  for(auto& p: fs::directory_iterator(obstacles_path)) {
    PointCloud<double, 3U> obs;
    string path(p.path());
    ifstream file(path);
    VectorDIM vec;
    while(file >> vec(0)) {
      file >> vec(1) >> vec(2);
      obs.addPoint(vec);
    }
    obs.convexHull();
    OBSTACLES.push_back(obs);
  }
  OccupancyGrid3D<double> OCCUPANCY_GRID(OBSTACLES, cell_size, 0, 20, 0, 20, 0, 20);
  cout << "Occupied cell count: " << OCCUPANCY_GRID._occupied_boxes.size() << endl;

  /* Get original trajectories */
  std::vector<Spline<double, 3U> > ORIGINAL_TRAJECTORIES;
  for(auto& p: fs::directory_iterator(initial_trajectories_path)) {
    Spline<double, 3U> spline;
    string path(p.path());
    ifstream file(path);
    string line;
    while(file >> line) {
      line = trim(line);
      if(line[0] == '#' || line.size() == 0)
        continue;
      vector<string> numbers = tokenize(line, ',');
      assert(numbers.size() % 3 == 1);
      Bezier<double, 3U> bez(stod(numbers[0]) * scale_traj);
      VectorDIM vec;
      for(int i = 1; i < numbers.size(); i+=3) {
        vec(0) = stod(numbers[i]);
        vec(1) = stod(numbers[i+1]);
        vec(2) = stod(numbers[i+2]);
        bez.m_controlPoints.push_back(vec);
      }
      spline.addPiece(bez);
    }
    ORIGINAL_TRAJECTORIES.push_back(spline);
  }

  const unsigned int robot_count = ORIGINAL_TRAJECTORIES.size();

  cout << "Robot count: " << robot_count << endl;

  /*
  * Initialize CURRENT_STATES and TOTAL_TIMES
  *
  * CURRENT_STATES[x][y] gives the yth derivative of xth robot
  *
  * TOTAL_TIMES[x] gives the total time of xth robot
  *
 */

  std::vector<std::vector<VectorDIM, Eigen::aligned_allocator<VectorDIM> > >
    CURRENT_STATES(ORIGINAL_TRAJECTORIES.size());
  std::vector<double> TOTAL_TIMES;
  double MAX_TOTAL_TIME = -1;
  for(int i = 0; i < ORIGINAL_TRAJECTORIES.size(); i++) {
    auto& spl = ORIGINAL_TRAJECTORIES[i];
    for(int c = 0; c <= max_continuity; c++) {
      CURRENT_STATES[i].push_back(spl.eval(0, c));
    }
    TOTAL_TIMES.push_back(spl.totalSpan());
    MAX_TOTAL_TIME = max(MAX_TOTAL_TIME, TOTAL_TIMES[TOTAL_TIMES.size() - 1]);
  }


  /*
  * Initialize TRAJECTORIES to be planned
  *
  * fix this
  */
  std::vector<Spline<double, 3U> > TRAJECTORIES(robot_count);
  for(unsigned int r = 0; r < robot_count; r++) {
    auto& traj = TRAJECTORIES[r];
    auto& origtraj = ORIGINAL_TRAJECTORIES[r];
    for(unsigned int i = 0; i < curve_count; i++) {
      Bezier<double, 3U> bez(origtraj.getPiece(min(i, origtraj.numPieces() - 1)));
      traj.addPiece(bez);
    }
  }


  vector<bool> LAST_QP_FAILED(robot_count, false);


  json output_json;
  output_json["robot_count"] = robot_count;
  output_json["robot_radius"] = robot_radius;
  for(unsigned int r = 0; r < robot_count; r++) {
    json original_trajectory_json;
    original_trajectory_json["robot_id"] = r;
    const auto& origtraj = ORIGINAL_TRAJECTORIES[r];
    original_trajectory_json["trajectory"] = json_spline<double, 3U>(origtraj);
    output_json["original_trajectories"].push_back(original_trajectory_json);
  }
  output_json["frame_dt"] = output_frame_dt;

  cout << output_json << endl;
  return 0;

  double SIMULATION_DURATION = MAX_TOTAL_TIME + additional_time;

  for(double ct = 0; ct <= SIMULATION_DURATION; ct += dt) {
    cout << endl << "#######" << endl << "#### Current Time: " << ct
      << "/" << SIMULATION_DURATION << " ####" << endl << "#######" << endl;

#ifdef ACT_DEBUG
    cout << "[DEBUG] ### Current States" << endl;
    for(unsigned int r = 0 ; r< robot_count; r++) {
      cout << "[DEBUG] robot " << r;
      for(unsigned int c = 0; c <= max_continuity; c++) {
        cout << ", deg" << c << ": " << string_vector<double, 3U>(CURRENT_STATES[r][c]);
      }
      cout << endl;
    }
#endif

    for(unsigned int r = 0; r < robot_count; r++) {
      cout << endl << "#### Iteration for robot " << r  << " ####"<< endl;

      const auto& origtraj = ORIGINAL_TRAJECTORIES[r];
      auto& traj = TRAJECTORIES[r];

      const auto& state = CURRENT_STATES[r];

      /*
      * Add other robots as obstacles
      */
      vector<pair<OccupancyGrid3D<double>::Index, unsigned int> > OG_CHANGES;
      for(int i = 0; i < robot_count; i++) {
        if(i != r) {
          auto changes = OCCUPANCY_GRID.addRobot(CURRENT_STATES[i][0], robot_radius);
          for(const auto& c: changes) {
            OG_CHANGES.push_back(c);
          }
        }
      }

#ifdef ACT_DEBUG
      cout << "[DEBUG] Occupied cell count after other robot additions: " << OCCUPANCY_GRID._occupied_boxes.size() << endl;
#endif

      /*
      * Calculate voronoi for the current robot
      */
      vector<Hyperplane> VORONOI_HYPERPLANES;
      if(enable_voronoi) {
        VORONOI_HYPERPLANES = voronoi<double, 3U>(CURRENT_STATES, r, robot_radius);
      }

#ifdef ACT_DEBUG
      cout << "[DEBUG] Number of voronoi hyperplanes: " << VORONOI_HYPERPLANES.size() << endl;
      for(const auto& vhp: VORONOI_HYPERPLANES) {
        cout << "[DEBUG] voronoi hyperplane " << string_hyperplane<double, 3U>(vhp) << endl;
      }
#endif

      /*
      * Check if the first piece of the previous trajectory is outside of the
      * buffered voronoi cell
      */
      bool previous_trajectory_first_piece_outside_voronoi = false;
      for(const auto& hp: VORONOI_HYPERPLANES) {
        if(!traj.onNegativeSide(hp, 0)) {
          previous_trajectory_first_piece_outside_voronoi = true;
          break;
        }
      }

#ifdef ACT_DEBUG
      cout << "[DEBUG] previous_trajectory_first_piece_outside_voronoi: "
      << previous_trajectory_first_piece_outside_voronoi << endl;
#endif

      /*
      * Check if original trajectory is occupied from ct to ct + desired_horizon
      */
      bool original_trajectory_occupied = origtraj.
        intersects(OCCUPANCY_GRID._occupied_boxes,
          min(TOTAL_TIMES[r], ct), min(TOTAL_TIMES[r], ct + desired_horizon),
          robot_radius);

#ifdef ACT_DEBUG
      cout << "[DEBUG] original_trajectory_occupied: "
        << original_trajectory_occupied << endl;
#endif

      /*
      * Check if previous trajectory is occupied from 0 to tau
      */
      bool previous_trajectory_occupied = traj.
        intersects(OCCUPANCY_GRID._occupied_boxes,
          0, traj.totalSpan(), robot_radius);

#ifdef ACT_DEBUG
      cout << "[DEBUG] previous_trajectory_occupied: "
        << previous_trajectory_occupied << endl;
#endif

#ifdef ACT_DEBUG
      cout << "[DEBUG] qp failed in previous iteration: " << LAST_QP_FAILED[r] << endl;
#endif

      /*
      * true if discrete planning is needed
      */
      bool discrete_planning_needed =
        previous_trajectory_first_piece_outside_voronoi ||
        original_trajectory_occupied ||
        previous_trajectory_occupied ||
        LAST_QP_FAILED[r];

#ifdef ACT_DEBUG
      cout << "[DEBUG] discrete_planning_needed: " << discrete_planning_needed << endl;
#endif


      /*
      * even if discrete planning is needed we may not be able to do it
      * because there is no unoccupied location in original trajectory
      * from time ct + desired_horizon to TOTAL_TIMES[r]
      */
      bool discrete_planning_done = false;
      if(discrete_planning_needed) {
        // discrete planning needed

        /*
        * find the target time that we try to reach
        */
        double target_time;
        /*
        * target_time_valid:
        * true if original trajectory is not occupied at time target_time
        */
        bool target_time_valid = false;
        if(ct + desired_horizon > TOTAL_TIMES[r]) {
          // we are using extra time or remaining time is less than desired_horizon
          // try to go to the end point
          target_time = TOTAL_TIMES[r];
          // we can go to the end point if original trajectory is not occupied at that time.
          target_time_valid = !origtraj.intersects(OCCUPANCY_GRID._occupied_boxes,
            target_time, target_time, robot_radius);
        } else {
          // we have not less than desired_horizon time
          for(target_time = ct + desired_horizon;
            target_time <= TOTAL_TIMES[r]; target_time += step) {
            if(!origtraj.intersects(OCCUPANCY_GRID._occupied_boxes,
              target_time, target_time, robot_radius)) {
              target_time_valid = true;
              break;
            }
          }
        }

#ifdef ACT_DEBUG
        cout << "[DEBUG] target_time_valid: " << target_time_valid << ", target_time: " << target_time << endl;
#endif

        // if the original trajectory is unoccupied at target_time
        if(target_time_valid) {
          // try discrete planning
          // it may fail if there is no path
          // or it may succeed if there is a path
          // if it succeeds set discrete_planning_done to True
          // else set it to False
          const VectorDIM& currentPos = state[0];
          const VectorDIM targetPos = origtraj.eval(target_time, 0);

#ifdef ACT_DEBUG
          cout << "[DEBUG] Discrete planning attempt from currentPos: "
            << string_vector<double, 3U>(currentPos) <<" to targetPos: "
            << string_vector<double, 3U>(targetPos) << endl;
#endif

          auto start_idx = OCCUPANCY_GRID.getIndex(currentPos);
          auto target_idx = OCCUPANCY_GRID.getIndex(targetPos);

          discreteSearch::State startState(start_idx.i,
              start_idx.j, start_idx.k, discreteSearch::Direction::NONE);
          discreteSearch::State targetState(target_idx.i,
              target_idx.j, target_idx.k, discreteSearch::Direction::NONE);

          discreteSearch::Environment<double> env(OCCUPANCY_GRID, robot_radius, targetState);

          libSearch::AStar<discreteSearch::State,
            discreteSearch::Action, int, discreteSearch::Environment<double> > astar(env);
          libSearch::PlanResult<discreteSearch::State, discreteSearch::Action, int> solution;

          discrete_planning_done = astar.search(startState, solution);

#ifdef ACT_DEBUG
          cout << "[DEBUG] discrete planning successful: " << discrete_planning_done << endl;
#endif

          if(discrete_planning_done) {
            vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> corners;
            corners.push_back(currentPos);
            const auto& firstSolutionState = solution.states[0].first;
            corners.push_back(OCCUPANCY_GRID.getCoordinates(firstSolutionState.x,
              firstSolutionState.y, firstSolutionState.z));
            for(unsigned int i = 0; i < solution.actions.size(); i++) {
              const auto& action = solution.actions[i].first;
              if(action != discreteSearch::Action::Forward) {
                const auto& state = solution.states[i].first; // i == i-1, only directions change
                corners.push_back(OCCUPANCY_GRID.getCoordinates(state.x,
                  state.y, state.z));
              }
            }
            const auto& lastSolutionState = solution.states.back().first;
            corners.push_back(OCCUPANCY_GRID.getCoordinates(lastSolutionState.x,
              lastSolutionState.y, lastSolutionState.z));
            corners.push_back(targetPos);

#ifdef ACT_DEBUG
            cout << "[DEBUG] Corners: ";
            for(const auto& crnr: corners) {
              cout << " " << string_vector<double, 3U>(crnr);
            }
            cout << endl;
#endif
            if(curve_count > corners.size() - 1) {
#ifdef ACT_DEBUG
              cout << "[DEBUG] Splitting segments" << endl;
#endif
              // we need to split segments since there are not enough segments
              // for each curve
              corners = bestSplitSegments<double, 3U>(corners, curve_count);
#ifdef ACT_DEBUG
              cout << "[DEBUG] Corners after split:";
              for(const auto& crnr: corners) {
                cout << " " << string_vector<double, 3U>(crnr);
              }
              cout << endl;
#endif
              // this is just a sanity check
              assert(corners.size() == curve_count + 1);
            }

            // corners are ready here
            // load the trajectory with first curve_count segments
            // interpolate and stuff



            double total_discrete_path_length = 0;
            for(int i = 0; i < corners.size() - 1; i++) {
              total_discrete_path_length += (corners[i+1] - corners[i]).norm();
            }

            // target time may be less than the current time since we allow extra time
            double discrete_path_total_duration = max(target_time - ct, 0.0);

            /*
            * if discrete_path_total_duration is not enough for robot to go from start to end
            * even with the maximum allowed velocity
            * increase it to needed duration if it goes with maximum allowed velocity
            * notice that this ignores higher order dynamic limits
            * but it is something ¯\_(ツ)_/¯
            * may need to multiply with 1.2 or 1.5 or smth like that in the future
            * to solve the problems with higher order dynamic limits!
            */
            if(max_derivative_magnitudes.size() > 1 && max_derivative_magnitudes[1] >= 0) {
              discrete_path_total_duration = max(discrete_path_total_duration,
                  total_discrete_path_length / max_derivative_magnitudes[1]);
            }


            /*
            * interpolate the corners, and set interpolation results
            * as control points in the corresponding pieces
            *
            * also set curve durations
            */
            for(unsigned int i = 0; i < curve_count; i++) {
              auto points = linearInterpolate<double, 3U>(corners[i], corners[i+1], points_per_curve);
              auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(traj.getPiece(i));
              bezptr->m_controlPoints = points;
              bezptr->m_a = ((corners[i+1] - corners[i]).norm() / total_discrete_path_length)
                * discrete_path_total_duration;
              // duration of first piece must be at least dt
              //if(i == 0) {
                bezptr->m_a = max(bezptr->m_a, 1.1 * dt);
              //}
            }
#ifdef ACT_DEBUG
            cout << "[DEBUG] [with discrete planning] piece durations: ";
            for(unsigned int i = 0; i < traj.numPieces(); i++) {
              auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(traj.getPiece(i));
              cout << bezptr->m_a << " ";
            }
            cout << endl;
#endif
          }
        }
      }


      // at this point, initial points in traj are set
      // and if discrete search is done, durations are set as well
      // yeah. cool.


      /*
      * initialize the durations if discrete planning is not done.
      * if it is done, durations are already initialized before
      */
      if(!discrete_planning_done) {
        // current policy: dont change them?
        double total_discrete_length = 0; // length of the point to point distance along the spline
        for(unsigned int i = 0; i < traj.numPieces(); i++) {
          auto piece = std::static_pointer_cast<Bezier<double, 3U>>(traj.getPiece(i));
          for(unsigned int j = 0; j < piece->m_controlPoints.size() - 1; j++) {
            const VectorDIM& cp = piece->m_controlPoints[j];
            const VectorDIM& nextcp = piece->m_controlPoints[j + 1];
            total_discrete_length += (nextcp - cp).norm();
          }
        }

        double total_duration = min(desired_horizon, TOTAL_TIMES[r] - ct);
        /*
        * total_duration may be negative since we allow extra time
        */
        total_duration = max(total_duration, 0.0);

        /*
        * If total duration is less than the duration needed when robot moves with maximum speed
        * set it to that value.
        *
        * If we can use arc length instead of discrete length, it can be better.
        *
        * This is overly conservative
        */
        if(max_derivative_magnitudes.size() > 1 && max_derivative_magnitudes[1] >= 0) {
          total_duration = max(total_duration,
            total_discrete_length / max_derivative_magnitudes[1]);
        }

        for(unsigned int i = 0; i < traj.numPieces(); i++) {
          auto piece = std::static_pointer_cast<Bezier<double, 3U>>(traj.getPiece(i));
          double piece_discrete_length = 0;
          for(unsigned int j = 0; j < piece->m_controlPoints.size() - 1; j++) {
            const VectorDIM& cp = piece->m_controlPoints[j];
            const VectorDIM& nextcp = piece->m_controlPoints[j + 1];
            piece_discrete_length += (nextcp - cp).norm();
          }

          piece->m_a = (piece_discrete_length
            / total_discrete_length) * total_duration;

          /*
          * If this is the first piece, make sure that it has at least dt duration
          * for voronoi!
          */
          //if(i == 0) {
            piece->m_a = max(piece->m_a, 1.1 * dt);
          //}
        }
#ifdef ACT_DEBUG
        cout << "[DEBUG] [without discrete planning] piece durations: ";
        for(unsigned int i = 0; i < traj.numPieces(); i++) {
          auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(traj.getPiece(i));
          cout << bezptr->m_a << " ";
        }
        cout << endl;
#endif
      }


      // initial guesses are set here

      // if dynamic limits are NOT violated after an optimization iteration,
      // this is set to true
      bool dynamic_limits_OK = false;

      while(!dynamic_limits_OK) {
        // Lets start to construct the QP
        vector<QPMatrices> qp = traj.getQPMatrices();


        // add initial point constraints
        for(unsigned int k = 0; k <= max_continuity; k++) {
          traj.extendQPBeginningConstraint(qp, k, CURRENT_STATES[r][k]);
        }

        // add voronoi constraints
        for(const auto& hp: VORONOI_HYPERPLANES) {
          traj.extendQPHyperplaneConstraint(qp, 0, hp);
        }


        // add svm constraints
        VectorDIM rad(robot_radius, robot_radius, robot_radius);
        for(unsigned int piece_idx = 0; piece_idx < traj.numPieces(); piece_idx++) {
          auto piece = std::static_pointer_cast<Bezier<double, 3U>>(
              traj.getPiece(piece_idx));
          vector<AlignedBox> robot_boxes;
          if(discrete_planning_done) {
            const VectorDIM& first_pt = piece->m_controlPoints[0];
            const VectorDIM& last_pt = piece->m_controlPoints.back();
            robot_boxes.push_back(AlignedBox(first_pt - rad, first_pt + rad));
            robot_boxes.push_back(AlignedBox(last_pt - rad, last_pt + rad));
          } else {
            for(const auto& pt: piece->m_controlPoints) {
              robot_boxes.push_back(AlignedBox(pt - rad, pt + rad));
            }
          }

          vector<Hyperplane> hyperplanes;

          if(enable_svm)
            hyperplanes = svm3d(robot_boxes, OCCUPANCY_GRID._occupied_boxes);

          for(const auto& hp: hyperplanes) {
            traj.extendQPHyperplaneConstraint(qp, piece_idx, hp);
          }
        }

        // add integrated squared derivative cost
        for(unsigned int i = 0;
            i < lambdas_integrated_squared_derivative.size(); i++) {
          double lambda = lambdas_integrated_squared_derivative[i];
          if(lambda > 0.0) {
            traj.extendQPIntegratedSquaredDerivative(qp, i, lambda);
          }
        }

        // add endpoint costs
        if(discrete_planning_done) {
          auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(
            traj.getPiece(traj.numPieces() - 1));
          const VectorDIM& endpt = bezptr->m_controlPoints.back();
          traj.extendQPPositionAt(qp, traj.totalSpan(), endpt, 100);
        } else {
          double time_forward = 0;
          double theta_factor = 0.5 / curve_count;
          for(unsigned int i = 0; i < traj.numPieces(); i++) {
            auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(traj.getPiece(i));
            time_forward += bezptr->m_a;
            const VectorDIM endpt = origtraj.eval(min(ct + time_forward, TOTAL_TIMES[r]), 0);
            traj.extendQPPositionAt(qp, time_forward, endpt, theta_factor * (i+1));
          }
        }

        // merge matrices
        QPMatrices combinedQP = traj.combineQPMatrices(qp);

        // add continuity constraints
        for(unsigned int i = 0; i <= max_continuity; i++) {
          for(unsigned int j = 0; j < traj.numPieces() - 1; j++) {
            traj.extendQPContinuityConstraint(combinedQP, qp, j, i);
          }
        }

        // solve
        qpOASES::QProblem problem(combinedQP.x.rows(), combinedQP.A.rows());
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_LOW;
        problem.setOptions(options);

        qpOASES::int_t nWSR = 10000;
        qpOASES::real_t cputime = dt;

        qpOASES::returnValue ret =
          problem.init(
            combinedQP.H.data(),
            combinedQP.g.data(),
            combinedQP.A.data(),
            combinedQP.lbX.data(),
            combinedQP.ubX.data(),
            combinedQP.lbA.data(),
            combinedQP.ubA.data(),
            nWSR,
            set_max_time ? &cputime: NULL,
            combinedQP.x.data()
          );
        qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(ret);
        if(simpleStatus != 0) {
          LAST_QP_FAILED[r] = true;
#ifdef ACT_DEBUG
          cout << "[DEBUG] QP Failed" << endl;
#endif
        } else {
          LAST_QP_FAILED[r] = false;
        }


        // load
        problem.getPrimalSolution(combinedQP.x.data());
        traj.loadControlPoints(combinedQP, qp);



        // check dynamic limits
        dynamic_limits_OK = true;
        for(unsigned int i = 0; i < max_derivative_magnitudes.size(); i++) {
          double max_allowed_mag = max_derivative_magnitudes[i];
          if(max_allowed_mag >= 0.0) {
            double max_mag = traj.maxDerivativeMagnitude(i, 0.01);
            if(max_mag > max_allowed_mag) {
              dynamic_limits_OK = false;
              break;
            }
          }
        }
#ifdef ACT_DEBUG
        cout << "[DEBUG] dynamic_limits_OK: " << dynamic_limits_OK << endl;
#endif
        if(!dynamic_limits_OK) {
          for(unsigned i = 0; i < traj.numPieces(); i++) {
            auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(
              traj.getPiece(i));

            bezptr->m_a *= scaling_multiplier;
          }
        }

      }

      /*
      * Remove other robots from occupancy grid
      */
      OCCUPANCY_GRID.undoRobotChanges(OG_CHANGES);
#ifdef ACT_DEBUG
      cout << "[DEBUG] Occupied cell count after other robot removals: " << OCCUPANCY_GRID._occupied_boxes.size() << endl;
#endif
    }

    for(int r = 0; r < robot_count; r++) {
      const auto& traj = TRAJECTORIES[r];
      for(int c = 0; c <= max_continuity; c++) {
        CURRENT_STATES[r][c] = traj.eval(dt, c);
      }
    }
  }



  return 0;

}
