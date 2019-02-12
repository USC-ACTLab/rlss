#include <iostream>
#include <fstream>
#include <experimental/filesystem>
#include <Eigen/StdVector>
#include "csv/CSVparser.hpp"

#include "json.hpp"

#include "PointCloud.h"
#include "OccupancyGrid3D.h"
#include "utility.h"
#include "spline.h"
#include "bezier.h"
#include "SVM.h"
#include "discrete_search.hpp"
#include "discrete_search.hpp"
#include "outpututils.hpp"
#include "qpOASES.hpp"

#include "ros/ros.h"
#include "crazyflie_driver/FullState.h"
#include <tf/transform_listener.h>

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

#if 0
using boost::asio::ip::udp;
using boost::asio::ip::address;

typedef chrono::high_resolution_clock Time;
typedef chrono::milliseconds ms;
typedef chrono::duration<float> fsec;

double linearInterpolation(double start, double end, size_t idx, size_t count)
{
  return start + (end - start) * idx / (count - 1);
}

vector<vectoreuc> positions;

void getPositions(const tf::TransformListener& listener, const std::vector<std::string>& robots) {
  tf::StampedTransform transform;

  for(int i=0; i<robots.size(); i++) {
    listener.lookupTransform("/world", robots[i], ros::Time(0), transform);
    positions[i][0] = transform.getOrigin().x();
    positions[i][1] = transform.getOrigin().y();
  }
}
#endif
int main(int argc, char **argv) {

#ifdef ACT_DEBUG
  cout << "ACT_DEBUG" << endl;
#else
  cout << "NO_ACT_DEBUG" << endl;
#endif

#ifdef ACT_GENERATE_OUTPUT_JSON
  cout << "ACT_GENERATE_OUTPUT_JSON" << endl;
#else
  cout << "NO_ACT_GENERATE_OUTPUT_JSON" << endl;
#endif

  ros::init(argc, argv, "planner3d");
  ros::NodeHandle nh;

  crazyflie_driver::FullState desired_state_msg;
  ros::Publisher desired_state_publisher = nh.advertise<crazyflie_driver::FullState>("desired_state", 1000);

  tf::TransformListener transformlistener;

  // Read parameters
  ros::NodeHandle nl("~");
  bool enable_voronoi;
  nl.getParam("enable_voronoi", enable_voronoi);
  bool set_max_time;
  nl.getParam("set_max_time_as_replan_period", set_max_time);
  bool enable_svm;
  nl.getParam("enable_svm", enable_svm);
  int max_continuity;
  nl.getParam("continuity_upto_degree", max_continuity);
  int curve_count;
  nl.getParam("plan_for_curves", curve_count);
  int points_per_curve;
  nl.getParam("points_per_curve", points_per_curve);
  string outputfile;
  nl.getParam("simulation_output_file", outputfile);
  string statoutputfile;
  nl.getParam("statistics_output_file", statoutputfile);
  string initial_trajectories_path;
  nl.getParam("trajectories", initial_trajectories_path);
  string obstacles_path;
  nl.getParam("obstacles", obstacles_path);
  double scale_traj;
  nl.getParam("scale_traj", scale_traj);
  double dt;
  nl.getParam("replan_period", dt);
  double robot_radius;
  nl.getParam("robot_radius", robot_radius);
  double cell_size;
  nl.getParam("cell_size", cell_size);
  double desired_horizon;
  nl.getParam("planning_horizon", desired_horizon);
  vector<double> max_derivative_magnitudes;
  // TODO: read vector
  vector<double> lambdas_integrated_squared_derivative;
  // TODO: read vector;
  double scaling_multiplier;
  nl.getParam("scaling_multiplier", scaling_multiplier);
  double additional_time;
  nl.getParam("additional_time", additional_time);
  double output_frame_dt;
  nl.getParam("output_frame_dt", output_frame_dt);

  const double step = 0.01; // general purpose step


#ifdef ACT_STATISTICS
  json stats;
#endif


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
  OccupancyGrid3D<double> OCCUPANCY_GRID(OBSTACLES, cell_size, -5, 25, -5, 25, -5, 25);
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


  vector<bool> LAST_QP_FAILED(robot_count, true);

  // how many of the last successive iterations the QP has failed
  vector<unsigned int> SUCCESSIVE_FAILURE_COUNTS(robot_count, 0);

#ifdef ACT_GENERATE_OUTPUT_JSON
  json output_json;
  unsigned int frame_step = 0; // step that corresponds to current time (ct / output_frame_dt)
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
#endif

  double SIMULATION_DURATION = MAX_TOTAL_TIME + additional_time;

  ros::Rate rate((int)(1.0/dt + 0.5));
  ros::Time start = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    ros::Duration duration = now - start;
    double ct = duration.toSec();
    if (ct > SIMULATION_DURATION) {
      break;
    }

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

#ifdef ACT_GENERATE_OUTPUT_JSON
    json frame_json; // json for the current frame
    frame_json["step"] = frame_step;
    if(frame_step == 0) {
      // if this is the first frame, add obstacles and occupied cells
      if(!OBSTACLES.empty())
        frame_json["obstacles"] = json_obstacles(OBSTACLES);
      if(!OCCUPANCY_GRID._occupied_boxes.empty())
        frame_json["occupied_cells"] = json_occupied_cells<double, 3U>(OCCUPANCY_GRID._occupied_boxes);
    }
#endif

    // TODO: get robot idx from file...
    const unsigned int r = 0;
    {
      cout << endl << "#### Iteration for robot " << r  << " ####"<< endl;

      const auto& origtraj = ORIGINAL_TRAJECTORIES[r];
      auto& traj = TRAJECTORIES[r];
      Spline<double, 3U> trajback(traj);

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

#ifdef ACT_GENERATE_OUTPUT_JSON
      frame_json["voronoi_hyperplanes"].push_back(
        json_voronoi_hyperplanes_of_robot<double, 3U>(VORONOI_HYPERPLANES, r));
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


#ifdef ACT_GENERATE_OUTPUT_JSON
      json trajectory_update_json; // an element of 'trajectories' field in output json
      trajectory_update_json["robot_id"] = r;
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

          bool targetStateValid; // constructor of environment sets targetStateValid
          discreteSearch::Environment<double> env(OCCUPANCY_GRID, robot_radius, targetState, targetStateValid);

          libSearch::AStar<discreteSearch::State,
            discreteSearch::Action, int, discreteSearch::Environment<double> > astar(env);
          libSearch::PlanResult<discreteSearch::State, discreteSearch::Action, int> solution;



          discrete_planning_done = targetStateValid && astar.search(startState, solution);

#ifdef ACT_DEBUG
          cout << "[DEBUG] discrete planning successful: " << discrete_planning_done << endl;
#endif

          if(discrete_planning_done) {
            vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> corners;
            corners.push_back(currentPos);
            const auto& firstSolutionState = solution.states[0].first;
            //corners.push_back(OCCUPANCY_GRID.getCoordinates(firstSolutionState.x,
            //  firstSolutionState.y, firstSolutionState.z));
            for(unsigned int i = 0; i < solution.actions.size(); i++) {
              const auto& action = solution.actions[i].first;
              if(action != discreteSearch::Action::Forward) {
                const auto& state = solution.states[i].first; // i == i-1, only directions change
                corners.push_back(OCCUPANCY_GRID.getCoordinates(state.x,
                  state.y, state.z));
              }
            }
            //const auto& lastSolutionState = solution.states.back().first;
            //corners.push_back(OCCUPANCY_GRID.getCoordinates(lastSolutionState.x,
            //  lastSolutionState.y, lastSolutionState.z));
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

#ifdef ACT_GENERATE_OUTPUT_JSON
            trajectory_update_json["discrete_path"] = json_vec_vectordim<double, 3U>(corners);
#endif


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

      // give it 5 tries to solve the problem
      unsigned int allowed_tries = 10;
      unsigned int tries_remaining = allowed_tries;

      Spline<double, 3U> planning_ready_traj(traj);

      while((!dynamic_limits_OK || LAST_QP_FAILED[r]) && tries_remaining > 0) {

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

#ifdef ACT_GENERATE_OUTPUT_JSON
        json svm_hyperplanes_json;
        svm_hyperplanes_json["robot_id"] = r;
        svm_hyperplanes_json["hyperplanes"] = json::array();
#endif

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
#ifdef ACT_DEBUG
//          cout << "[DEBUG] SVM Hyperplane count for piece " << piece_idx << ": " << hyperplanes.size() << endl;
//          cout << "[DEBUG] SVM Hyperplanes: ";
//          for(const auto& hp: hyperplanes) {
//            cout << string_hyperplane<double, 3U>(hp) << " ";
//          }
//          cout << endl;
#endif
#ifdef ACT_GENERATE_OUTPUT_JSON
          json_svm_hyperplanes_of_piece_insert<double, 3U>(svm_hyperplanes_json["hyperplanes"], hyperplanes, piece_idx);
#endif
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
        if(!LAST_QP_FAILED[r]) {
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
        }

        if(LAST_QP_FAILED[r] || !dynamic_limits_OK) {
          traj = planning_ready_traj;
          for(unsigned i = 0; i < traj.numPieces(); i++) {
            auto bezptr = std::static_pointer_cast<Bezier<double, 3U>>(
              traj.getPiece(i));

            bezptr->m_a *= std::pow(scaling_multiplier, allowed_tries - tries_remaining + 1);
          }
        } else {
#ifdef ACT_GENERATE_OUTPUT_JSON
          frame_json["svm_hyperplanes"].push_back(svm_hyperplanes_json);
#endif
        }

        tries_remaining--;
      }

      if(LAST_QP_FAILED[r] || !dynamic_limits_OK) {
        SUCCESSIVE_FAILURE_COUNTS[r]++;
        traj = trajback;
      } else
        SUCCESSIVE_FAILURE_COUNTS[r] = 0;

#ifdef ACT_GENERATE_OUTPUT_JSON
      if(!LAST_QP_FAILED[r] && dynamic_limits_OK) {
        trajectory_update_json["trajectory"] = json_spline(traj);
        frame_json["trajectories"].push_back(trajectory_update_json);
      }
#endif

      /*
      * Remove other robots from occupancy grid
      */
      OCCUPANCY_GRID.undoRobotChanges(OG_CHANGES);
#ifdef ACT_DEBUG
      cout << "[DEBUG] Occupied cell count after other robot removals: " << OCCUPANCY_GRID._occupied_boxes.size() << endl;
#endif

      /*
      * Send new setpoint to CF
      */

      // const auto& traj = TRAJECTORIES[r];

      auto pos = traj.eval(dt * (SUCCESSIVE_FAILURE_COUNTS[r] + 1), 0);
      auto vel = traj.eval(dt * (SUCCESSIVE_FAILURE_COUNTS[r] + 1), 1);
      auto acc = traj.eval(dt * (SUCCESSIVE_FAILURE_COUNTS[r] + 1), 2);


      desired_state_msg.header.seq += 1;
      desired_state_msg.header.stamp = ros::Time::now();
      desired_state_msg.header.frame_id = "world";

      desired_state_msg.pose.position.x = pos(0);
      desired_state_msg.pose.position.y = pos(1);
      desired_state_msg.pose.position.z = pos(2);
      desired_state_msg.twist.linear.x = vel(0);
      desired_state_msg.twist.linear.y = vel(1);
      desired_state_msg.twist.linear.z = vel(2);
      desired_state_msg.acc.x = acc(0);
      desired_state_msg.acc.y = acc(1);
      desired_state_msg.acc.z = acc(2);
      // fixed yaw=0
      desired_state_msg.pose.orientation.x = 0;
      desired_state_msg.pose.orientation.y = 0;
      desired_state_msg.pose.orientation.z = 0;
      desired_state_msg.pose.orientation.w = 1;

      // compute omega
      const double yaw = 0.0;
      const double dyaw = 0.0;
      auto jerk = traj.eval(dt * (SUCCESSIVE_FAILURE_COUNTS[r] + 1), 3);
      auto thrust = acc + VectorDIM(0.0, 0.0, 9.81); // add gravity
      auto z_body = thrust.normalized();
      auto x_world = VectorDIM(cos(yaw), sin(yaw), 0);
      auto y_body = z_body.cross(x_world).normalized();
      auto x_body = y_body.cross(z_body);
      auto jerk_orth_zbody = jerk - (jerk.dot(z_body) * z_body);
      auto h_w = jerk_orth_zbody / thrust.norm();

      desired_state_msg.twist.angular.x = -h_w.dot(y_body);
      desired_state_msg.twist.angular.y = h_w.dot(x_body);
      desired_state_msg.twist.angular.z = z_body(2) * dyaw;

      desired_state_publisher.publish(desired_state_msg);
    }


    for(int r = 0; r < robot_count; r++) {
      const auto& traj = TRAJECTORIES[r];
      for(int c = 0; c <= max_continuity; c++) {
        CURRENT_STATES[r][c] = traj.eval(dt * (SUCCESSIVE_FAILURE_COUNTS[r] + 1), c);
      }
    }

#ifdef ACT_GENERATE_OUTPUT_JSON
    for(double t = 0; t < dt - output_frame_dt / 2; t += output_frame_dt) {
      for(unsigned int r = 0; r < robot_count; r++) {
        frame_json["robot_positions"].push_back(json_robot_position<double, 3U>(TRAJECTORIES[r], r, t + SUCCESSIVE_FAILURE_COUNTS[r] * dt));
      }
      output_json["frames"].push_back(frame_json);
      frame_json = json();
      frame_step++;
      frame_json["step"] =frame_step;
    }
#endif

#ifdef ACT_GENERATE_OUTPUT_JSON
    ofstream json_outp_file(outputfile);
    json_outp_file << output_json;
    json_outp_file.close();
#endif

    rate.sleep();
  }

#ifdef ACT_GENERATE_OUTPUT_JSON
  ofstream json_outp_file(outputfile);
  json_outp_file << output_json;
  json_outp_file.close();
#endif


#if 0
  create2_controller::TrajectoryState2D desired_state_msg;
  ros::Publisher desired_state_publisher = nh.advertise<create2_controller::TrajectoryState2D>("desired_state", 1000);

  tf::TransformListener transformlistener;

  ros::NodeHandle nl("~");

  string initial_trajectory_path;
  nl.getParam("trajectory", initial_trajectory_path);
  cout << initial_trajectory_path << endl;

  string obstacles_path;
  nl.getParam("obstacles", obstacles_path);

  double dt;
  nl.getParam("replan_period", dt);

  int problem_dimension;
  nl.getParam("problem_dimension", problem_dimension);

  int ppc;
  nl.getParam("points_per_curve", ppc);
  int max_continuity;
  nl.getParam("continuity_upto_degree", max_continuity);
  bool set_max_time;
  nl.getParam("set_max_time_as_replan_period", set_max_time);
  double hor;
  nl.getParam("planning_horizon", hor);
  int curve_count;
  nl.getParam("plan_for_curves", curve_count);
  double time_per_curve = hor / curve_count;
  string outputfile;
  nl.getParam("output_file", outputfile);

  double robot_radius;
  nl.getParam("robot_radius", robot_radius);
  double cell_size;
  nl.getParam("cell_size", cell_size);
  double v_max;
  nl.getParam("v_max", v_max);
  double a_max;
  nl.getParam("a_max", a_max);
  double lambda_hyperplanes;
  nl.getParam("lambda_hyperplanes", lambda_hyperplanes);
  double scaling_multiplier;
  nl.getParam("scaling_multiplier", scaling_multiplier);

  vector<string> robots;

  nl.getParam("robots", robots);

  for(int i=0; i<robots.size(); i++)
    cout << robots[i] << " ";
  cout << endl;

  int number_of_robots = (int)robots.size();

  int robot_id;
  nl.getParam("robot_id", robot_id);
  double scale_traj;
  nl.getParam("scale_traj", scale_traj);
  float lambda_min_der;
  nl.getParam("lambda_min_der", lambda_min_der);
  float lambda_min_der_vel;
  nl.getParam("lambda_min_der_vel", lambda_min_der_vel);
  float lambda_min_der_acc;
  nl.getParam("lambda_min_der_acc", lambda_min_der_acc);
  float lambda_min_der_jerk;
  nl.getParam("lambda_min_der_jerk", lambda_min_der_jerk);
  float lambda_min_der_snap;
  nl.getParam("lambda_min_der_snap", lambda_min_der_snap);

  bool enable_voronoi;
  nl.getParam("enable_voronoi", enable_voronoi);

  string alg;
  nl.getParam("algorithm", alg);

  string outFile;
  nl.getParam("outFile", outFile);

  std::ofstream out(outFile);

  srand(time(NULL));

  const double stoppingDistance = v_max * v_max / (2 * a_max);
  std::cout << "stoppingDistance: " << stoppingDistance << std::endl;


  ros::Rate rate((int)(1.0/dt + 0.5));

  trajectory original_trajectory;


  csv::Parser file(initial_trajectory_path);
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

  out << "environment:" << endl;
  out << "  robot_radius: " << robot_radius << endl;
  out << "  cellSize: " << cell_size << endl;
  out << "  obstacles:" << endl;

  vector<obstacle2D> cell_based_obstacles;
  for (size_t i = 0; i < og.max_i(); ++i) {
    for (size_t j = 0; j < og.max_j(); ++j) {
      OG::index idx(i, j);
      if (og.idx_occupied(idx)) {
        pair<double, double> coord = og.get_coordinates(idx);
        out << "    - [" << coord.first << "," << coord.second << "]" << endl;
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
  for(int i=0;i<number_of_robots; ++i) {
    positions[i] = vectoreuc(2);
  }
  positions[robot_id] = traj.eval(0);
  vectoreuc velocitydes = traj.neval(0,1);
  vectoreuc accelerationdes = traj.neval(0,2);



  double total_time_for_opt = 0;
  int total_count_for_opt = 0;




  ObjectiveBuilder::Init();


  std::chrono::time_point<std::chrono::high_resolution_clock> t_start, t_start_a_star, t_end_a_star, t_start_svm, t_end_svm, t_start_qp, t_end_qp;

  double everyone_reached = false;


  boost::asio::io_service ioserv;
  udp::socket socket(ioserv);
  socket.open(udp::v4());
  udp::endpoint ep(boost::asio::ip::address_v4::any(), 5007);
  boost::asio::socket_base::reuse_address option(true);
  socket.set_option(option);
  socket.bind(ep);


  boost::asio::ip::address multicast_address = boost::asio::ip::address::from_string("224.1.1.1");
  boost::asio::ip::multicast::join_group multicast_join(multicast_address);
  socket.set_option(multicast_join);


  std::array<char, 128> recv_buf;

  ROS_INFO("Waiting for startTrajectory command!");
  while(true) {
    int len = socket.receive(boost::asio::buffer(recv_buf));
    string socketdata(recv_buf.data(), len);
    cout << socketdata << endl;
    if(socketdata == "startTrajectory")
      break;
  }
  socket.set_option(boost::asio::ip::multicast::leave_group(multicast_address));
  socket.close();


  ros::Time start = ros::Time::now();

  out << "iterations:" << endl;

  // for(double ct = 0; ct <= total_t && ros::ok() ; ct+=dt) {
  while (ros::ok()) {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    ros::Duration duration = now - start;
    double ct = duration.toSec();
    if (ct > total_t) {
      break;
    }

    cerr << ct << " / " << total_t << endl;
    out << "  - time: " << ct << endl;

    getPositions(transformlistener, robots);
    out << "    positions:" << endl;
    for (size_t i = 0; i < number_of_robots; ++i) {
      out << "      - [" << positions[i][0] << "," << positions[i][1] << "]" << endl;
    }

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


    // output initially guessed controlpoints
    out << "    controlpointsGuessed:" << endl;
    for(int j=0; j<traj.size(); j++) {
      for(int k=0; k<ppc; k++) {
        out << "      - [" << y(0, j * ppc + k) << "," << y(1, j * ppc + k) << "]" << endl;
      }
    }

    // output hyperplanes
    out << "    hyperplanes:" << endl;
    for (const auto& hpc : hyperplaneConstraints) {
      out << "      - piece: " << hpc.piece << endl;
      out << "        normal: [" << hpc.normal[0] << "," << hpc.normal[1] << "]" << endl;
      out << "        dist: " << hpc.dist << endl;
    }

    do { // loop for temporal scaling

    // construct QP matrices
    ObjectiveBuilder ob(problem_dimension, pieceDurations);

    double remaining_time = std::max(total_t - ct, curve_count * dt * 1.1);
    double factor = lambda_min_der / remaining_time;
    ob.minDerivativeSquared(lambda_min_der_vel * factor, lambda_min_der_acc * factor, lambda_min_der_jerk * factor, lambda_min_der_snap * factor);

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


    // output computed controlpoints
    out << "    controlpoints:" << endl;
    for(int j=0; j<traj.size(); j++) {
      for(int k=0; k<ppc; k++) {
        out << "      - [" << y(0, j * ppc + k) << "," << y(1, j * ppc + k) << "]" << endl;
      }
    }

    auto t1 = Time::now();
    fsec fs = t1 - t_start;
    ms d = chrono::duration_cast<ms>(fs);
    total_time_for_opt += d.count();
    total_count_for_opt++;
    cout << "optimization time: " << d.count() << "ms" << endl;

    out << "    timeQP: " << d.count() << endl;
    out << "    timeAstar: " <<  chrono::duration_cast<ms>(t_end_a_star - t_start_a_star).count() << endl;
    out << "    timeSVM: " <<  chrono::duration_cast<ms>(t_end_svm - t_start_svm).count() << endl;



    vectoreuc despos = traj.neval(2*dt, 0);
    velocitydes = traj.neval(2*dt, 1);
    accelerationdes = traj.neval(2*dt, 2);

    desired_state_msg.position.x = despos[0];
    desired_state_msg.position.y = despos[1];
    desired_state_msg.velocity.x = velocitydes[0];
    desired_state_msg.velocity.y = velocitydes[1];
    desired_state_msg.acceleration.x = accelerationdes[0];
    desired_state_msg.acceleration.y = accelerationdes[1];

    desired_state_publisher.publish(desired_state_msg);

    out << "    desiredState:" << endl;
    out << "      - pos: [" << desired_state_msg.position.x << "," << desired_state_msg.position.y << "]" << endl;
    out << "      - vel: [" << desired_state_msg.velocity.x << "," << desired_state_msg.velocity.y << "]" << endl;
    out << "      - acc: [" << desired_state_msg.acceleration.x << "," << desired_state_msg.acceleration.y << "]" << endl;

    rate.sleep();
  }



  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;
#endif
  return 0;

}
