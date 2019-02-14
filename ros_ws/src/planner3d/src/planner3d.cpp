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
#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Land.h>
#include <crazyflie_driver/GoTo.h>
#include <tf/transform_listener.h>

#include <boost/asio/ip/udp.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/multicast.hpp>

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

using boost::asio::ip::udp;
using boost::asio::ip::address;

void waitForStartTrajectoryCommand()
{
  boost::asio::io_service ioserv;
  udp::socket socket(ioserv);
  socket.open(udp::v4());
  udp::endpoint ep(boost::asio::ip::address_v4::any(), 5008);
  boost::asio::socket_base::reuse_address option(true);
  socket.set_option(option);
  socket.bind(ep);

  boost::asio::ip::address multicast_address = boost::asio::ip::address::from_string("224.1.1.1");
  boost::asio::ip::multicast::join_group multicast_join(multicast_address);
  socket.set_option(multicast_join);

  std::array<char, 128> recv_buf;

  ROS_INFO("Waiting for startTrajectory command!");
  while(ros::ok()) {
    int len = socket.receive(boost::asio::buffer(recv_buf));
    string socketdata(recv_buf.data(), len);
    cout << socketdata << endl;
    if(socketdata == "startTrajectory")
      break;
  }
  socket.set_option(boost::asio::ip::multicast::leave_group(multicast_address));
  socket.close();
}

int main(int argc, char** argv) {

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

  tf::TransformListener transformlistener;

  // Read parameters
  ros::NodeHandle nl("~");

  string config_path;
  nh.getParam("config_path", config_path);

  vector<string> robots;
  nh.getParam("robots", robots);

  int robotIdx;
  nl.getParam("robotIdx", robotIdx);


  crazyflie_driver::FullState desired_state_msg;
  ros::Publisher desired_state_publisher = nh.advertise<crazyflie_driver::FullState>("/" + robots[robotIdx] + "/cmd_full_state", 1);

  ros::Publisher cmdStop_publisher = nh.advertise<std_msgs::Empty>("/" + robots[robotIdx] + "/cmd_stop", 1);


  // read config file
  ifstream cfg(config_path);
  if (!cfg) {
    std::cerr << "Couldn't find: " << config_path << endl;
    return 1;
  }


  json jsn = nlohmann::json::parse(cfg);


  const bool enable_voronoi = jsn["enable_voronoi"];
  const bool set_max_time = jsn["set_max_time_as_replan_period"];
  const bool enable_svm = jsn["enable_svm"];
  const unsigned int max_continuity = jsn["continuity_upto_degree"];
  const unsigned int curve_count = jsn["plan_for_curves"];
  const unsigned int points_per_curve = jsn["points_per_curve"];
  const string outputfile = jsn["simulation_output_file"];
  const string statoutputfile = jsn["statistics_output_file"];
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
  const double grid_min = jsn["grid_min"];
  const double grid_max = jsn["grid_max"];
  const double scaling_multiplier = jsn["scaling_multiplier"];
  const double additional_time = jsn["additional_time"];
  const double output_frame_dt = jsn["output_frame_dt"];
  const double lambda_hyperplane_cost = jsn["lambda_hyperplane_cost"];

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
  OccupancyGrid3D<double> OCCUPANCY_GRID(OBSTACLES, cell_size, grid_min, grid_max, grid_min, grid_max, grid_min, grid_max);
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


  ros::service::waitForService("/" + robots[robotIdx] + "/takeoff");
  ros::ServiceClient serviceTakeoff = nh.serviceClient<crazyflie_driver::Takeoff>("/" + robots[robotIdx] + "/takeoff");

  ros::service::waitForService("/" + robots[robotIdx] + "/land");
  ros::ServiceClient serviceLand = nh.serviceClient<crazyflie_driver::Land>("/" + robots[robotIdx] + "/land");

  ros::service::waitForService("/" + robots[robotIdx] + "/go_to");
  ros::ServiceClient serviceGoTo = nh.serviceClient<crazyflie_driver::GoTo>("/" + robots[robotIdx] + "/go_to");
  {
    crazyflie_driver::Takeoff srv;
    srv.request.groupMask = 0;
    srv.request.height = CURRENT_STATES[robotIdx][0](2);
    srv.request.duration = ros::Duration(2.0);
    serviceTakeoff.call(srv);
    ros::Duration(2.5).sleep();
  }
  {
    crazyflie_driver::GoTo srv;
    srv.request.groupMask = 0;
    srv.request.relative = false;
    srv.request.goal.x = CURRENT_STATES[robotIdx][0](0);
    srv.request.goal.y = CURRENT_STATES[robotIdx][0](1);
    srv.request.goal.z = CURRENT_STATES[robotIdx][0](2);
    srv.request.yaw = 0;
    srv.request.duration = ros::Duration(2.0);
    serviceGoTo.call(srv);
    ros::Duration(2.5).sleep();
  }

  waitForStartTrajectoryCommand();

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

    // update positions
    tf::StampedTransform transform;
    for(int i=0; i < robots.size(); i++) {
      transformlistener.lookupTransform("/world", robots[i], ros::Time(0), transform);

      CURRENT_STATES[i][0](0) = transform.getOrigin().x();
      CURRENT_STATES[i][0](1) = transform.getOrigin().y();
      CURRENT_STATES[i][0](2) = transform.getOrigin().z();
    }

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

    const unsigned int r = robotIdx;
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
      if(!VORONOI_HYPERPLANES.empty())
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


            /*
            * voronoi fix to handle voronoi/svm/continuity constraint
            * strange case
            */
            voronoi_fix<double, 3U>(corners, VORONOI_HYPERPLANES);

#ifdef ACT_DEBUG
            cout << "[DEBUG] Corners after Voronoi fix: ";
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


      // this is for adding hp costs later
      class hp_constraint {
        public:
          Hyperplane hp;
          unsigned int piece_idx;
          hp_constraint(const Hyperplane& h, unsigned int i): hp(h), piece_idx(i) {

          }
      };

      vector<hp_constraint> hp_constraints;

      /*
      * Add every constraint that is not related to duration of curves here
      * Later in the loop we will add constraints and costs which
      * changes with curve duration
      */
      vector<QPMatrices> QP_init = traj.getQPMatrices();


      //constain decision variables to the grid cube
      traj.extendQPDecisionConstraint(QP_init, grid_min, grid_max);

      // add voronoi constraints
      for(const auto& hp: VORONOI_HYPERPLANES) {
        traj.extendQPHyperplaneConstraint(QP_init, 0, hp, true, 1);
        hp_constraints.push_back(hp_constraint(hp, 0));
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


#ifdef ACT_GENERATE_OUTPUT_JSON
        json_svm_hyperplanes_of_piece_insert<double, 3U>(svm_hyperplanes_json["hyperplanes"], hyperplanes, piece_idx);
#endif
        for(const auto& hp: hyperplanes) {
          traj.extendQPHyperplaneConstraint(QP_init, piece_idx, hp, piece_idx == 0, 1);
          hp_constraints.push_back(hp_constraint(hp, piece_idx));
        }
      }


      // if dynamic limits are NOT violated after an optimization iteration,
      // this is set to true
      bool dynamic_limits_OK = false;

      // give it 5 tries to solve the problem
      unsigned int allowed_tries = 10;
      unsigned int tries_remaining = allowed_tries;

      Spline<double, 3U> planning_ready_traj(traj);

      while((!dynamic_limits_OK || LAST_QP_FAILED[r]) && tries_remaining > 0) {

        // Lets start to construct the QP
        vector<QPMatrices> qp = QP_init;

        // add initial point constraints
        for(unsigned int k = 0; k <= max_continuity; k++) {
          traj.extendQPBeginningConstraint(qp, k, CURRENT_STATES[r][k]);
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

        // add hp costs
        for(const auto& constraint: hp_constraints) {
          traj.extendQPHyperplaneCost(qp, constraint.piece_idx, constraint.hp,
            lambda_hyperplane_cost);
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
      auto jerk = traj.eval(dt * (SUCCESSIVE_FAILURE_COUNTS[r] + 1), 3);

      // auto pos = origtraj.eval(min(ct, TOTAL_TIMES[r]), 0);
      // auto vel = origtraj.eval(min(ct, TOTAL_TIMES[r]), 1);
      // auto acc = origtraj.eval(min(ct, TOTAL_TIMES[r]), 2);
      // auto jerk = origtraj.eval(min(ct, TOTAL_TIMES[r]), 3);

      // trajfile << pos << " " << vel << " " << acc << " " << jerk << std::endl;


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
      VectorDIM gravity(0.0, 0.0, 9.81);
      auto thrust = acc + gravity; // add gravity
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
      for(int c = 1; c <= max_continuity; c++) {
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

  // attempt to land...

  // 1. use goTo to update the high-level setpoint
  {
    crazyflie_driver::GoTo srv;
    srv.request.groupMask = 0;
    srv.request.relative = false;
    srv.request.goal.x = CURRENT_STATES[robotIdx][0](0);
    srv.request.goal.y = CURRENT_STATES[robotIdx][0](1);
    srv.request.goal.z = CURRENT_STATES[robotIdx][0](2);
    srv.request.yaw = 0;
    srv.request.duration = ros::Duration(0.1);
    serviceGoTo.call(srv);
  }

  // 2. wait until high-level took over from low level (2s in default firmware)
  ros::Duration(2.0).sleep();

  // 3. land
  {
    crazyflie_driver::Land srv;
    srv.request.groupMask = 0;
    srv.request.height = 0.05;
    srv.request.duration = ros::Duration(2.0);
    serviceLand.call(srv);
    ros::Duration(2.5).sleep();
  }

#if 0
  while (true) {
    desired_state_msg.header.seq += 1;
    desired_state_msg.header.stamp = ros::Time::now();

    const double v_max = 0.5;
    desired_state_msg.pose.position.z -= v_max * dt;
    if (desired_state_msg.pose.position.z < 0) {
      break;
    }

    desired_state_publisher.publish(desired_state_msg);

    rate.sleep();
  }

  for (size_t i = 0; i < 50; ++i) {
    std_msgs::Empty empty_msg;
    cmdStop_publisher.publish(empty_msg);
    rate.sleep();
  }
#endif


  return 0;

}
