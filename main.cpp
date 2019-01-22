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


  nlohmann::json jsn = nlohmann::json::parse(cfg);


  const bool enable_voronoi = jsn["enable_voronoi"];
  const bool set_max_time = jsn["set_max_time_as_replan_period"];
  const unsigned int max_continuity = jsn["continuity_upto_degree"];
  const unsigned int curve_count = jsn["plan_for_curves"];
  const unsigned int points_per_curve = jsn["points_per_curve"];
  const string outputfile = jsn["output_file"];
  const string alg = jsn["algorithm"];
  const string initial_trajectories_path = jsn["trajectories"];
  const string obstacles_path = jsn["obstacles"];
  const double scale_traj = jsn["scale_traj"];
  const double dt = jsn["replan_period"];
  const double robot_radius = jsn["robot_radius"];
  const double cell_size = jsn["cell_size"];
  const double desired_horizon = jsn["planning_horizon"];
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
  const double output_frame_dt = jsn["output_frame_dt"];

  const double step = 0.01; // general purpose step

  ofstream LOG("log", ofstream::out);


  using VectorDIM = Eigen::Matrix<double, 3U, 1U>;
  using Hyperplane = Eigen::Hyperplane<double, 3U>;
  using AlignedBox = PointCloud<double, 3U>::AlignedBox;


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
  OccupancyGrid3D<double> OCCUPANCY_GRID(OBSTACLES, cell_size);


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
    for(int c = 0; c < max_continuity; c++) {
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

  double SIMULATION_DURATION = MAX_TOTAL_TIME + additional_time;
  for(double ct = 0; ct <= SIMULATION_DURATION; ct += dt) {
    LOG << "Current Time: " << ct << "/" << SIMULATION_DURATION << endl;
    for(unsigned int r = 0; r < robot_count; r++) {
      LOG << "Robot: " << r << endl;

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

      /*
      * Calculate voronoi for the current robot
      */
      vector<Hyperplane> VORONOI_HYPERPLANES;
      if(enable_voronoi) {
        VORONOI_HYPERPLANES = voronoi<double, 3U>(CURRENT_STATES, r, robot_radius);
      }


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


      /*
      * Check if original trajectory is occupied from ct to ct + desired_horizon
      */
      bool original_trajectory_occupied = origtraj.
        intersects(OCCUPANCY_GRID._occupied_boxes,
          min(TOTAL_TIMES[r], ct), min(TOTAL_TIMES[r], ct + desired_horizon),
          robot_radius);


      /*
      * Check if previous trajectory is occupied from 0 to tau
      */
      bool previous_trajectory_occupied = traj.
        intersects(OCCUPANCY_GRID._occupied_boxes,
          0, traj.totalSpan(), robot_radius);


      /*
      * true if discrete planning is needed
      */
      bool discrete_planning_needed =
        previous_trajectory_first_piece_outside_voronoi ||
        original_trajectory_occupied ||
        previous_trajectory_occupied;

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

        // if the original trajectory is unoccupied at target_time
        if(target_time_valid) {
          // try discrete planning
          // it may fail if there is no path
          // or it may succeed if there is a path
          // if it succeeds set discrete_planning_done to True
          // else set it to False
          const VectorDIM& currentPos = state[0];
          const VectorDIM targetPos = origtraj.eval(target_time, 0);

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

          if(curve_count > corners.size() - 1) {
            // we need to split segments since there are not enough segments
            // for each curve
            bestSplitSegments<double, 3U>(corners, curve_count);
          }

          // corners are ready here
          // load the trajectory with first curve_count segments
          // TODO!!!
        }
      }


      if(discrete_planning_done) {

      } else {

      }



      /*
      * Remove other robots from occupancy grid
      */
      OCCUPANCY_GRID.undoRobotChanges(OG_CHANGES);
    }

  }



  LOG.close();
  return 0;

}
