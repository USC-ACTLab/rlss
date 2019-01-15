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



namespace fs = std::experimental::filesystem;
using std::string;
using std::cout;
using std::endl;
using std::ifstream;
using namespace ACT;
using namespace splx;
using std::max;
using std::min;
using std::pair;





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
  const int max_continuity = jsn["continuity_upto_degree"];
  const int curve_count = jsn["plan_for_curves"];
  const int points_per_curve = jsn["points_per_curve"];
  const string outputfile = jsn["output_file"];
  const string alg = jsn["algorithm"];
  const string initial_trajectories_path = jsn["trajectories"];
  const string obstacles_path = jsn["obstacles"];
  const double scale_traj = jsn["scale_traj"];
  const double dt = jsn["replan_period"];
  const double robot_radius = jsn["robot_radius"];
  const double cell_size = jsn["cell_size"];
  const double hor = jsn["planning_horizon"];
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


  using VectorDIM = Spline<double, 3U>::VectorDIM;
  using Hyperplane = Eigen::Hyperplane<double, 3U>;
  using AlignedBox = PointCloud<double, 3>::AlignedBox;


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
    cout << path << endl;
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
        //cout << vec << endl;
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
    for(unsigned int i = 0; i < curve_count; i++) {
      Bezier<double, 3U> bez(traj.getPiece(min(i, traj.numPieces() - 1)));
      traj.addPiece(bez);
    }
  }

  double SIMULATION_DURATION = MAX_TOTAL_TIME + additional_time;
  for(double ct = 0; ct <= SIMULATION_DURATION; ct += dt) {
    for(unsigned int r = 0; r < robot_count; r++) {

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
      * Check if the first piece of the previous plan is outside of the
      * buffered voronoi cell
      */
      bool first_piece_outside_voronoi = false;
      for(const auto& hp: VORONOI_HYPERPLANES) {
        // DO DIS MATE
      }


      /*
      * Check if original trajectory is occupied from ct to ct + hor
      */
      bool original_trajectory_occupied = false;
      for(auto& alignedBox: OCCUPANCY_GRID._occupied_boxes) {
        if(ORIGINAL_TRAJECTORIES[r].intersects(alignedBox,
            min(TOTAL_TIMES[r], ct), min(TOTAL_TIMES[r], ct + hor), robot_radius)) {
          original_trajectory_occupied = true;
          break;
        }
      }





      /*
      * Remove other robots from occupancy grid
      */
      OCCUPANCY_GRID.undoRobotChanges(OG_CHANGES);
    }

  }




  return 0;

}
