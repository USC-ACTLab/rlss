#include <iostream>
#include <nlopt.hpp>
#include <armadillo>
#include <fstream>
#include <experimental/filesystem>
#include <vector>
#include "csv/CSVparser.hpp"
#include <boost/geometry.hpp> // for geometry algorithms
#include <boost/geometry/geometries/geometries.hpp> // for geometry types
#include <algorithm>
#include <cassert>
#include <cstdarg>
#include "vectoreuc.h"
#include <chrono>
#include "trajectory.h"
#include "curve.h"
#include "hyperplane.h"
#include "obstacle.h"
#include "optimization.h"
#include "cxxopts.hpp"
#include "json.hpp"
#include <chrono>
#include "svmoptimization.h"
#include "utility.h"



#define PATHREPLAN_EPSILON 0.00001

namespace fs = std::experimental::filesystem::v1;
using namespace std;
using namespace boost::geometry;


typedef chrono::high_resolution_clock Time;
typedef chrono::milliseconds ms;
typedef chrono::duration<float> fsec;


int main(int argc, char** argv) {

  string config_path;
  cxxopts::Options options("Path Replanner", "Path replanner for UAV swarms");
  options.add_options()
    ("cfg", "Config file", cxxopts::value<std::string>()->default_value("../config.json")),
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
  string outputfile = jsn["output_file"];


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



  /*cout << "initial_trajectories_path: " << initial_trajectories_path << endl
       << "obstacles_path: " << obstacles_path << endl
       << "dt: " << dt << endl
       << "integral_stopval: " << integral_stopval << endl
       << "relative_integral_stopval: " << relative_integral_stopval << endl
       << "problem_dimension: " << problem_dimension << endl
       << "points per curve: " << ppc << endl
       << "continuity upto: " << max_continuity << endl
       << "set max time: " << set_max_time << endl
       << "output file: " << outputfile << endl << endl;

       int a; cin >> a;*/
  srand(time(NULL));


  vector<trajectory> trajectories;
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
    trajectories.push_back(trj);
  }


  vector<trajectory> orijinal_trajectories = trajectories;

/*
  cout << "Number of trajectories: " << trajectories.size() << endl;

  for(int i=0; i<trajectories.size(); i++) {
    cout << "\ttraj#" << i << endl;
    cout << "\t\tnumber of curves: " << trajectories[i].size() << endl;
  }
*/


  vector<obstacle2D> obstacles;

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
    o.ch_planes();
    obstacles.push_back(o);
  }



  double total_t = 0;
  for(int i=0; i<trajectories[0].size(); i++) {
    total_t += trajectories[0][i].duration;
  }


  vector<vectoreuc> positions(trajectories.size());

  double printdt = jsn["print_dt"];

  output_json["dt"] = printdt;
  output_json["number_of_robots"] = trajectories.size();
  int steps_per_cycle = (dt / printdt)+0.5;
  int output_iter = 0;


  double total_time_for_opt = 0;
  int total_count_for_opt = 0;


  for(int i=0; i<trajectories.size(); i++) {
    for(double t=0; t<=total_t; t+=printdt) {
      vectoreuc eu = orijinal_trajectories[i].eval(t);
      output_json["originals"][i]["x"].push_back(eu[0]);
      output_json["originals"][i]["y"].push_back(eu[1]);
    }
  }

  for(double ct = 0; ct <= total_t; ct+=dt) {
    for(int i=0; i<trajectories.size(); i++) {
      positions[i] = trajectories[i].eval(ct);
      output_json["points"][output_iter].push_back(positions[i].crds);
      //out << i << " (" << positions[i][0] << "," << positions[i][1] << ")" << endl;
    }

    for(int i=0; i<trajectories.size(); i++ ) {
      auto t0 = Time::now();

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

      /*
        number of curves \times number of points per curve \times problem_dimension
      */

      unsigned varcount = trajectories[i].size() * ppc * problem_dimension;
      //nlopt::opt problem(nlopt::AUGLAG, varcount);
      //problem.set_local_optimizer(local_opt);
      nlopt::opt problem;
      if(alg == "MMA") {
        problem = nlopt::opt(nlopt::LD_MMA, varcount);
      } else if(alg == "SLSQP") {
        problem = nlopt::opt(nlopt::LD_SLSQP, varcount);
      } else {
        throw "no such algorithm";
      }
      problem_data data;
      data.current_t = ct;
      data.time_horizon = hor;
      data.original_trajectory = &(orijinal_trajectories[i]);
      //data.original_trajectory = &(trajectories[i]);
      data.problem_dimension = problem_dimension;
      data.ppc = ppc;
      data.tt = total_t;

      problem.set_min_objective(optimization::objective, (void*)&data);

      alt_obj_data alt_data;
      alt_data.pdata = &data;
      vectoreuc OBJPOS = orijinal_trajectories[i].neval(min(ct+hor, total_t), 0);
      vectoreuc OBJVEL = orijinal_trajectories[i].neval(min(ct+hor, total_t), 1);
      vectoreuc OBJACC = orijinal_trajectories[i].neval(min(ct+hor, total_t), 2);
      alt_data.pos = &OBJPOS;
      alt_data.vel = &OBJVEL;
      alt_data.acc = &OBJACC;

    //  problem.set_min_objective(optimization::alt_objective, (void*)&alt_data);


      /* all control points should be in range [-10, 10].
        since curves are bezier, resulting curve will be inside the corresponding rectangle
        */
      vector<double> lower_bounds(varcount);
      vector<double> upper_bounds(varcount);
      for(int j=0; j<varcount; j+=2) {
        lower_bounds[j] = lower_bounds[j+1] = -10;
        upper_bounds[j] = upper_bounds[j+1] = 10;
      }
      problem.set_lower_bounds(lower_bounds);
      problem.set_upper_bounds(upper_bounds);

      // just to delete them from heap later.
      vector<voronoi_data*> vdpts;


      for(int j=0; j<voronoi_hyperplanes.size(); j++) {
        hyperplane& plane = voronoi_hyperplanes[j];


        double current_time = ct;
        double end_time  = ct+dt;
        //double end_time  = ct+dt;

        int idx = 0;

        while(idx < trajectories[i].size() && current_time >= trajectories[i][idx].duration) {
          current_time -= trajectories[i][idx].duration;
          end_time -= trajectories[i][idx].duration;
          idx++;
        }

        current_time = max(current_time, 0.0); // to resolve underflows. just for ct == 0.

        while(end_time > 0 && idx < trajectories[i].size()) {
          double cend_time = min(end_time, trajectories[i][idx].duration);
          double curvet = 0;
          double step = trajectories[i][idx].duration / (trajectories[i][idx].size()-1);
          for(int p=0; p<trajectories[i][idx].size(); p++) {
            //if((curvet > current_time || fabs(curvet - current_time) < PATHREPLAN_EPSILON) /*&& (curvet < cend_time || fabs(curvet - cend_time) < PATHREPLAN_EPSILON)*/) {
              voronoi_data* vd = new voronoi_data;
              vd->pdata = &data;
              vd->plane = plane;
              vd->curve_idx = idx;
              vd->point_idx = p;
              problem.add_inequality_constraint(optimization::voronoi_constraint, (void*)vd, 0);
              vdpts.push_back(vd);
          //}
            curvet += step;
          }


          end_time -= cend_time;
          current_time = 0;
          idx++;
        }

      }

      // just to delete them from heap later.
      /* obstacle avoidance v1 */
      vector<obstacle_data*> obspts;

      for(int j=0; j<obstacles.size(); j++) {
        obstacle_data* od = new obstacle_data;
        od->pdata = &data;
        od->hps = &(obstacles[j].chplanes);

        problem.add_inequality_constraint(optimization::obstacle_constraint, (void*)od, obstacle_tolerance);
        obspts.push_back(od);
      }


      vector<svmopt_data*> svmoptpts;
      vector<svmobs_data*> svmobspts;
      vector<svmrobot_data*> svmrobotpts;
      /* SVM STUFF
      for(int j=0; j<obstacles.size(); j++) {
        nlopt::opt obstaclesvm(nlopt::LD_SLSQP, problem_dimension+1);
        svmopt_data* objdata = new svmopt_data;
        vectoreuc vel = trajectories[i].neval(ct, 1);
        objdata->velocity_direction = vel.normalized();
        objdata->alpha = 1;
        objdata->beta = 1;
        svmoptpts.push_back(objdata);
        obstaclesvm.set_min_objective(svmoptimization::objective, (void*)objdata);

        for(int k=0; k<obstacles[j].ch.size()-1; k++) {
          svmobs_data* svmobsdata = new svmobs_data;
          svmobspts.push_back(svmobsdata);
          svmobsdata->obspt = &obstacles[j][obstacles[j].ch[k]];
          obstaclesvm.add_inequality_constraint(svmoptimization::obspointconstraint, (void*)svmobsdata, 0.00001);
          cout << "obs: " << obstacles[j][obstacles[j].ch[k]] << endl;
        }

        svmrobot_data* svmrobotdata = new svmrobot_data;
        svmrobotdata->robotpt = &positions[i];
        cout << "ro: " << positions[i] << endl;
        svmrobotpts.push_back(svmrobotdata);
        obstaclesvm.add_inequality_constraint(svmoptimization::robotpointconstraint, (void*)svmrobotdata,0.00001);
        obstaclesvm.set_ftol_rel(0.00000000001);
        vector<double> init_points(problem_dimension+1);
        for(int k=0; k<init_points.size(); k++) {
          init_points[k] = 1;
        }
        double f_opt;
        nlopt::result res;
        try {
          res = obstaclesvm.optimize(init_points, f_opt);
        } catch(nlopt::roundoff_limited& e) {
        }
        cout << "pla: ";
        for(int j=0; j<init_points.size(); j++) {
          cout << init_points[j] << " ";
        }
        cout << endl;
        for(int l=0; l<svmobspts.size(); l++) {
          vector<double> a;
          cout << svmoptimization::obspointconstraint(init_points, a, svmobspts[l]) << endl;
        }
        cout << "svm ret: " << res << " val: " << f_opt << endl;
        hyperplane sep_plane;
        vectoreuc normal(2);
        normal[0] = init_points[0];
        normal[1] = init_points[1];
        double norm = normal.L2norm();
        sep_plane.normal = normal.normalized();
        sep_plane.distance = init_points[2]/norm;

        double current_time = ct;
        double end_time  = ct+obstacle_horizon;

        int idx = 0;

        while(idx < trajectories[i].size() && current_time >= trajectories[i][idx].duration) {
          current_time -= trajectories[i][idx].duration;
          end_time -= trajectories[i][idx].duration;
          idx++;
        }

        current_time = max(current_time, 0.0); // to resolve underflows. just for ct == 0.
        int cnt = 0;
        while(end_time >= 0 && idx < trajectories[i].size()) {
          double cend_time = min(end_time, trajectories[i][idx].duration);
          double curvet = 0;
          double step = trajectories[i][idx].duration / (trajectories[i][idx].size()-1);
          for(int p=0; p<trajectories[i][idx].size(); p++) {
            if((curvet >= current_time || fabs(curvet - current_time) < PATHREPLAN_EPSILON) && (curvet <= cend_time || fabs(curvet - cend_time) < PATHREPLAN_EPSILON)) {
              voronoi_data* vd = new voronoi_data;
              vd->pdata = &data;
              vd->plane = sep_plane;
              vd->curve_idx = idx;
              vd->point_idx = p;
              problem.add_inequality_constraint(optimization::voronoi_constraint, (void*)vd, 0);
              vdpts.push_back(vd);
              cnt++;
            }
            curvet += step;
          }


          end_time -= cend_time;
          current_time = 0;
          idx++;
        }
        cout << "number of effected points: " << cnt << endl;
      }
*/

      vector<continuity_data*> contpts;
      vector<maxnvalue_data*> maxpts;
      /*vector<double> continuity_tolerances(problem_dimension);
      for(int i=0; i<problem_dimension; i++) {
        continuity_tolerances[i] = continuity_tol;
      }*/
      double current_time = ct;
      double end_time = ct+hor;
      int start_curve = 0;
      int end_curve = 0;
      int j;
      for(j=0; j<trajectories[i].size(); j++) {
        if(current_time > trajectories[i][j].duration) {
          current_time -= trajectories[i][j].duration;
          end_time -= trajectories[i][j].duration;
          start_curve++;
          end_curve++;
        } else {
          break;
        }
      }

      for(; j<trajectories[i].size(); j++) {
        if(end_time > trajectories[i][j].duration) {
          end_time -= trajectories[i][j].duration;
          end_curve++;
        } else {
          break;
        }
      }

      end_curve = min((int)trajectories[i].size()-1, end_curve);
      //start_curve = max(0, start_curve-1);
      for(int j=start_curve; j <= end_curve && j<trajectories[i].size()-1; j++) {
        for(int n=0; n<=max_continuity; n++) {
        // nth degree continuity
          continuity_data* cd = new continuity_data;
          cd->pdata = &data;
          cd->n = n;
          cd->c = j;

          problem.add_inequality_constraint(optimization::continuity_constraint, (void*)cd, continuity_tols[n]);
          contpts.push_back(cd);


        }


      }
      for(int n=start_curve; n<=end_curve; n++) {
        for(int j=0; j<dynamic_constraint_degrees.size(); j++) {
          int deg = dynamic_constraint_degrees[j];
          double max_val = dynamic_constraint_max_values[j];

          maxnvalue_data* dd = new maxnvalue_data;
          dd->pdata = &data;
          dd->cidx = n;
          dd->degree = deg;
          dd->max_val = max_val;

          problem.add_inequality_constraint(optimization::maximum_nvalue_of_curve, (void*)dd, 0);
          maxpts.push_back(dd);
        }
      }
      vector<point_data*> pointpts;

      for(int j=0; j<=max_initial_point_degree; j++) {
        point_data* pd = new point_data;
        pd->pdata = &data;
        pd->point = trajectories[i].neval(ct, j);
        /*if(j==0) {
          pd->point[0] += fRand(-0.002, 0.002);
          pd->point[1] += fRand(-0.002, 0.002);
        }*/
        pd->time = ct;
        pd->degree = j;

        problem.add_inequality_constraint(optimization::point_constraint, (void*)pd, initial_point_tols[j]);
        pointpts.push_back(pd);
      }


      /* if integral is less than this value, stop.*/
      problem.set_stopval(integral_stopval);

      /* if objective function changes relatively less than this value, stop.*/
      problem.set_ftol_abs(relative_integral_stopval);

      if(set_max_time) {
        problem.set_maxtime(dt);
      }
      //problem.set_maxtime(5);


      vector<double> initial_values;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            initial_values.push_back(trajectories[i][j][k][p] + fRand(-0.002, 0.002));
          }
        }
      }
      //initial_values[initial_values.size()-1]+=frand(-0.1, 0.1);

      //cout << initial_values.size() << endl;

      double opt_f;

      int initidx = 0;
      //problem.set_maxeval(1000);

      /*for(int j=0; j<trajectories[i].size(); j++) {
        cout << trajectories[i][j].duration;
        for(int k=0; k<trajectories[i][j].size(); k++) {
          for(int p=0; p<trajectories[i][j][k].size(); p++) {
            cout << "," << initial_values[initidx++];
          }
        }
        cout << endl;
      }
      cout << endl << endl;*/


      nlopt::result res;
      try {
        res = problem.optimize(initial_values, opt_f);
      } catch(nlopt::roundoff_limited& e) {
        cout << "roundoff_limited" << endl;
      } catch(std::runtime_error& e) {
        cout << "runtime error " << i << endl;
        cout << e.what()<< endl;
      }

      initidx = 0;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            trajectories[i][j][k][p] = initial_values[initidx++];
          }
        }
      }


      cout << "nlopt result: " << res << " objective value: " << opt_f << endl;

      /*int a; cin >> a;*/

      /*initidx = 0;
      for(int j=0; j<trajectories[i].size(); j++) {
        cout << trajectories[i][j].duration;
        for(int k=0; k<trajectories[i][j].size(); k++) {
          for(int p=0; p<trajectories[i][j][k].size(); p++) {
            cout << "," << initial_values[initidx++];
          }
        }
        cout << endl;
      }
      cout << endl << endl;*/




      for(int j=0; j<vdpts.size(); j++) {
        delete vdpts[j];
      }
      for(int j=0; j<obspts.size(); j++) {
        delete obspts[j];
      }
      for(int j=0; j<contpts.size(); j++) {
        delete contpts[j];
      }
      for(int j=0; j<pointpts.size(); j++) {
        delete pointpts[j];
      }
      for(int j=0; j<maxpts.size(); j++) {
        delete maxpts[j];
      }
      for(int j=0; j<svmoptpts.size(); j++) {
        delete svmoptpts[j];
      }
      for(int j=0; j<svmobspts.size(); j++) {
        delete svmobspts[j];
      }
      for(int j=0; j<svmrobotpts.size(); j++) {
        delete svmrobotpts[j];
      }
      auto t1 = Time::now();
      fsec fs = t1 - t0;
      ms d = chrono::duration_cast<ms>(fs);
      total_time_for_opt += d.count();
      total_count_for_opt++;
      cout << "optimization time: " << d.count() << "ms" << endl;

      for(double t = ct; t<min(total_t, ct+hor); t+=printdt) {
        vectoreuc ev = trajectories[i].eval(t);
        output_json["planned_trajs"][output_iter][i]["x"].push_back(ev[0]);
        output_json["planned_trajs"][output_iter][i]["y"].push_back(ev[1]);
      }
    }
    output_iter++;
    int v = 0;
    for(double t = ct + printdt; t<=total_t && v < steps_per_cycle - 1; t+=printdt, v++) {
      for(int i=0; i<trajectories.size(); i++) {
        vectoreuc vec = trajectories[i].eval(t);
        output_json["points"][output_iter].push_back(vec.crds);
        //out << i << " (" << vec[0] << "," << vec[1] << ")" << endl;
      }
      output_iter++;
    }
  }



  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;
  out << output_json << endl;

  out.close();
  return 0;

}
