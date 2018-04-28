#include <iostream>
#include <nlopt.hpp>
#include <fstream>
#include <experimental/filesystem>
#include <vector>
#include "csv/CSVparser.hpp"
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
#include "edt.h"
#include "edtv2.h"

#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>


#define PATHREPLAN_EPSILON 0.00001

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

  edt distance_transform(0.01, -10, 10, -10, 10);
  distance_transform.construct(&obstacles);

  edtv2 distance_transformv2(0.01, -10, 10, -10, 10, -0.20);
  distance_transformv2.construct(&obstacles);



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
    velocities[i] = trajectories[i].neval(0, 1);
    accelerations[i] = trajectories[i].neval(0, 2);
  }

  for(int i=0; i<original_trajectories.size(); i++) {
    for(double t=0; t<=total_t; t+=printdt) {
      vectoreuc eu = original_trajectories[i].eval(t);
      output_json["originals"][i]["x"].push_back(eu[0]);
      output_json["originals"][i]["y"].push_back(eu[1]);
    }
  }

  for(double ct = 0; ct <= total_t ; ct+=dt) {

    cout << ct << " / " << total_t << endl;

    for(int i=0; i<original_trajectories.size(); i++ ) {
      cout << "traj " << i << " start" << endl;
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


      unsigned varcount = curve_count * ppc * problem_dimension;
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
      data.original_trajectory = &(original_trajectories[i]);
      data.current_trajectory = &(trajectories[i]);
      data.problem_dimension = problem_dimension;
      data.ppc = ppc;
      data.tt = total_times[i];
      data.curve_count = curve_count;
      data.time_per_curve = time_per_curve;


      alt_obj_data alt_data;
      alt_data.pdata = &data;
      vectoreuc OBJPOS = original_trajectories[i].neval(min(ct+hor, total_times[i]), 0);
      vectoreuc OBJVEL = original_trajectories[i].neval(min(ct+hor, total_times[i]), 1);
      vectoreuc OBJACC = original_trajectories[i].neval(min(ct+hor, total_times[i]), 2);
      alt_data.pos = &OBJPOS;
      alt_data.vel = &OBJVEL;
      alt_data.acc = &OBJACC;

    //  problem.set_min_objective(optimization::alt_objective, (void*)&alt_data);

      problem.set_min_objective(optimization::pos_energy_combine_objective, (void*)&alt_data);


      edt_data edata;
      edata.pdata = &data;
      edata.distance_transform = &distance_transform;

      problem.add_inequality_constraint(optimization::edt_constraint, (void*)&edata, 0.0000001);


      edt_collision_data edatacol;
      edatacol.pdata = &data;
      edatacol.distance_transform = &distance_transformv2;


      alt_edt_combination_data aecombdata;

      aecombdata.edt = &edatacol;
      aecombdata.alt = &alt_data;

      //problem.set_min_objective(optimization::pos_energy_edt_combine_objective, (void*)&aecombdata);

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


        double current_time = 0;
        double end_time  = 2*dt;
        //double end_time  = ct+dt;

        int idx = 0;

        while(end_time > 0 && idx < trajectories[i].size()) {
          double cend_time = min(end_time, trajectories[i][idx].duration);
          double curvet = 0;
          for(int p=0; p<ppc; p++) {
            voronoi_data* vd = new voronoi_data;
            vd->pdata = &data;
            vd->plane = plane;
            vd->curve_idx = idx;
            vd->point_idx = p;
            problem.add_inequality_constraint(optimization::voronoi_constraint, (void*)vd, 0);
            vdpts.push_back(vd);
          }
          end_time -= cend_time;
          idx++;
        }

      }


      vector<continuity_data*> contpts;

      for(int j=0; j < curve_count-1; j++) {
        for(int n=0; n<=max_continuity; n++) {
          continuity_data* cd = new continuity_data;
          cd->pdata = &data;
          cd->n = n;
          cd->c = j;
          problem.add_inequality_constraint(optimization::continuity_constraint, (void*)cd, continuity_tols[n]);
          contpts.push_back(cd);
        }
      }

      vector<point_data*> pointpts;


      if(max_initial_point_degree >= 0) {
        point_data* pos_point_data = new point_data;
        pos_point_data->pdata = &data;
        pos_point_data->point = positions[i];
        pos_point_data->time = 0;
        pos_point_data->degree = 0;
        problem.add_inequality_constraint(optimization::point_constraint, (void*)pos_point_data, initial_point_tols[0]);
        pointpts.push_back(pos_point_data);
      }



      if(max_initial_point_degree >= 1) {
        point_data* vel_point_data = new point_data;
        vel_point_data->pdata = &data;
        vel_point_data->point = velocities[i];
        vel_point_data->time = 0;
        vel_point_data->degree = 1;

        problem.add_inequality_constraint(optimization::point_constraint, (void*)vel_point_data, initial_point_tols[1]);
        pointpts.push_back(vel_point_data);
      }

      if(max_initial_point_degree >= 2) {
        point_data* acc_point_data = new point_data;
        acc_point_data->pdata = &data;
        acc_point_data->point = accelerations[i];
        acc_point_data->time = 0;
        acc_point_data->degree = 2;

        problem.add_inequality_constraint(optimization::point_constraint, (void*)acc_point_data, initial_point_tols[2]);
        pointpts.push_back(acc_point_data);
      }









      /* if integral is less than this value, stop.*/
      problem.set_stopval(integral_stopval);

      /* if objective function changes relatively less than this value, stop.*/
      problem.set_ftol_abs(relative_integral_stopval);

      if(set_max_time) {
        problem.set_maxtime(dt);
      }


      vector<double> initial_values;
      //trajectories[i][0][0] = positions[i];
      for(int j=0; j<curve_count; j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            initial_values.push_back(trajectories[i][j][j][p]/* + fRand(-0.004, 0.004)*/);
          }
        }
      }


      double opt_f;
      int initidx = 0;
      nlopt::result res;


      try {
        res = problem.optimize(initial_values, opt_f);
      } catch(nlopt::roundoff_limited& e) {
        cout << "roundoff_limited" << endl;
      } catch(std::runtime_error& e) {
        cout << "runtime error " << i << endl;
        cout << e.what()<< endl;
      }

      cout << "nlopt result: " << res << " objective value: " << opt_f << endl;



      initidx = 0;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            trajectories[i][j][k][p] = initial_values[initidx++];
          }
        }
      }

      cout << trajectories[i].eval(0) << endl;
      cout << positions[i] << endl;
      cout << "--" << endl;


      // Sanity check of constraints:
      vector<double> dummyGradient(initial_values.size());

      for (auto& vd : vdpts) {
        double val = optimization::voronoi_constraint(initial_values, dummyGradient, vd);
        if (val > 1e-3) {
          std::stringstream sstr;
          sstr << "Voronoi constraint violated: " << val;
          std::cout << sstr.str() << std::endl;
          // throw std::runtime_error(sstr.str());
        }
      }

      for (auto& cd : contpts) {
        double val = optimization::continuity_constraint(initial_values, dummyGradient, cd);
        if (val > continuity_tols[cd->n]) {
          std::stringstream sstr;
          sstr << "continuity constraint violated: " << val << " (max: " << continuity_tols[cd->n] << ", degree: " << cd->n << ")";
          std::cout << sstr.str() << std::endl;
          // throw std::runtime_error(sstr.str());
        }
      }

      int a = 0;
      for (auto& pd : pointpts) {
        double val = optimization::point_constraint(initial_values, dummyGradient, pd);
        if (val > initial_point_tols[a]) {
          std::stringstream sstr;
          sstr << "point constraint violated: " << val << " (max: " << initial_point_tols[a] << ", degree: " << a << ")";
          std::cout << sstr.str() << std::endl;
          // throw std::runtime_error(sstr.str());
        }
        a++;
      }


      for(int j=0; j<vdpts.size(); j++) {
        delete vdpts[j];
      }
      for(int j=0; j<contpts.size(); j++) {
        delete contpts[j];
      }
      for(int j=0; j<pointpts.size(); j++) {
        delete pointpts[j];
      }
      auto t1 = Time::now();
      fsec fs = t1 - t0;
      ms d = chrono::duration_cast<ms>(fs);
      total_time_for_opt += d.count();
      total_count_for_opt++;
      cout << "optimization time: " << d.count() << "ms" << endl;


      for(double t = 0; t<=hor; t+=printdt) {
        vectoreuc ev = trajectories[i].eval(t);
        output_json["planned_trajs"][output_iter][i]["x"].push_back(ev[0]);
        output_json["planned_trajs"][output_iter][i]["y"].push_back(ev[1]);
      }

    }
    vectoreuc vec;
    for(int v = 0; v < steps_per_cycle; v++) {
      for(int i=0; i<trajectories.size(); i++) {
        vec = trajectories[i].neval(v*printdt, 0);
        output_json["points"][output_iter].push_back(vec.crds);
      }
      output_iter++;
    }

    cout << vec << endl;


    for(int i=0; i<trajectories.size(); i++) {
      positions[i] = trajectories[i].neval(dt, 0);
      velocities[i] = trajectories[i].neval(dt, 1);
      accelerations[i] = trajectories[i].neval(dt, 2);
    }

    cout << positions[positions.size() - 1] << endl;
  }



  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;
  out << output_json << endl;

  out.close();
  return 0;

}
