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
#include <random>
#include "svmoptimization.h"
#include "utility.h"
#include "edt.h"
#include "edtv2.h"

#define USE_IPOPT 0
#define USE_QP    1
#define QP_SOLVER_QPOASES 0
#define QP_SOLVER_OSQP    1
#define QP_SOLVER QP_SOLVER_QPOASES

#if USE_IPOPT
#include <coin/IpIpoptApplication.hpp>
#include "ipopt_optimize.h"
using namespace Ipopt;
#endif

#if USE_QP
#include "qp_optimize.h"

  #if QP_SOLVER == QP_SOLVER_QPOASES
  #include <qpOASES.hpp>
  #endif
  #if QP_SOLVER == QP_SOLVER_OSQP
  #include "osqp.h"
  #endif
#endif

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

#if USE_QP
  ObjectiveBuilder::Init();
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0,0.001);
#endif

  double everyone_reached = false;

  for(double ct = 0; ct <= total_t /*!everyone_reached*/ ; ct+=dt) {

    cout << ct << " / " << total_t << endl;

    for(int i=0; i<original_trajectories.size(); i++ ) {
      cout << "traj " << i << " start " << ct << " / " << total_times[i] << endl;
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

      // if (ct >= total_times[i] - 3 * dt) {
      //   continue;
      // }


      unsigned varcount = curve_count * ppc * problem_dimension;

#if USE_QP

      // Create objective (min energy + reach goal)
      ObjectiveBuilder ob(problem_dimension, curve_count);
      ob.minDerivativeSquared(1, 0, 5e-3, 0);

      for (size_t j = 0; j < curve_count; ++j) {
        vectoreuc OBJPOS = original_trajectories[i].neval(min(ct+hor * (j+1)/(double)curve_count, total_times[i]), 0);
        Vector targetPosition(problem_dimension);
        targetPosition << OBJPOS[0], OBJPOS[1];
        ob.endCloseTo(j, 100 * (j+1), targetPosition);
      }

      // y are our control points (decision variable)
      // Vector y(numVars);
      Matrix y(problem_dimension, 8 * curve_count);

      // initialize y with previous solution
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            y(p, j * ppc + k) = trajectories[i][j][k][p] + pos_diffs[i][p];
          }
        }
      }

      // lower and upper bound for decision variables (i.e., workspace)
      const size_t numVars = problem_dimension * 8 * curve_count;
      Vector lb(numVars);
      lb.setConstant(-10);
      Vector ub(numVars);
      ub.setConstant(10);

      // Vector zeroVec(problem_dimension);
      // zeroVec.setZero();

      // constraint matrix A
      ConstraintBuilder cb(problem_dimension, curve_count);

      // initial point constraints

        // position (with added noise)
      if(max_initial_point_degree >= 0) {
        Vector value(problem_dimension);
        value << positions[i][0] + distribution(generator), positions[i][1] + distribution(generator);
        cb.addConstraintBeginning(0, 0, value); // Position
      }

        // higher order constraints
      for (size_t d = 1; d <= max_initial_point_degree; ++d) {
        vectoreuc val = trajectories[i].neval(dt, d);
        Vector value(problem_dimension);
        value << val[0], val[1];
        cb.addConstraintBeginning(0, d, value);
      }

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

      const size_t numConstraints = cb.A().rows();

#if QP_SOLVER == QP_SOLVER_OSQP
      ////
      // Problem settings
      OSQPSettings settings;
      osqp_set_default_settings(&settings);
      // settings.polish = 1;
      settings.eps_abs = 1.0e-4;
      settings.eps_rel = 1.0e-4;
      settings.max_iter = 20000;
      settings.verbose = false;

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
        sstr << "Couldn't solve QP!";
        throw std::runtime_error(sstr.str());
        // std::cerr << "Couldn't solve QP!" << std::endl;
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
      options.printLevel = qpOASES::PL_LOW; // only show errors
      qp.setOptions(options);

      // The  integer  argument nWSR specifies  the  maximum  number  of  working  set
      // recalculations to be performed during the initial homotopy (on output it contains the number
      // of  working  set  recalculations  actually  performed!)
      qpOASES::int_t nWSR = 10000;

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
        sstr << "Couldn't solve QP!";
        throw std::runtime_error(sstr.str());
        // std::cerr << "Couldn't solve QP!" << std::endl;
      }

      qp.getPrimalSolution(y.data());

      std::cout << "status: " << status << std::endl;
      std::cout << "objective: " << qp.getObjVal() << std::endl;
      // std::cout << "y: " << y << std::endl;
#endif
      // Update trajectories with our solution
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            trajectories[i][j][k][p] = y(p, j * ppc + k);
          }
        }
      }

      // temporal scaling

      // double desired_time_per_curve = hor / curve_count;
      double remaining_time = std::max(total_times[i] - ct, 3 * dt * 1.5);

      for (size_t j = 0; j < curve_count; ++j) {
        trajectories[i][j].duration = /*std::max(*/std::min(hor, remaining_time) / curve_count;//, 1.5 * dt);
      }
      // For robot-robot safety make sure that the first piece (with voronoi constraints) lasts until the next planning cycle
      // trajectories[i][0].duration = std::max(trajectories[i][0].duration, 1.5 * dt);

      // scale until slow enough
      while (true)
      {
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
        if (   max_velocity > 4.0
            || max_acceleration > 8.0) {
          for (size_t j = 0; j < curve_count; ++j) {
            trajectories[i][j].duration *= 2;
          }
        } else {
          break;
        }
      }

      for (size_t j = 0; j < curve_count; ++j) {
        std::cout << "traj " << j << " " << trajectories[i][j].duration << std::endl;
      }





#else
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

      //problem.add_inequality_constraint(optimization::edt_constraint, (void*)&edata, 0.0000001);


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
      cout << integral_stopval << endl;
      problem.set_stopval(integral_stopval);

      /* if objective function changes relatively less than this value, stop.*/
      problem.set_ftol_rel(relative_integral_stopval);

      if(set_max_time) {
        problem.set_maxtime(dt);
      }


      vector<double> initial_values;
      //trajectories[i][0][0] = positions[i];
      for(int j=0; j<curve_count; j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            initial_values.push_back(trajectories[i][j][j][p] /*+ fRand(-0.004, 0.004)*/ + pos_diffs[i][p]);
          }
        }
      }



      int initidx = 0;

#if USE_IPOPT

      SmartPtr<IpOptProblem> mynlp = new IpOptProblem(
      varcount,
      lower_bounds,
      upper_bounds,
      vdpts,
      contpts,
      continuity_tols,
      pointpts,
      initial_point_tols,
      initial_values,
      &data,
      &alt_data);

      // Create a new instance of IpoptApplication
      //  (use a SmartPtr, not raw)
      // We are using the factory, since this allows us to compile this
      // example with an Ipopt Windows DLL
      SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
      app->RethrowNonIpoptException(true);

      // Change some options
      // Note: The following choices are only examples, they might not be
      //       suitable for your optimization problem.
      app->Options()->SetStringValue("hessian_approximation", "limited-memory");
      app->Options()->SetNumericValue("tol", 1e-5);
      // app->Options()->SetStringValue("mu_strategy", "adaptive");
      // app->Options()->SetStringValue("output_file", "ipopt.out");

      app->Options()->SetStringValue("derivative_test", "first-order");

      app->Options()->SetIntegerValue("max_iter", 5000);
      // app->Options()->SetStringValue("jacobian_approximation", "finite-difference-values");


      // The following overwrites the default name (ipopt.opt) of the
      // options file
      // app->Options()->SetStringValue("option_file_name", "hs071.opt");

      // Initialize the IpoptApplication and process the options
      ApplicationReturnStatus status;
      status = app->Initialize();
      if (status != Solve_Succeeded) {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        return (int) status;
      }

      // Ask Ipopt to solve the problem
      status = app->OptimizeTNLP(mynlp);

      if (status == Solve_Succeeded) {
        std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
      }
      else {
        std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
        //throw std::runtime_error("IpOpt failed!");
      }

      initial_values = mynlp->m_finalValues;

#else

      double opt_f;
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

#endif

      initidx = 0;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            trajectories[i][j][k][p] = initial_values[initidx++];
          }
        }
      }




      // Sanity check of constraints:
      vector<double> dummyGradient(initial_values.size());

      for (auto& vd : vdpts) {
        double val = optimization::voronoi_constraint(initial_values, dummyGradient, vd);
        if (val > 0) {
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
#endif
      auto t1 = Time::now();
      fsec fs = t1 - t0;
      ms d = chrono::duration_cast<ms>(fs);
      total_time_for_opt += d.count();
      total_count_for_opt++;
      cout << "optimization time: " << d.count() << "ms" << endl;


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

    }
    //cout << "------" << endl;
    vectoreuc vec;
    for(int v = 0; v < steps_per_cycle; v++) {
      for(int i=0; i<trajectories.size(); i++) {
        vec = trajectories[i].neval(v*printdt, 0);
        //cout << vec << endl;
        output_json["points"][output_iter].push_back(vec.crds);
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

  }



  cout << "average opt time: " << total_time_for_opt / total_count_for_opt << "ms" << endl;
  out << output_json << endl;

  out.close();
  return 0;

}
