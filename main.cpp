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

#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>


#define PATHREPLAN_EPSILON 0.00001

namespace fs = std::experimental::filesystem::v1;
using namespace std;


typedef chrono::high_resolution_clock Time;
typedef chrono::milliseconds ms;
typedef chrono::duration<float> fsec;

using namespace Ipopt;
class IpOptProblem : public Ipopt::TNLP
{


public:
  /** default constructor */
  IpOptProblem(
    int varcount,
    const vector<double>& lower_bounds,
    const vector<double>& upper_bounds,
    const vector<voronoi_data*>& vdpts,
    const vector<continuity_data*>& contpts,
    const vector<double>& continuity_tols,
    const vector<point_data*>& pointpts,
    const vector<double>& initial_point_tols,
    const vector<double>& initial_values,
    problem_data* pdata,
    alt_obj_data* altpdata)
    : m_varcount(varcount)
    , m_lower_bounds(lower_bounds)
    , m_upper_bounds(upper_bounds)
    , m_vdpts(vdpts)
    , m_contpts(contpts)
    , m_continuity_tols(continuity_tols)
    , m_pointpts(pointpts)
    , m_initial_point_tols(initial_point_tols)
    , m_initial_values(initial_values)
    , m_problem_data(pdata)
    , m_alt_problem_data(altpdata)
  {

  }

  /** default destructor */
  virtual ~IpOptProblem()
  {
  }

  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style)
  {
    // number of variables
    n = m_varcount;

    // number of constraints
    m = m_vdpts.size() + m_contpts.size() + m_pointpts.size();

    // in this example the jacobian is dense
    nnz_jac_g = m  * n;

    // the hessian is also dense
    nnz_h_lag = n * n;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
  }


  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u)
  {
    // set lower and upper bounds of variables
    for (int i = 0; i < m_varcount; ++i) {
      x_l[i] = m_lower_bounds[i];
      x_u[i] = m_upper_bounds[i];
    }

    // set bounds for voronoi constraints
    int constraintIdx = 0;
    for (int i = 0; i < m_vdpts.size(); ++i) {
      g_l[constraintIdx] = std::numeric_limits<Number>::lowest();
      g_u[constraintIdx] = 0;
      ++constraintIdx;
    }

    // set bounds for continuity constraints
    for (int i = 0; i < m_contpts.size(); ++i) {
      g_l[constraintIdx] = std::numeric_limits<Number>::lowest();
      g_u[constraintIdx] = m_continuity_tols[m_contpts[i]->n];
      ++constraintIdx;
    }

    // set bounds for point constraints
    for (int i = 0; i < m_pointpts.size(); ++i) {
      g_l[constraintIdx] = std::numeric_limits<Number>::lowest();
      g_u[constraintIdx] = m_initial_point_tols[m_pointpts[i]->degree];
      ++constraintIdx;
    }

    return true;
  }

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda)
  {
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    // initialize to the given starting point
    for (int i = 0; i < m_varcount; ++i) {
      x[i] = m_initial_values[i];
    }

    return true;
  }

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
  {
    std::vector<double> vecX(x, x + n);
    std::vector<double> grad;
    // obj_value = optimization::alt_objective(vecX, grad, m_alt_problem_data);
    obj_value = optimization::objective(vecX, grad, m_problem_data);

    return true;
  }

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
  {
    std::vector<double> vecX(x, x + n);
    std::vector<double> grad(n);
    // optimization::alt_objective(vecX, grad, m_alt_problem_data);
    optimization::objective(vecX, grad, m_problem_data);

    for (int i = 0; i < n; ++i) {
      grad_f[i] = grad[i];
    }

    return true;
  }

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
  {
    std::vector<double> vecX(x, x + n);
    std::vector<double> grad;

    int constraintIdx = 0;

    // voronoi constraints
    for (int i = 0; i < m_vdpts.size(); ++i) {
      g[constraintIdx] = optimization::voronoi_constraint(vecX, grad, m_vdpts[i]);
      ++constraintIdx;
    }

    // continuity constraints
    for (int i = 0; i < m_contpts.size(); ++i) {
      g[constraintIdx] = optimization::continuity_constraint(vecX, grad, m_contpts[i]);
      ++constraintIdx;
    }

    // point constraints
    for (int i = 0; i < m_pointpts.size(); ++i) {
      g[constraintIdx] = optimization::point_constraint(vecX, grad, m_pointpts[i]);
      // std::cout << "g[constraintIdx]" << g[constraintIdx] << " " << constraintIdx << std::endl;
      ++constraintIdx;
    }

    return true;
  }

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values)
  {
    if (values == NULL) {
      // return the structure of the jacobian
      // row i, column j: derivative of constraint g^i with respect to variable x^j

      // this particular jacobian is dense
      for (int i = 0; i < m * n; ++i) {
        iRow[i] = i / n;
        jCol[i] = i % n;
      }
    }
    else {
      // return the values of the jacobian of the constraints

      std::vector<double> vecX(x, x + n);
      std::vector<double> grad(n);

      int constraintIdx = 0;

      // voronoi constraints
      for (int i = 0; i < m_vdpts.size(); ++i) {
        optimization::voronoi_constraint(vecX, grad, m_vdpts[i]);
        for (int j = 0; j < n; ++j) {
          values[constraintIdx * n + j] = grad[j];
        }
        ++constraintIdx;
      }

      // continuity constraints
      for (int i = 0; i < m_contpts.size(); ++i) {
        optimization::continuity_constraint(vecX, grad, m_contpts[i]);
        for (int j = 0; j < n; ++j) {
          values[constraintIdx * n + j] = grad[j];
        }
        ++constraintIdx;
      }

      // point constraints
      for (int i = 0; i < m_pointpts.size(); ++i) {
        optimization::point_constraint(vecX, grad, m_pointpts[i]);
        for (int j = 0; j < n; ++j) {
          values[constraintIdx * n + j] = grad[j];
        }
        ++constraintIdx;
      }
    }

    return true;
  }

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values)
  {
    // We rely on quasi-newton approximation here, see https://www.coin-or.org/Ipopt/documentation/node31.html
    return false;
  }

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
         const IpoptData* ip_data,
         IpoptCalculatedQuantities* ip_cq)
  {
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.

    // For this example, we write the solution to the console
    // std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
    // for (Index i=0; i<n; i++) {
    //    std::cout << "x[" << i << "] = " << x[i] << std::endl;
    // }

    // std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
    // for (Index i=0; i<n; i++) {
    //   std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
    // }
    // for (Index i=0; i<n; i++) {
    //   std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
    // }

    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;

    std::cout << std::endl << "Final value of the constraints:" << std::endl;
    for (Index i=0; i<m ;i++) {
      std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }

    m_finalValues.resize(n);
    for (Index i=0; i<n; i++) {
      m_finalValues[i] = x[i];
    }
  }
  //@}

public:
  vector<double> m_finalValues;

private:
  int m_varcount;
  vector<double> m_lower_bounds;
  vector<double> m_upper_bounds;
  vector<voronoi_data*> m_vdpts;
  vector<continuity_data*> m_contpts;
  vector<double> m_continuity_tols;
  vector<point_data*> m_pointpts;
  vector<double> m_initial_point_tols;

  vector<double> m_initial_values;

  problem_data* m_problem_data;
  alt_obj_data* m_alt_problem_data;
};


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


  vector<trajectory> original_trajectories = trajectories;

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



  vector<double> total_times(original_trajectories.size());
  double total_t = 0;
  for(int i=0; i<original_trajectories.size(); i++) {
    double tt = 0;
    for(int j=0; j<original_trajectories[i].size(); j++) {
      tt += original_trajectories[i][j].duration;
    }
    total_t = max(total_t, tt);
    total_times[i] = tt;
    cout << total_times[i] << endl;
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
      vectoreuc eu = original_trajectories[i].eval(t);
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
    cout << "ct: " << ct << endl;

    for(int i=0; i<trajectories.size(); i++ ) {
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

      /*
        number of curves \times number of points per curve \times problem_dimension
      */

      if(ct <= total_times[i]) {

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
        data.original_trajectory = &(original_trajectories[i]);
        //data.original_trajectory = &(trajectories[i]);
        data.problem_dimension = problem_dimension;
        data.ppc = ppc;
        data.tt = total_times[i];

        //problem.set_min_objective(optimization::objective, (void*)&data);

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
          double end_time  = ct+2*dt;
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

        /*for(int j=0; j<obstacles.size(); j++) {
          obstacle_data* od = new obstacle_data;
          od->pdata = &data;
          od->hps = &(obstacles[j].chplanes);

          problem.add_inequality_constraint(optimization::obstacle_constraint, (void*)od, obstacle_tolerance);
          obspts.push_back(od);
        }*/


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
          pd->point = trajectories[i].neval(min(ct, total_times[i]), j);
          /*if(j==0) {
            pd->point[0] += fRand(-0.002, 0.002);
            pd->point[1] += fRand(-0.002, 0.002);
          }*/
          pd->time = min(ct, total_times[i]);
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
              initial_values.push_back(trajectories[i][j][k][p] + fRand(-0.004, 0.004));
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

  #if 0

    // Create a new instance of your nlp
    //  (use a SmartPtr, not raw)
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
          if (val > 1e-3) {
            std::stringstream sstr;
            sstr << "Voronoi constraint violated: " << val;
            std::cout << sstr.str() << std::endl;
            // throw std::runtime_error(sstr.str());
          }
        }

        for (auto& od : obspts) {
          double val = optimization::obstacle_constraint(initial_values, dummyGradient, od);
          if (val > obstacle_tolerance) {
            std::stringstream sstr;
            sstr << "obstacle contraint violated: " << val;
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

        for (auto& dd : maxpts) {
          double val = optimization::maximum_nvalue_of_curve(initial_values, dummyGradient, dd);
          if (val > 0) {
            std::stringstream sstr;
            sstr << "maximum_nvalue_of_curve constraint violated: " << val << " (max: " << 0 << ")";
            std::cout << sstr.str() << std::endl;
            // throw std::runtime_error(sstr.str());
          }
        }

        for (auto& pd : pointpts) {
          double val = optimization::point_constraint(initial_values, dummyGradient, pd);
          if (val > initial_point_tols[pd->degree]) {
            std::stringstream sstr;
            sstr << "point constraint violated: " << val << " (max: " << initial_point_tols[pd->degree] << ")";
            std::cout << sstr.str() << std::endl;
            // throw std::runtime_error(sstr.str());
          }
        }



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



        cout << "before deletes" << endl;

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
      }
      
      for(double t = ct; t<min(total_t, ct+hor); t+=printdt) {
        vectoreuc ev = trajectories[i].eval(t);
        output_json["planned_trajs"][output_iter][i]["x"].push_back(ev[0]);
        output_json["planned_trajs"][output_iter][i]["y"].push_back(ev[1]);
      }

      cout << "traj " << i << " end" << endl;
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
