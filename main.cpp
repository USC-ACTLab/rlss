#include <iostream>
#include <nlopt.hpp>
#include <armadillo>
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

#define PATHREPLAN_EPSILON 0.00001

namespace fs = std::experimental::filesystem::v1;
using namespace std;
using namespace boost::geometry;

double frand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


int main(int argc, char** argv) {

  string initial_trajectories_path = "../../initial_trajectories/";
  string obstacles_path = "../../obstacles/";
  double dt = 0.2;
  double integral_stopval = 0.0001;
  double relative_integral_stopval = 0.001;
  int problem_dimension = 2;
  int ppc = 8;
  int max_continuity = 3;
  double continuity_tol = 0.001;

  cxxopts::Options options("Path Replanner", "Path replanner for UAV swarms");
  options.add_options()
    ("trajectories", "Folder that contains trajectories", cxxopts::value<std::string>()->default_value("../../initial_trajectories/"))
    ("obstacles", "Folder that contains obstacles", cxxopts::value<std::string>()->default_value("../../obstacles/"))
    ("dt", "Delta time", cxxopts::value<double>()->default_value("0.2"))
    ("is", "Integral stop value", cxxopts::value<double>()->default_value("0.0001"))
    ("ris", "Relative integral stop ratio", cxxopts::value<double>()->default_value("0.001"))
    ("dimension", "Problem dimension", cxxopts::value<int>()->default_value("2"))
    ("ppc", "Points per curve", cxxopts::value<int>()->default_value("8"))
    ("cont", "Contiuity upto this degree", cxxopts::value<int>()->default_value("3"))
    ("tol", "Continuity tolerances for equality", cxxopts::value<double>()->default_value("0.001"))
    ("help", "Display help page");

  auto result = options.parse(argc, argv);

  if(result.count("help")>0) {
    cout << options.help() << endl;
    return 0;
  }


  initial_trajectories_path = result["trajectories"].as<string>();
  obstacles_path = result["obstacles"].as<string>();
  dt = result["dt"].as<double>();
  integral_stopval = result["is"].as<double>();
  relative_integral_stopval = result["ris"].as<double>();
  problem_dimension = result["dimension"].as<int>();
  max_continuity = result["cont"].as<int>();
  continuity_tol = result["tol"].as<double>();
  ppc = result["ppc"].as<int>();


  cout << "initial_trajectories_path: " << initial_trajectories_path << endl
       << "obstacles_path: " << obstacles_path << endl
       << "dt: " << dt << endl
       << "integral_stopval: " << integral_stopval << endl
       << "relative_integral_stopval: " << relative_integral_stopval << endl
       << "problem_dimension: " << problem_dimension << endl
       << "points per curve: " << ppc << endl
       << "continuity upto: " << max_continuity << endl
       << "continuity tolerance: " << continuity_tol << endl << endl;

  srand(time(NULL));


  vector<trajectory> trajectories;


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
    obstacles.push_back(o);
  }



  double total_t = 0;
  for(int i=0; i<trajectories[0].size(); i++) {
    total_t += trajectories[0][i].duration;
  }


  vector<vectoreuc> positions(trajectories.size());

  double printdt = 0.01;

  for(double ct = 0; ct <= total_t; ct+=dt) {
    for(int i=0; i<trajectories.size(); i++) {
      positions[i] = trajectories[i].eval(ct);
      cout << i << " (" << positions[i][0] << "," << positions[i][1] << ")" << endl;
    }

    for(int i=0; i<trajectories.size(); i++) {

      /*calculate voronoi hyperplanes for robot i*/
      vector<hyperplane> voronoi_hyperplanes = voronoi(positions, i);


      /*
        number of curves \times number of points per curve \times problem_dimension
      */

      unsigned varcount = trajectories[i].size() * ppc * problem_dimension;
      nlopt::opt problem(nlopt::LD_SLSQP, varcount);

      problem_data data;
      data.current_t = ct;
      data.delta_t = dt;
      data.original_trajectory = &(trajectories[i]);
      data.problem_dimension = problem_dimension;
      data.ppc = ppc;

      problem.set_min_objective(optimization::objective, (void*)&data);


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
            if((curvet > current_time || fabs(curvet - current_time) < PATHREPLAN_EPSILON) && (curvet < cend_time || fabs(curvet - cend_time) < PATHREPLAN_EPSILON)) {
              voronoi_data* vd = new voronoi_data;
              vd->pdata = &data;
              vd->plane = plane;
              vd->curve_idx = idx;
              vd->point_idx = p;

              problem.add_inequality_constraint(optimization::voronoi_constraint, (void*)vd, 0);
              vdpts.push_back(vd);
            }
            curvet += step;
          }


          end_time -= cend_time;
          current_time = 0;
          idx++;
        }

      }

      // just to delete them from heap later.
      vector<obstacle_data*> obspts;

      for(int j=0; j<obstacles.size(); j++) {
        obstacle_data* od = new obstacle_data;
        od->pdata = &data;
        od->obs = &obstacles[j];

        problem.add_inequality_constraint(optimization::obstacle_constraint, (void*)od, 0);
        obspts.push_back(od);
      }


      vector<continuity_data*> contpts;
      /*vector<double> continuity_tolerances(problem_dimension);
      for(int i=0; i<problem_dimension; i++) {
        continuity_tolerances[i] = continuity_tol;
      }*/
      for(int n=0; n<=max_continuity; n++) {
        // nth degree continuity
        for(int j=0; j<trajectories[i].size()-1; j++) {
          continuity_data* cd = new continuity_data;
          cd->pdata = &data;
          cd->n = n;
          cd->c = j;

          //problem.add_equality_mconstraint(optimization::continuity_mconstraint, (void*)cd, continuity_tolerances);
          problem.add_equality_constraint(optimization::continuity_constraint, (void*)cd, continuity_tol);
          contpts.push_back(cd);
        }
      }

      /* if integral is less than this value, stop.*/
      problem.set_stopval(integral_stopval);

      /* if objective function changes relatively less than this value, stop.*/
      problem.set_ftol_rel(relative_integral_stopval);

      //problem.set_maxtime(10);


      vector<double> initial_values;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            initial_values.push_back(trajectories[i][j][k][p]+frand(0, 0.2));
          }
        }
      }

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
        //cout << "roundoff_limited" << endl;
      } catch(std::runtime_error& e) {
        //cout << "runtime error " << i << endl;
      }

      initidx = 0;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<ppc; k++) {
          for(int p=0; p<problem_dimension; p++) {
            trajectories[i][j][k][p] = initial_values[initidx++];
          }
        }
      }


      /*initidx = 0;
      cout << "nlopt result: " << res << " objective value: " << opt_f << endl;
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
    }

    for(double t = ct + printdt; t<ct+dt; t+=printdt) {
      for(int i=0; i<trajectories.size(); i++) {
        vectoreuc vec = trajectories[i].eval(t);
        cout << i << " (" << vec[0] << "," << vec[1] << ")" << endl;
      }
    }
  }

  return 0;

}
