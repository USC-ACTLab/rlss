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

namespace fs = std::experimental::filesystem::v1;
using namespace std;
using namespace boost::geometry;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}


int main() {

  srand(time(NULL));

  int problem_dimension = 2;

  vector<trajectory> trajectories;

  string path = "../../initial_trajectories/";
  for (auto & p : fs::directory_iterator(path)) {
    csv::Parser file(p.path().string());
    trajectory trj;
    for(int i=0; i<file.rowCount(); i++) {
      double duration = stod(file[i][0]);
      curve crv(duration, problem_dimension);
      for(int j=1; j<=16; j+=problem_dimension) {
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

  path = "../../obstacles/";
  for(auto & p : fs::directory_iterator(path)) {
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


  double dt = total_t;
  vector<vectoreuc> positions(trajectories.size());

  for(double ct = 0; ct <= total_t; ct+=dt) {
    for(int i=0; i<trajectories.size(); i++) {
      positions[i] = trajectories[i].eval(ct);
    }

    for(int i=0; i<trajectories.size(); i++) {
      int a;

      /*calculate voronoi hyperplanes for robot i*/
      vector<hyperplane> voronoi_hyperplanes = voronoi(positions, i);


      /*
        number of curves \times number of points per curve \times problem_dimension
      */
      unsigned varcount = trajectories[i].size() * trajectories[i][0].size() * problem_dimension;
      nlopt::opt problem(nlopt::LD_MMA, varcount);

      problem_data data;
      data.current_t = ct;
      data.delta_t = dt;
      data.original_trajectory = &trajectories[i];
      data.problem_dimension = problem_dimension;

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
        voronoi_data* vd = new voronoi_data;
        vd->pdata = &data;
        vd->plane = plane;

        problem.add_inequality_constraint(optimization::voronoi_constraint, (void*)vd, 0);
        vdpts.push_back(vd);
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

      /* if integral is less than this value, stop.*/
      problem.set_stopval(0.00000001);

      /* if objective function changes relatively less than this value, stop.*/
      problem.set_ftol_rel(0.00000001);


      vector<double> initial_values;
      for(int j=0; j<trajectories[i].size(); j++) {
        for(int k=0; k<trajectories[i][j].size(); k++) {
          initial_values.push_back(trajectories[i][j][k][0]+fRand(0, 0.2));
          initial_values.push_back(trajectories[i][j][k][1]+fRand(0, 0.2));
        }
      }

      cout << initial_values.size() << endl;

      double opt_f;

      int idx = 0;

      for(int j=0; j<trajectories[i].size(); j++) {
        cout << trajectories[i][j].duration;
        for(int k=0; k<trajectories[i][j].size(); k++) {
          for(int p=0; p<trajectories[i][j][k].size(); p++) {
            cout << "," << initial_values[idx++];
          }
        }
        cout << endl;
      }
      cout << endl << endl;
      nlopt::result res = problem.optimize(initial_values, opt_f);
      idx = 0;

      cout << "nlopt result: " << res << " objective value: " << opt_f << endl;
      for(int j=0; j<trajectories[i].size(); j++) {
        cout << trajectories[i][j].duration;
        for(int k=0; k<trajectories[i][j].size(); k++) {
          for(int p=0; p<trajectories[i][j][k].size(); p++) {
            cout << "," << initial_values[idx++];
          }
        }
        cout << endl;
      }



      cin >> a;


      for(int j=0; j<vdpts.size(); j++) {
        delete vdpts[j];
      }
      for(int j=0; j<obspts.size(); j++) {
        delete obspts[j];
      }
    }

    return 0;

  }

}
