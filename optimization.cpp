#include "optimization.h"
#include <vector>
using namespace std;


double optimization::objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  problem_data& pdata = *((problem_data*)f_data);

  double ct = pdata.current_t;
  double dt = pdata.delta_t;
  trajectory& ot = *(pdata.original_trajectory);
  int pd = pdata.problem_dimension;

  /*
    get the difference of original trajectory and current trajectory.
    newtraj.integrate(ct, ct+dt) is the result.
  */

  trajectory nt;

  int xcnt = 0;

  for(int i=0; i<ot.size(); i++) {
    curve nc(ot[i].duration, pd);
    for(int j=0; j<ot[i].size(); j++, xcnt+=2) {
      vectoreuc nv(2);
      nv[0] = x[xcnt];
      nv[1] = x[xcnt+1];
      nc.add_cpt(nv);
    }
    nt.add_curve(nc);
  }

  nt -= ot;

  return nt.integrate(ct, ct+dt, grad);
}

double optimization::voronoi_constraint(const vector<double>& x, vector<double>& grad, void* v_data) {
  voronoi_data& vd = *((voronoi_data*)v_data);
  problem_data& pdata = *(vd.pdata);

  double ct = pdata.current_t;
  double dt = pdata.delta_t;
  trajectory& ot = *(pdata.original_trajectory);
  int pd = pdata.problem_dimension;
  hyperplane& plane = vd.plane;

  return 0;
}

double optimization::obstacle_constraint(const vector<double>& x, vector<double>& grad, void* o_data) {
  obstacle_data& od = *((obstacle_data*)o_data);
  problem_data& pdata = *(od.pdata);

  obstacle2D& obs = *(od.obs);
  double ct = pdata.current_t;
  double dt = pdata.delta_t;
  int pd = pdata.problem_dimension;

  return 0;
}
