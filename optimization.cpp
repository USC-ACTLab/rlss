#include "optimization.h"
#include <vector>
#include <iostream>
#include "bezier_mathematica.h"
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
  double result = nt.integrate(ct, ct+dt, grad);
  return result;
}

double optimization::voronoi_constraint(const vector<double>& x, vector<double>& grad, void* v_data) {
  voronoi_data& vd = *((voronoi_data*)v_data);
  problem_data& pdata = *(vd.pdata);

  double ct = pdata.current_t;
  double dt = pdata.delta_t;
  trajectory& ot = *(pdata.original_trajectory);
  int pd = pdata.problem_dimension;
  hyperplane& plane = vd.plane;
  int cidx = vd.curve_idx;
  int pidx = vd.point_idx;
  int ppc = pdata.ppc;

  int fpidx = cidx * ppc * pd + pidx * pd;

  vectoreuc cur_pt(pd);
  for(int i=0; i<pd; i++) {
    cur_pt[i] = x[fpidx+i];
  }


  if(grad.size() > 0) {
    for(int i=0; i< grad.size(); i++) {
      grad[i] = 0;
    }
    for(int i=0; i<pd; i++) {
      grad[fpidx+i] = plane.normal[i];
    }
  }
  double result = cur_pt.dot(plane.normal) - plane.distance;
  return result;
}

double optimization::obstacle_constraint(const vector<double>& x, vector<double>& grad, void* o_data) {
  obstacle_data& od = *((obstacle_data*)o_data);
  problem_data& pdata = *(od.pdata);

  obstacle2D& obs = *(od.obs);
  double ct = pdata.current_t;
  double dt = pdata.delta_t;
  int pd = pdata.problem_dimension;

  if(grad.size() >  0) {
    for(int i=0; i<grad.size(); i++)
      grad[i] = 0;
  }


  return -1;
}

void optimization::continuity_mconstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* c_data) {
  continuity_data& cd = *((continuity_data*)c_data);
  problem_data& pdata = *(cd.pdata);

  int continuity_degree = cd.n;
  int c = cd.c;
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;

  const double* P = x + ppc * pd * c;
  const double* Q = x + ppc* pd * (c+1);

  vector<vector<double> > gradvec;
  if(grad)
    gradvec.resize(pd);

  bezier_2d_8pts_continuity(P, Q, continuity_degree, gradvec, result);
  if(grad) {
    for(int i=0; i<pd; i++) {
      for(int j=0; j<n; j++) {
        grad[i*n + j] = 0;
      }
    }


    for(int i=0; i<pd; i++) {
      for(int j=0; j<ppc; j++) {
        for(int k=0; k<pd; k++) {
          grad[n*i + ppc*pd*c+j*pd+k] = gradvec[i][j*pd + k];
          grad[n*i + ppc*pd*(c+1)+j*pd+k] = gradvec[i][ppc*pd + j*pd+k];
        }
      }
    }
  }

}

double optimization::continuity_constraint(const vector<double>& x, vector<double>& grad, void* c_data) {
  continuity_data& cd = *((continuity_data*)c_data);
  problem_data& pdata = *(cd.pdata);

  int continuity_degree = cd.n;
  int c = cd.c;
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;
  int cpts = ppc * pd;

  vector<double> P(cpts);
  vector<double> Q(cpts);

  int poffs = cpts*c;
  int qoffs = cpts*(c+1);
  for(int i=0; i<cpts; i++) {
    P[i] = x[poffs+i];
    Q[i] = x[qoffs+i];
  }

  vector<double> innergrad;

  if(grad.size() > 0) {
    innergrad.resize(cpts*2);
  }

  double result = bezier_2d_8pts_continuity_euc(P, Q, continuity_degree, innergrad);

  if(grad.size() > 0) {
    for(int i=0; i<grad.size(); i++) {
      grad[i] = 0;
    }
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[poffs + i*pd + j] = innergrad[i*pd+j];
        grad[qoffs + i*pd + j] = innergrad[cpts + i*pd + j];
      }
    }
  }
  return result;
}
