#include "optimization.h"
#include <vector>
#include <iostream>
#include <limits>
#include "bezier_mathematica.h"
using namespace std;



double optimization::alt_objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  double alpha_pos = 1;
  double alpha_vel = 0;
  double alpha_acc = 0;

  alt_obj_data& odata = *((alt_obj_data*)f_data);
  problem_data& pdata = *(odata.pdata);
  vectoreuc& pos = *(odata.pos);
  vectoreuc& vel = *(odata.vel);
  vectoreuc& acc = *(odata.acc);
  trajectory& ot = *(pdata.original_trajectory);

  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;
  int cpts = pd*ppc;

  double T = min(pdata.tt, pdata.time_horizon + pdata.current_t);

  int curveidx;
  for(curveidx = 0; curveidx<ot.size()-1 && T > ot[curveidx].duration; T-=ot[curveidx++].duration);

  T = min(T, ot[curveidx].duration);

  vector<double> P(cpts);

  for(int i=0; i<ppc; i++) {
    for(int j=0; j<pd; j++) {
      P[i*pd + j] = x[cpts*curveidx+i*pd+j];
    }
  }

  vector<double> innergrad;
  if(grad.size()>0) {
    for(int i=0; i<grad.size(); i++)
      grad[i] = 0;
    innergrad.resize(cpts);
  }


  //cout <<"T: " <<  T << endl;
  double posdis = bezier_2d_8pts_ndistance_from_point(P, pos, innergrad, T, 0, ot[curveidx].duration);
  if(grad.size() > 0) {
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[cpts*curveidx+i*pd + j] += innergrad[i*pd+j] * alpha_pos;
      }
    }
  }

  double veldis = bezier_2d_8pts_ndistance_from_point(P, vel, innergrad, T, 1, ot[curveidx].duration);
  if(grad.size() > 0) {
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[cpts*curveidx+i*pd + j] += innergrad[i*pd+j] * alpha_vel;
      }
    }
  }

  double accdis = bezier_2d_8pts_ndistance_from_point(P, acc, innergrad, T, 2, ot[curveidx].duration);
  if(grad.size() > 0) {
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[cpts*curveidx+i*pd + j] += innergrad[i*pd+j] * alpha_acc;
      }
    }
  }

  double cost = alpha_pos * posdis + alpha_vel * veldis + alpha_acc * accdis;
  //cout << "obj cost: " << cost << endl;
  return cost;

}


double optimization::objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  problem_data& pdata = *((problem_data*)f_data);

  double ct = pdata.current_t;
  double T = pdata.time_horizon;
  trajectory& ot = *(pdata.original_trajectory);
  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;
  double tt = pdata.tt;

  /*
    get the difference of original trajectory and current trajectory.
    newtraj.integrate(ct, ct+dt) is the result.
  */

  trajectory nt;

  int xcnt = 0;

  for(int i=0; i<ot.size(); i++) {
    curve nc(ot[i].duration, pd);
    for(int j=0; j<ppc; j++, xcnt+=pd) {
      vectoreuc nv(pd);
      for(int k=0; k<pd; k++) {
        nv[k] = x[xcnt+k];
      }
      nc.add_cpt(nv);
    }
    nt.add_curve(nc);
  }

  nt -= ot;
  double result = nt.integrate(ct, ct+T, grad);
  //cout << ct << ct+T << " objective: " << result << endl;
  return result;
}

double optimization::voronoi_constraint(const vector<double>& x, vector<double>& grad, void* v_data) {
  voronoi_data& vd = *((voronoi_data*)v_data);
  problem_data& pdata = *(vd.pdata);

  double ct = pdata.current_t;
  double T = pdata.time_horizon;
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
  //cout << "voro constraint: " << result << endl;
  return result;
}

double optimization::obstacle_constraint(const vector<double>& x, vector<double>& grad, void* o_data) {
  /*
    if a point (dot) obstacle hyperplane - b < 0 than it is in free space
  */
  obstacle_data& od = *((obstacle_data*)o_data);
  problem_data& pdata = *(od.pdata);

  vector<hyperplane>& hps = *(od.hps);
  //cout << hps.size() << endl;
  trajectory& ot = *(pdata.original_trajectory);
  double ct = pdata.current_t;
  double T = pdata.time_horizon;
  double tt = pdata.tt;
  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;

  if(grad.size() >  0) {
    for(int i=0; i<grad.size(); i++)
      grad[i] = 0;
  }

  trajectory nt;

  int xcnt = 0;

  for(int i=0; i<ot.size(); i++) {
    curve nc(ot[i].duration, pd);
    for(int j=0; j<ppc; j++, xcnt+=pd) {
      vectoreuc nv(pd);
      for(int k=0; k<pd; k++) {
        nv[k] = x[xcnt+k];
      }
      nc.add_cpt(nv);
    }
    nt.add_curve(nc);
  }

  double constraint_result = 0;
  double prev_constraint_result = 0;

  double obsdt = 0.01;
  for(double t = ct; t<=min(ct+T, tt); t+=obsdt) {
    //double velocity = nt.neval(t, 1).L2norm();
    int curveidx;
    double curvet;
    vectoreuc cur = nt.eval(t, curveidx, curvet);
    bool avoided = false;
    double closest_dist = numeric_limits<double>::infinity();
    int closest_hp = -1;
    for(int i=0; i<hps.size(); i++) {
      double dist = cur.dot(hps[i].normal) - hps[i].distance;
      if(dist < 0) {
        avoided = true;
        break;
      }
      if(dist < closest_dist) {
        closest_dist = dist;
        closest_hp = i;
      }
    }

    if(!avoided) {
      /* current point is on curve curveidx */

      // calculate gradient and distance using bezier_2d_8pts_distance_from_plane
      vector<double> innergrad;
      if(grad.size() > 0) {
        innergrad.resize(ppc * pd);
      }
      double dist = bezier_2d_8pts_distance_from_plane(nt[curveidx].cpts, hps[closest_hp], innergrad, curvet, nt[curveidx].duration);
      // sum the gradient to the input grad, and distance to the result of constraint
      constraint_result += dist * obsdt;
      prev_constraint_result += dist*obsdt;
      if(innergrad.size() > 0) {
        for(int i=0; i<ppc; i++) {
          for(int j=0; j<pd; j++) {
            grad[curveidx * ppc * pd + i*pd + j] += innergrad[i*pd + j] * obsdt;
          }
        }
      }

    }
  }
  //cout << "obstacle cons: " << constraint_result << endl;
  return constraint_result;
}

double optimization::continuity_constraint(const vector<double>& x, vector<double>& grad, void* c_data) {
  continuity_data& cd = *((continuity_data*)c_data);
  problem_data& pdata = *(cd.pdata);

  int continuity_degree = cd.n;
  int c = cd.c;
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;
  trajectory& ot = *(pdata.original_trajectory);
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

  double result = bezier_2d_8pts_continuity_euc(P, Q, continuity_degree, innergrad, ot[c].duration, ot[c+1].duration);

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
  //cout << "con constraint: " << result << endl;
  return result;
}

double optimization::point_constraint(const vector<double>& x, vector<double>& grad, void* p_data) {
  point_data& poidata = *((point_data*)p_data);
  problem_data& pdata = *(poidata.pdata);
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;
  int cpts = ppc * pd;

  trajectory& ot = *(pdata.original_trajectory);

  vectoreuc& point = poidata.point;
  double T = poidata.time;
  int degree = poidata.degree;

  int i = 0;
  while(i<ot.curves.size() && ot.curves[i].duration < T) {
    T -= ot.curves[i].duration;
    i++;
  }
  if(i == ot.curves.size()) {
    i--;
    T = ot.curves[i].duration;
  }


  vector<double> ccurve(cpts);

  for(int j=0; j<ppc; j++) {
    for(int k=0; k<pd; k++) {
      ccurve[j*pd+k] = x[i*cpts+j*pd+k];
    }
  }

  vector<double> innergrad;

  if(grad.size() > 0) {
    innergrad.resize(cpts);
  }

  double result = bezier_2d_8pts_ndistance_from_point(ccurve, point, innergrad, T, degree, ot.curves[i].duration);

  if(grad.size() > 0) {
    for(int j=0; j<grad.size(); j++) {
      grad[j] = 0;
    }

    for(int j=0; j<ppc; j++) {
      for(int k=0; k<pd; k++) {
        grad[i*cpts+j*pd+k] = innergrad[j*pd+k];
      }
    }
  }
  //cout << "point constraint deg "<< degree << ": " << result << endl;
  return result;

}

double optimization::maximum_nvalue_of_curve(const vector<double>& x, vector<double>& grad, void* d_data) {
  maxnvalue_data& maxdata = *((maxnvalue_data*)d_data);
  problem_data& pdata = *(maxdata.pdata);
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;
  int cpts = ppc * pd;
  trajectory& ot = *(pdata.original_trajectory);


  int cidx = maxdata.cidx;
  int deg = maxdata.degree;
  double max_allowed = maxdata.max_val;


  double maxdt = 0.01;

  vector<double> curvecpts(cpts);
  for(int i=0; i<ppc; i++) {
    for(int j=0; j<pd; j++) {
      curvecpts[i*pd+j] = x[cidx*cpts+i*pd+j];
    }
  }

  vector<double> innergrad;
  if(grad.size() > 0) {
    innergrad.resize(cpts);
    for(int i=0; i<grad.size(); i++) {
      grad[i] = 0;
    }
  }
  double max_val = -1;
  double dur = ot[cidx].duration;
  for(double t=0; t<=dur; t+=maxdt) {
    double val = bezier_2d_8pts_neval_l2normsq(curvecpts, innergrad, t, deg, dur);
    if(val > max_val) {
      max_val = val;
      if(innergrad.size() > 0) {
        for(int i=0; i<ppc; i++) {
          for(int j=0; j<pd; j++) {
            grad[cidx*cpts+i*pd+j] = innergrad[i*pd+j];
          }
        }
      }
    }
  }
  /*
  if(max_val - max_allowed > 0) {
    cout << " >>>> ";
  }*/
  //cout << "dyn consraint deg " << deg << ": " << max_val - max_allowed << endl;
  return max_val - max_allowed;
}
