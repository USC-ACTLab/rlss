#include "optimization.h"
#include <vector>
#include <iostream>
#include <limits>
#include "bezier_mathematica.h"

using namespace std;

double optimization::pos_energy_edt_combine_objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  double alpha_poseng = 1;
  double alpha_edt = 100;

  alt_edt_combination_data& aedata = *((alt_edt_combination_data*) f_data);

  edt_collision_data* edata = aedata.edt;
  alt_obj_data* adata = aedata.alt;

  vector<double> innergrad(grad.size());

  double res = alpha_edt * optimization::edt_cost(x, innergrad, (void*)edata);

  for(int i=0; i<grad.size(); i++)
    grad[i] = innergrad[i] * alpha_edt;

  res += alpha_poseng * optimization::pos_energy_combine_objective(x, innergrad, (void*)adata);

  for(int i=0; i<grad.size(); i++)
    grad[i] += innergrad[i] * alpha_poseng;

  return res;
}

double optimization::edt_cost(const vector<double>& x, vector<double>& grad, void* f_data) {
  edt_collision_data& edata = *((edt_collision_data*) f_data);

  problem_data& pdata = *(edata.pdata);

  double ct = pdata.current_t;
  double hor = pdata.time_horizon;
  trajectory& curtraj = *(pdata.current_trajectory);
  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;
  double tt = pdata.tt;

  edtv2 distance_transform = *(edata.distance_transform);

  int cpts = pd * ppc;

  for(int i=0; i<grad.size(); i++)
    grad[i] = 0;



  double edtdt = 0.01;

  double cost = 0;

  for(double t = 0; t<=hor; t+=edtdt) {
    pair<int, double> cdata = curtraj.curvedata(t);

    int cidx = cdata.first;
    double T = cdata.second;

    vector<double> P(cpts);

    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        P[i*pd+j] = x[cidx*cpts+i*pd+j];
      }
    }

    vector<double> innergrad;
    if(grad.size() > 0)
      innergrad.resize(cpts);

    cost += distance_transform.cost(P, curtraj[cidx].duration, T, innergrad) * edtdt;

    if(innergrad.size() > 0) {
      for(int i=0; i<ppc; i++) {
        for(int j=0; j<pd; j++) {
          grad[cidx*cpts + i*pd + j] += innergrad[i*pd + j] * edtdt;
        }
      }
    }
  }

  return cost;
}

double optimization::energy_objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  //cout << "energy start" << endl;
  double alpha_vel = 0;
  double alpha_acc = 1;
  double alpha_jerk = 1e-4;


  problem_data& pdata = *((problem_data*)f_data);

  double ct = pdata.current_t;
  double T = pdata.time_horizon;
  trajectory& curtraj = *(pdata.current_trajectory);
  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;
  double tt = pdata.tt;

  int cpts = pd * ppc;

  if(grad.size() > 0) {
    for(int i=0; i<grad.size(); i++) {
      grad[i] = 0;
    }
  }


  double start_time = 0;
  double end_time = curtraj.total_duration;

  int startcurveidx = 0;
  int endcurveidx = curtraj.size() - 1;



  double total_energy = 0;
  vector<double> innergrad;
  if(grad.size() > 0) {
    innergrad.resize(cpts);
  }

  for(int i=startcurveidx; i<=endcurveidx; i++) {
    vector<double> P(cpts);
    for(int j=0; j<ppc; j++) {
      for(int k=0; k<pd; k++) {
        P[j*pd+k] = x[i*cpts + j*pd+k];
      }
    }
    double energy = bezier_2d_8pts_nthderivative_normsqintegrate(P, innergrad, curtraj[i].duration, 1);
    total_energy += alpha_vel * energy;

    if(grad.size()>  0) {
      for(int j=0; j<ppc; j++) {
        for(int k=0; k<pd; k++) {
          grad[i*cpts+j*pd+k] += innergrad[j*pd+k] * alpha_vel;
        }
      }
    }

    energy = bezier_2d_8pts_nthderivative_normsqintegrate(P, innergrad, curtraj[i].duration, 2);
    total_energy += alpha_acc * energy;

    if(grad.size()>  0) {
      for(int j=0; j<ppc; j++) {
        for(int k=0; k<pd; k++) {
          grad[i*cpts+j*pd+k] += innergrad[j*pd+k] * alpha_acc;
        }
      }
    }

    energy = bezier_2d_8pts_nthderivative_normsqintegrate(P, innergrad, curtraj[i].duration, 3);
    total_energy += alpha_jerk * energy;


    if(grad.size() > 0) {
      for(int j=0; j<ppc; j++) {
        for(int k=0; k<pd; k++) {
          grad[i*cpts+j*pd+k] += innergrad[j*pd+k] * alpha_jerk;
        }
      }
    }

  }

  //cout << "energy result: " << total_energy << endl;
  return total_energy;
}


double optimization::alt_objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  //cout << "alt obj start" << endl;
  double alpha_pos = 1;
  double alpha_vel = 0;
  double alpha_acc = 0;

  alt_obj_data& odata = *((alt_obj_data*)f_data);
  problem_data& pdata = *(odata.pdata);
  vectoreuc& pos = *(odata.pos);
  vectoreuc& vel = *(odata.vel);
  vectoreuc& acc = *(odata.acc);
  trajectory& curtraj = *(pdata.current_trajectory);

  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;
  int cpts = pd*ppc;

  double T = curtraj[curtraj.size() - 1].duration;
  int curveidx = curtraj.size() - 1;

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


  cout <<"T: " <<  T << endl;
  double posdis = bezier_2d_8pts_ndistance_from_point(P, pos, innergrad, T, 0, curtraj[curveidx].duration);
  if(grad.size() > 0) {
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[cpts*curveidx+i*pd + j] += innergrad[i*pd+j] * alpha_pos;
      }
    }
  }

  double veldis = bezier_2d_8pts_ndistance_from_point(P, vel, innergrad, T, 1, curtraj[curveidx].duration);
  if(grad.size() > 0) {
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[cpts*curveidx+i*pd + j] += innergrad[i*pd+j] * alpha_vel;
      }
    }
  }

  double accdis = bezier_2d_8pts_ndistance_from_point(P, acc, innergrad, T, 2, curtraj[curveidx].duration);
  if(grad.size() > 0) {
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        grad[cpts*curveidx+i*pd + j] += innergrad[i*pd+j] * alpha_acc;
      }
    }
  }

  double cost = alpha_pos * posdis + alpha_vel * veldis + alpha_acc * accdis;
  //cout << "alt obj cost: " << cost << endl;
  return cost;

}


double optimization::pos_energy_combine_objective(const vector<double>& x, vector<double>& grad, void* f_data) {
  //cout << "objective start" << endl;
  alt_obj_data& odata = *((alt_obj_data*)f_data);
  problem_data& pdata = *(odata.pdata);
  vectoreuc& pos = *(odata.pos);
  vectoreuc& vel = *(odata.vel);
  vectoreuc& acc = *(odata.acc);
  trajectory& curtraj = *(pdata.current_trajectory);

  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;
  int cpts = pd*ppc;

  double alpha_pos = 10;
  double alpha_energy = 0;

  for(int i=0; i<grad.size(); i++) {
    grad[i] = 0;
  }

  vector<double> innergrad(grad.size());

  double res = alpha_pos * optimization::alt_objective(x, innergrad, f_data);

  if(grad.size() > 0) {
    for(int i=0; i<grad.size() ;i++) {
      grad[i] += alpha_pos * innergrad[i];
    }
  }

  res += alpha_energy * optimization::energy_objective(x, innergrad, &pdata);

  if(grad.size() > 0) {
    for(int i=0; i<grad.size(); i++) {
      grad[i] += alpha_energy * innergrad[i];
    }
  }

  //cout << "obj result: " << res << endl;

  return res;
}



double optimization::voronoi_constraint(const vector<double>& x, vector<double>& grad, void* v_data) {
  //cout << "voronoi start" << endl;
  voronoi_data& vd = *((voronoi_data*)v_data);
  problem_data& pdata = *(vd.pdata);

  double ct = pdata.current_t;
  double T = pdata.time_horizon;
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

double optimization::edt_constraint(const vector<double>& x, vector<double>& grad, void* e_data) {
  //cout << "edt start" << endl;
  edt_data& ed = *((edt_data*)e_data);
  edt& distance_transform = *(ed.distance_transform);
  problem_data& pdata = *(ed.pdata);

  trajectory& curtraj = *(pdata.current_trajectory);

  double ct = pdata.current_t;
  double hor = pdata.time_horizon;
  double total_time = pdata.tt;
  int pd = pdata.problem_dimension;
  int ppc = pdata.ppc;

  int cpts = pd*ppc;

  if(grad.size() > 0) {
    for(int i=0; i<grad.size(); i++)
      grad[i] = 0;
  }


  double obsdt = 0.005;

  double constraint_result = 0;

  for(double t = 0; t<=hor; t+=obsdt) {
    pair<int, double> cdata = curtraj.curvedata(t);
    int cidx = cdata.first;
    double T = cdata.second;

    vector<double> P;
    P.resize(cpts);
    for(int i=0; i<ppc; i++) {
      for(int j=0; j<pd; j++) {
        P[i*pd+j] = x[cpts*cidx+i*pd+j];
      }
    }

    vector<double> innergrad;
    if(grad.size() > 0) {
      innergrad.resize(cpts);
    }

    double result = distance_transform.interpolate_for(P, curtraj[cidx].duration, T, innergrad);

    constraint_result += result*obsdt;

    if(grad.size() > 0) {
      for(int i=0; i<ppc; i++) {
        for(int j=0; j<pd; j++) {
          grad[cidx*cpts+i*pd+j] += innergrad[i*pd+j] * obsdt;
        }
      }
    }

  }
  //cout << "edt results: " << constraint_result << endl;
  return constraint_result;

}

double optimization::continuity_constraint(const vector<double>& x, vector<double>& grad, void* c_data) {
  //cout << "continuity start" << endl;
  continuity_data& cd = *((continuity_data*)c_data);
  problem_data& pdata = *(cd.pdata);

  int continuity_degree = cd.n;
  int c = cd.c;
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;
  int cpts = ppc * pd;
  trajectory& curtraj = *(pdata.current_trajectory);

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

  double result = bezier_2d_8pts_continuity_euc(P, Q, continuity_degree, innergrad, curtraj[c].duration, curtraj[c+1].duration);

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
  //cout << "point start" << endl;
  point_data& poidata = *((point_data*)p_data);
  problem_data& pdata = *(poidata.pdata);
  int ppc = pdata.ppc; //pts per curve
  int pd = pdata.problem_dimension;
  int cpts = ppc * pd;

  vectoreuc& point = poidata.point;
  double T = poidata.time;
  int degree = poidata.degree;

  trajectory& curtraj = *(pdata.current_trajectory);



  int i = 0;
  while(i<curtraj.curves.size() && curtraj.curves[i].duration < T) {
    T -= curtraj.curves[i].duration;
    i++;
  }
  if(i == curtraj.curves.size()) {
    i--;
    T = curtraj.curves[i].duration;
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

  double result = bezier_2d_8pts_ndistance_from_point(ccurve, point, innergrad, T, degree, curtraj.curves[i].duration);

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
  /*
  if(degree == 0) {
    for(int i=0; i<innergrad.size(); i++) {
      cout << innergrad[i] << " ";
    }
    cout << endl;
  }
  */
  //cout << "point constraint deg "<< degree << ": " << result << endl;
  return result;

}
