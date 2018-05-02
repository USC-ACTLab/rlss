#include "ipopt_optimize.h"
#include <iostream>
#include <limits>

using namespace std;

IpOptProblem::IpOptProblem(
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
    , m_alt_problem_data(altpdata) {

}


bool IpOptProblem::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag, IndexStyleEnum& index_style) {
  n = m_varcount;
  m = m_contpts.size() + m_vdpts.size() + m_pointpts.size();
  nnz_jac_g = n*m;
  nnz_h_lag = n*n;
  index_style = TNLP::C_STYLE;
  return true;
}

bool IpOptProblem::get_bounds_info(Index n, Number* x_l, Number* x_u, Index m, Number* g_l, Number* g_u) {
  for(int i=0; i<n; i++) {
    x_l[i] = m_lower_bounds[i];
    x_u[i] = m_upper_bounds[i];
  }

  int idx = 0;
  for(int i=0; i<m_vdpts.size(); i++) {
    g_l[idx] = std::numeric_limits<Number>::lowest();
    g_u[idx] = 0;
    idx++;
  }

  for(int i=0; i<m_contpts.size(); i++) {
    g_l[idx] = std::numeric_limits<Number>::lowest();
    g_u[idx] = m_continuity_tols[m_contpts[i]->n];
    idx++;
  }

  for(int i=0; i<m_pointpts.size(); i++) {
    g_l[idx] = std::numeric_limits<Number>::lowest();
    g_u[idx] = m_initial_point_tols[m_pointpts[i]->degree];
    idx++;
  }

  return true;
}

bool IpOptProblem::get_starting_point(Index n, bool init_x, Number* x,
                                bool init_z, Number* z_L, Number* z_U,
                                Index m, bool init_lambda, Number* lambda)
{
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  for(int i=0; i<n; i++) {
    x[i] = m_initial_values[i];
  }
  return true;
}

bool IpOptProblem::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
  std::vector<double> vecX(x, x+n);
  std::vector<double> grad;
  obj_value = optimization::pos_energy_combine_objective(vecX, grad, (void*)m_alt_problem_data);
  return true;
}

bool IpOptProblem::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
  std::vector<double> vecX(x, x+n);
  std::vector<double> grad(n);

  optimization::pos_energy_combine_objective(vecX, grad, (void*)m_alt_problem_data);

  for(int i=0; i<n; i++) {
    grad_f[i] = grad[i];
  }

  return true;
}

bool IpOptProblem::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
  std::vector<double> vecX(x, x+n);
  std::vector<double> grad;

  int idx = 0;

  for(int i=0; i<m_vdpts.size(); i++) {
    g[idx] = optimization::voronoi_constraint(vecX, grad, (void*)m_vdpts[i]);
    idx++;
  }

  for(int i=0; i<m_contpts.size(); i++) {
    g[idx] = optimization::continuity_constraint(vecX, grad, (void*)m_contpts[i]);
    idx++;
  }

  for(int i=0; i<m_pointpts.size(); i++) {
    g[idx] = optimization::point_constraint(vecX, grad, (void*)m_pointpts[i]);
    idx++;
  }


  return true;
}

bool IpOptProblem::eval_jac_g(Index n, const Number* x, bool new_x,
                        Index m, Index nele_jac, Index* iRow, Index *jCol,
                        Number* values)
{
  if(values == NULL) {
    for(int i=0; i<m*n; i++) {
      iRow[i] = i/n;
      jCol[i] = i%n;
    }
  } else {
    std::vector<double> vecX(x, x+n);
    std::vector<double> grad(n);

    int idx = 0;

    for(int i=0; i<m_vdpts.size(); i++) {
      optimization::voronoi_constraint(vecX, grad, (void*)m_vdpts[i]);
      for(int j=0; j<n; j++) {
        values[idx*n+j] = grad[j];
      }
      idx++;
    }

    for(int i=0; i<m_contpts.size(); i++) {
      optimization::continuity_constraint(vecX, grad, (void*)m_contpts[i]);
      for(int j=0; j<n; j++) {
        values[idx*n+j] = grad[j];
      }
      idx++;
    }

    for(int i=0; i<m_pointpts.size(); i++) {
      optimization::point_constraint(vecX, grad, (void*)m_pointpts[i]);
      for(int j=0; j<n; j++) {
        values[idx*n+j] = grad[j];
      }
      idx++;
    }
  }

  return true;
}


bool IpOptProblem::eval_h(Index n, const Number* x, bool new_x,
            Number obj_factor, Index m, const Number* lambda,
            bool new_lambda, Index nele_hess, Index* iRow,
            Index* jCol, Number* values)
{
  return false;
}

void IpOptProblem::finalize_solution(SolverReturn status,
                               Index n, const Number* x, const Number* z_L, const Number* z_U,
                               Index m, const Number* g, const Number* lambda,
                               Number obj_value,
       const IpoptData* ip_data,
       IpoptCalculatedQuantities* ip_cq)
{
  cout << std::endl << std::endl << "Objective value" << endl;
  cout << "f(x*) = " << obj_value << std::endl;

  cout << std::endl << "Final value of the constraints:" << endl;
  for (Index i=0; i<m ;i++) {
    cout << "g(" << i << ") = " << g[i] << endl;
  }

  m_finalValues.resize(n);
  for (Index i=0; i<n; i++) {
    m_finalValues[i] = x[i];
  }
}
