#ifndef PATHREPLAN_IPOPT_H
#define PATHREPLAN_IPOPT_H

#include <coin/IpTNLP.hpp>
#include <vector>
#include "optimization.h"

using namespace Ipopt;
using namespace std;

class IpOptProblem : public Ipopt::TNLP {
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
  public:
    vector<double> m_finalValues;
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
      alt_obj_data* altpdata);

    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                              Index& nnz_h_lag, IndexStyleEnum& index_style);
    bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                                 Index m, Number* g_l, Number* g_u);
    bool get_starting_point(Index n, bool init_x, Number* x,
                                    bool init_z, Number* z_L, Number* z_U,
                                    Index m, bool init_lambda,
                                    Number* lambda);

    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);
    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);
    bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);
    bool eval_jac_g(Index n, const Number* x, bool new_x,
                            Index m, Index nele_jac, Index* iRow, Index *jCol,
                            Number* values);
    bool eval_h(Index n, const Number* x, bool new_x,
                        Number obj_factor, Index m, const Number* lambda,
                        bool new_lambda, Index nele_hess, Index* iRow,
                        Index* jCol, Number* values);
    void finalize_solution(SolverReturn status,
                                   Index n, const Number* x, const Number* z_L, const Number* z_U,
                                   Index m, const Number* g, const Number* lambda,
                                   Number obj_value,
           const IpoptData* ip_data,
           IpoptCalculatedQuantities* ip_cq);

};

#endif
