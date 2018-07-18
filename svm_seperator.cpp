#include "svm_seperator.h"
#include "libsvm/svm.h"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include "svmcvxwrapper_2pt_4obs.h"
#include "svmcvxwrapper_8pt_4obs.h"
#include "svmcvxwrapper_32pt_4obs.h"
#include "svmcvxwrapper_16pt_4obs.h"
#include "svmcvxwrapper_20pt_4obs.h"
#include <cassert>
#include <qpOASES.hpp>
#include <Eigen/Dense>
#include "osqp.h"



#define QPOASES_SVM 1
#define OSQP_SVM 0

#define SVM_SOLVER OSQP_SVM

using namespace std;

void print_null(const char* s) {}

SvmSeperator::SvmSeperator(vector<obstacle2D>* obs): obstacles(obs) {
  svm_set_print_string_function(&print_null);
}


void SvmSeperator::reset_pts() {
  pts.clear();
}

void SvmSeperator::add_pt(const vectoreuc& pt) {
  pts.push_back(pt);
}

vector<hyperplane> SvmSeperator::_2_4_seperate() {
  assert(pts.size() == 2);
  vector<hyperplane> result;
  vectoreuc& pt1 = pts[0];
  vectoreuc& pt2 = pts[1];
  for(int i=0; i<obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];
    assert(obs.pts.size() == 4);
    vectoreuc& obs1 = obs.pts[0];
    vectoreuc& obs2 = obs.pts[1];
    vectoreuc& obs3 = obs.pts[2];
    vectoreuc& obs4 = obs.pts[3];

    result.push_back(_2pt4obspt_svm(pt1, pt2, obs1, obs2, obs3, obs4));

    /*auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (obs.pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }*/

  }



  return result;
}

vector<hyperplane> SvmSeperator::_8_4_seperate() {
  assert(pts.size() == 8);
  vector<hyperplane> result;
  vectoreuc& pt1 = pts[0];
  vectoreuc& pt2 = pts[1];
  vectoreuc& pt3 = pts[2];
  vectoreuc& pt4 = pts[3];
  vectoreuc& pt5 = pts[4];
  vectoreuc& pt6 = pts[5];
  vectoreuc& pt7 = pts[6];
  vectoreuc& pt8 = pts[7];
  for(int i=0; i<obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];
    assert(obs.pts.size() == 4);
    vectoreuc& obs1 = obs.pts[0];
    vectoreuc& obs2 = obs.pts[1];
    vectoreuc& obs3 = obs.pts[2];
    vectoreuc& obs4 = obs.pts[3];

    result.push_back(_8pt4obspt_svm(pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8, obs1, obs2, obs3, obs4));

    /*auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (obs.pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }*/

  }

  return result;
}


vector<hyperplane> SvmSeperator::_20_4_seperate() {
  assert(pts.size() == 20);
  vector<hyperplane> result;
  vectoreuc& pt1 = pts[0];
  vectoreuc& pt2 = pts[1];
  vectoreuc& pt3 = pts[2];
  vectoreuc& pt4 = pts[3];
  vectoreuc& pt5 = pts[4];
  vectoreuc& pt6 = pts[5];
  vectoreuc& pt7 = pts[6];
  vectoreuc& pt8 = pts[7];
  vectoreuc& pt9 = pts[8];
  vectoreuc& pt10 = pts[9];
  vectoreuc& pt11 = pts[10];
  vectoreuc& pt12 = pts[11];
  vectoreuc& pt13 = pts[12];
  vectoreuc& pt14 = pts[13];
  vectoreuc& pt15 = pts[14];
  vectoreuc& pt16 = pts[15];
  vectoreuc& pt17 = pts[16];
  vectoreuc& pt18 = pts[17];
  vectoreuc& pt19 = pts[18];
  vectoreuc& pt20 = pts[19];
  for(int i=0; i<obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];
    assert(obs.pts.size() == 4);
    vectoreuc& obs1 = obs.pts[0];
    vectoreuc& obs2 = obs.pts[1];
    vectoreuc& obs3 = obs.pts[2];
    vectoreuc& obs4 = obs.pts[3];

    result.push_back(_20pt4obspt_svm(pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8,
                                    pt9, pt10, pt11, pt12, pt13, pt14, pt15, pt16,
                                    pt17, pt18, pt19, pt20,
                                    obs1, obs2, obs3, obs4));

    auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (obs.pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

  }

  return result;
}


vector<hyperplane> SvmSeperator::_32_4_seperate() {
  assert(pts.size() == 32);
  vector<hyperplane> result;
  vectoreuc& pt1 = pts[0];
  vectoreuc& pt2 = pts[1];
  vectoreuc& pt3 = pts[2];
  vectoreuc& pt4 = pts[3];
  vectoreuc& pt5 = pts[4];
  vectoreuc& pt6 = pts[5];
  vectoreuc& pt7 = pts[6];
  vectoreuc& pt8 = pts[7];
  vectoreuc& pt9 = pts[8];
  vectoreuc& pt10 = pts[9];
  vectoreuc& pt11 = pts[10];
  vectoreuc& pt12 = pts[11];
  vectoreuc& pt13 = pts[12];
  vectoreuc& pt14 = pts[13];
  vectoreuc& pt15 = pts[14];
  vectoreuc& pt16 = pts[15];
  vectoreuc& pt17 = pts[16];
  vectoreuc& pt18 = pts[17];
  vectoreuc& pt19 = pts[18];
  vectoreuc& pt20 = pts[19];
  vectoreuc& pt21 = pts[20];
  vectoreuc& pt22 = pts[21];
  vectoreuc& pt23 = pts[22];
  vectoreuc& pt24 = pts[23];
  vectoreuc& pt25 = pts[24];
  vectoreuc& pt26 = pts[25];
  vectoreuc& pt27 = pts[26];
  vectoreuc& pt28 = pts[27];
  vectoreuc& pt29 = pts[28];
  vectoreuc& pt30 = pts[29];
  vectoreuc& pt31 = pts[30];
  vectoreuc& pt32 = pts[31];
  for(int i=0; i<obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];
    assert(obs.pts.size() == 4);
    vectoreuc& obs1 = obs.pts[0];
    vectoreuc& obs2 = obs.pts[1];
    vectoreuc& obs3 = obs.pts[2];
    vectoreuc& obs4 = obs.pts[3];

    result.push_back(_32pt4obspt_svm(pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8,
                                    pt9, pt10, pt11, pt12, pt13, pt14, pt15, pt16,
                                    pt17, pt18, pt19, pt20, pt21, pt22, pt23, pt24,
                                    pt25, pt26, pt27, pt28, pt29, pt30, pt31, pt32,
                                    obs1, obs2, obs3, obs4));

    auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (obs.pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

  }

  return result;
}


vector<hyperplane> SvmSeperator::seperate() {
  vector<hyperplane> result;
  for(int i=0; i<obstacles->size(); i++) {

    bool fail = false;
    obstacle2D& obs = (*obstacles)[i];
#if SVM_SOLVER
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(obs.pts.size() + pts.size(), 3);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H(3, 3);
#else
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> A(obs.pts.size() + pts.size(), 3);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> H(3, 3);
#endif
    Eigen::Matrix<double, Eigen::Dynamic, 1> lbA(obs.pts.size() + pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> ubA(obs.pts.size() + pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> lb(3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> ub(3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> g(3);
    ub(0) = ub(1) = 1.0; // std::numeric_limits<double>::infinity();
    lb(0) = lb(1) = -1.0; //-std::numeric_limits<double>::infinity();
    ub(2) = std::numeric_limits<double>::infinity();
    lb(2) = -std::numeric_limits<double>::infinity();

    for(unsigned int i = 0; i < obs.pts.size(); i++) {
      A(i, 0) = obs.pts[i][0];
      A(i, 1) = obs.pts[i][1];
      A(i, 2) = -1.0;
      ubA(i) = std::numeric_limits<double>::max();
      lbA(i) = 1.0;
    }
    for(unsigned int i = 0; i < pts.size(); i++) {
      A(i + obs.pts.size(), 0) = pts[i][0];
      A(i + obs.pts.size(), 1) = pts[i][1];
      A(i + obs.pts.size(), 2) = -1.0;
      ubA(i + obs.pts.size()) = -1.0;
      lbA(i + obs.pts.size()) = std::numeric_limits<double>::lowest();
    }
    for(unsigned i = 0; i < 3; i++)
      for(unsigned j = 0; j < 3; j++)
        H(i, j) = (i == j ? 2.0 : 0.0);
    H(2, 2) = 0.0;
    g(0) = g(1) = g(2) = 0.0;

    Eigen::Matrix<double, Eigen::Dynamic, 1> y(3);
    y(0) = y(1) = y(2) = 0.0;

#if SVM_SOLVER
    qpOASES::QProblem qp(3, pts.size() + obs.pts.size(), qpOASES::HST_SEMIDEF);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qp.setOptions(options);
    qpOASES::int_t nWSR = 10000;

    // std::cout << "H: " << ob.H() << std::endl;
    // std::cout << "A: " << cb.A() << std::endl;

    // auto t0 = Time::now();
    qpOASES::returnValue status = qp.init(
      H.data(),
      g.data(),
      A.data(),
      lb.data(),
      ub.data(),
      lbA.data(),
      ubA.data(),
      nWSR,
      NULL);

    qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(status);

    if(simpleStatus != 0) {
      cout << "svm failed" << endl;
      for(unsigned int i = 0; i < obs.pts.size(); i++) {
        cout << obs.pts[i] << endl;
      }
      cout << "--" << endl;
      for(unsigned int i = 0; i < pts.size(); i++) {
        cout << pts[i] << endl;
      }
      //int a; cin >> a;
      cout << A << endl;
      cout << g << endl;
      cout << "lb" << endl << lbA << endl;
      cout << "ub" << endl << ubA << endl;
      //cin >> a;
      fail = true;
    } else {

    }

    qp.getPrimalSolution(y.data());

#else

    OSQPSettings settings;
    osqp_set_default_settings(&settings);

    settings.max_iter = 200000;
    int numVars = 3;
    int numConstraints = A.rows();

    std::vector<c_int> rowIndicesH(numVars * numVars);
    std::vector<c_int> columnIndicesH(numVars + 1);
    for (size_t c = 0; c < numVars; ++c) {
      for (size_t r = 0; r < numVars; ++r) {
        rowIndicesH[c * numVars + r] = r;
      }
      columnIndicesH[c] = c * numVars;
    }
    columnIndicesH[numVars] = numVars * numVars;

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
    data.P = csc_matrix(numVars, numVars, numVars * numVars, H.data(), rowIndicesH.data(), columnIndicesH.data());
    data.q = const_cast<double*>(g.data());
    data.A = csc_matrix(numConstraints, numVars, numConstraints * numVars, A.data(), rowIndicesA.data(), columnIndicesA.data());
    data.l = const_cast<double*>(lbA.data());
    data.u = const_cast<double*>(ubA.data());

    OSQPWorkspace* work = osqp_setup(&data, &settings);

    // Solve Problem
    osqp_warm_start_x(work, y.data());
    osqp_solve(work);
    if (work->info->status_val != OSQP_SOLVED) {
      cout << "svm failed" << endl;
      for(unsigned int i = 0; i < obs.pts.size(); i++) {
        cout << obs.pts[i] << endl;
      }
      cout << "--" << endl;
      for(unsigned int i = 0; i < pts.size(); i++) {
        cout << pts[i] << endl;
      }
      int a;
      //cin >> a;
      cout << A << endl;
      cout << g << endl;
      cout << "lb" << endl << lbA << endl;
      cout << "ub" << endl << ubA << endl;
      //cin >> a;
      fail = true;
    } else {

    }

    // work->solution->x
    for (size_t i = 0; i < numVars; ++i) {
      y(i) = work->solution->x[i];
    }
    osqp_cleanup(work);
#endif
    hyperplane pl;
    pl.normal = vectoreuc(2);
    pl.normal[0] = y(0);
    pl.normal[1] = y(1);
    pl.distance = y(2) / pl.normal.L2norm();
    pl.normal = pl.normal.normalized();
    result.push_back(pl);

    cout << "svm success" << endl;
    for(unsigned int i = 0; i < obs.pts.size(); i++) {
      cout << obs.pts[i] << endl;
    }
    cout << "--" << endl;
    for(unsigned int i = 0; i < pts.size(); i++) {
      cout << pts[i] << endl;
    }
    cout << "---" << endl << pl.normal << " " << pl.distance << endl;
    if(fail) {
      int  a;
      //cin >> a;
    }
  }


  return result;
}

vector<hyperplane> SvmSeperator::_16_4_seperate() {
  assert(pts.size() == 16);
  vector<hyperplane> result;
  vectoreuc& pt1 = pts[0];
  vectoreuc& pt2 = pts[1];
  vectoreuc& pt3 = pts[2];
  vectoreuc& pt4 = pts[3];
  vectoreuc& pt5 = pts[4];
  vectoreuc& pt6 = pts[5];
  vectoreuc& pt7 = pts[6];
  vectoreuc& pt8 = pts[7];
  vectoreuc& pt9 = pts[8];
  vectoreuc& pt10 = pts[9];
  vectoreuc& pt11 = pts[10];
  vectoreuc& pt12 = pts[11];
  vectoreuc& pt13 = pts[12];
  vectoreuc& pt14 = pts[13];
  vectoreuc& pt15 = pts[14];
  vectoreuc& pt16 = pts[15];

  for(int i=0; i<obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];
    assert(obs.pts.size() == 4);
    vectoreuc& obs1 = obs.pts[0];
    vectoreuc& obs2 = obs.pts[1];
    vectoreuc& obs3 = obs.pts[2];
    vectoreuc& obs4 = obs.pts[3];

    result.push_back(_16pt4obspt_svm(pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8,
                                    pt9, pt10, pt11, pt12, pt13, pt14, pt15, pt16,
                                    obs1, obs2, obs3, obs4));


  }

  return result;
}

vector<hyperplane> SvmSeperator::soft_seperate() {
  vector<hyperplane> result;
  for(size_t i = 0; i < obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];

    unsigned int numVars = 3 + obs.pts.size();
    unsigned int numConstraints = obs.pts.size() + pts.size();

    double C = 100.0;

#if SVM_SOLVER
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, numVars);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H(numVars, numVars);
#else
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> A(numConstraints, numVars);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> H(numVars, numVars);
#endif
    Eigen::Matrix<double, Eigen::Dynamic, 1> lbA(numConstraints);
    Eigen::Matrix<double, Eigen::Dynamic, 1> ubA(numConstraints);
    Eigen::Matrix<double, Eigen::Dynamic, 1> lb(numVars);
    Eigen::Matrix<double, Eigen::Dynamic, 1> ub(numVars);
    Eigen::Matrix<double, Eigen::Dynamic, 1> g(numVars);
    Eigen::Matrix<double, Eigen::Dynamic, 1> y(numVars);


    for(unsigned int m = 0; m < numVars; m++) {
      for(unsigned int n = 0; n < numVars; n++) {
        if(m == n) {
          if(m == 2)
            H(m, n) = 0.0;
          else if(m < 2)
            H(m, n) = 1.0;
          else
            H(m, n) = 0.0;
        } else {
          H(m, n) = 0.0;
        }
      }
      if(m > 2) {
        g(m) = C;
        lb(m) = 0;
        ub(m) = std::numeric_limits<double>::infinity();
      } else {
        g(m) = 0.0;
        lb(m) = -std::numeric_limits<double>::infinity();
        ub(m) = std::numeric_limits<double>::infinity();
      }
      y(m) = 0.0;
    }

    for(unsigned int m = 0; m < obs.pts.size(); m++) {
      for(unsigned int n = 0; n < numVars; n++) {
        A(m, n) = 0.0;
      }
      A(m, 0) = obs.pts[m][0];
      A(m, 1) = obs.pts[m][1];
      A(m, 2) = -1.0;
      A(m, 3 + m) = 1.0;
      lbA(m) = 1.0;
      ubA(m) = std::numeric_limits<double>::infinity();
    }

    for(unsigned int m = 0; m < pts.size(); m++) {
      for(unsigned int n = 0; n < numVars; n++) {
        A(m + obs.pts.size(), n) = 0.0;
      }
      A(m + obs.pts.size(), 0) = pts[m][0];
      A(m + obs.pts.size(), 1) = pts[m][1];
      A(m + obs.pts.size(), 2) = -1.0;
      lbA(m + obs.pts.size()) = -std::numeric_limits<double>::infinity();
      ubA(m + obs.pts.size()) = -1.0;
    }

#if SVM_SOLVER
    qpOASES::QProblem qp(numVars, numConstraints, qpOASES::HST_SEMIDEF);
    qpOASES::Options options;
    options.setToDefault();
    options.printLevel = qpOASES::PL_LOW;
    qp.setOptions(options);
    qpOASES::int_t nWSR = 10000;

    // std::cout << "H: " << ob.H() << std::endl;
    // std::cout << "A: " << cb.A() << std::endl;

    // auto t0 = Time::now();
    qpOASES::returnValue status = qp.init(
      H.data(),
      g.data(),
      A.data(),
      lb.data(),
      ub.data(),
      lbA.data(),
      ubA.data(),
      nWSR,
      NULL);

    qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(status);

    if(simpleStatus != 0) {
      cout << "svm failed" << endl;
      for(unsigned int i = 0; i < obs.pts.size(); i++) {
        cout << obs.pts[i] << endl;
      }
      cout << "--" << endl;
      for(unsigned int i = 0; i < pts.size(); i++) {
        cout << pts[i] << endl;
      }
      int a; cin >> a;
      cout << A << endl;
      cout << g << endl;
      cout << "lb" << endl << lbA << endl;
      cout << "ub" << endl << ubA << endl;
      cin >> a;
    } else {

    }

    qp.getPrimalSolution(y.data());

#else

    OSQPSettings settings;
    osqp_set_default_settings(&settings);

    settings.max_iter = 200000;

    std::vector<c_int> rowIndicesH(numVars * numVars);
    std::vector<c_int> columnIndicesH(numVars + 1);
    for (size_t c = 0; c < numVars; ++c) {
      for (size_t r = 0; r < numVars; ++r) {
        rowIndicesH[c * numVars + r] = r;
      }
      columnIndicesH[c] = c * numVars;
    }
    columnIndicesH[numVars] = numVars * numVars;

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
    data.P = csc_matrix(numVars, numVars, numVars * numVars, H.data(), rowIndicesH.data(), columnIndicesH.data());
    data.q = const_cast<double*>(g.data());
    data.A = csc_matrix(numConstraints, numVars, numConstraints * numVars, A.data(), rowIndicesA.data(), columnIndicesA.data());
    data.l = const_cast<double*>(lbA.data());
    data.u = const_cast<double*>(ubA.data());

    OSQPWorkspace* work = osqp_setup(&data, &settings);

    // Solve Problem
    osqp_warm_start_x(work, y.data());
    osqp_solve(work);
    if (work->info->status_val != OSQP_SOLVED) {
      cout << "svm failed" << endl;
      for(unsigned int i = 0; i < obs.pts.size(); i++) {
        cout << obs.pts[i] << endl;
      }
      cout << "--" << endl;
      for(unsigned int i = 0; i < pts.size(); i++) {
        cout << pts[i] << endl;
      }
      int a;
      cin >> a;
      cout << A << endl;
      cout << g << endl;
      cout << "lb" << endl << lbA << endl;
      cout << "ub" << endl << ubA << endl;
      cin >> a;
    } else {

    }

    // work->solution->x
    for (size_t i = 0; i < numVars; ++i) {
      y(i) = work->solution->x[i];
    }
    osqp_cleanup(work);
#endif
    hyperplane pl;
    pl.normal = vectoreuc(2);
    pl.normal[0] = y(0);
    pl.normal[1] = y(1);
    pl.distance = y(2) / pl.normal.L2norm();
    pl.normal = pl.normal.normalized();
    result.push_back(pl);

    /*cout << "svm success" << endl;
    for(unsigned int i = 0; i < obs.pts.size(); i++) {
      cout << obs.pts[i] << endl;
    }
    cout << "--" << endl;
    for(unsigned int i = 0; i < pts.size(); i++) {
      cout << pts[i] << endl;
    }
    cout << "---" << endl << pl.normal << " " << pl.distance << endl;
    /*if(rand() % 1000 == 0) {
    int a; cin >> a;}*/
  }
  return result;
}

vector<hyperplane> SvmSeperator::libsvm_seperate() {
  vector<hyperplane> result;
  for(auto& obs : (*obstacles)) {
    struct svm_problem* prob = new svm_problem;
    prob->l = obs.size() + pts.size();
    prob->y = new double[prob->l];
    prob->x = new svm_node*[prob->l];
    for(size_t i = 0; i < obs.size(); i++) {
      struct svm_node* n = new svm_node[3];
      n[0].index = 1;
      n[0].value = obs[i][0];
      n[1].index = 2;
      n[1].value = obs[i][1];
      n[2].index = -1;
      prob->y[i] = 1;
      prob->x[i] = n;
    }

    for(size_t i = 0; i < pts.size(); i++) {
      struct svm_node* n = new svm_node[3];
      n[0].index = 1;
      n[0].value = pts[i][0];
      n[1].index = 2;
      n[1].value = pts[i][1];
      n[2].index = -1;
      prob->y[obs.size() + i] = -1;
      prob->x[obs.size() + i] = n;
    }

    struct svm_parameter* param = new svm_parameter;
    param->svm_type = C_SVC;
    param->kernel_type = LINEAR;
    param->cache_size = 64;
    param->eps = 0.001;
    param->C = 100;
    param->nr_weight = 0;

    struct svm_model* model = svm_train(prob, param);

    /*cout << "support vectors from SV" << endl;
    for(unsigned int i = 0; i < model->l; i++) {
      int cur = 0;
      while((model->SV)[i][cur].index  != -1) {
        cout << (model->SV)[i][cur].value << ", ";
        cur++;
      }
      cout << endl;
    }

    cout << endl << "support vectors from training set" << endl;
    for(unsigned i = 0; i < model->l; i++) {
      int idx = model->sv_indices[i] - 1;
      if(idx >= obs.size()) {
        cout << pts[idx - obs.size()][0] << ", " << pts[idx - obs.size()][1] << endl;
      } else {
        cout << obs[idx][0] << ", " << obs[idx][1] << endl;
      }
    }
    */
    vectoreuc normal(2);
    normal.zero();
    double distance = model->rho[0];

    for(unsigned int i = 0; i < model->l; i++) {
      int idx = model->sv_indices[i] - 1;
      vectoreuc sv;
      if(idx >= obs.size()) {
        sv = pts[idx - obs.size()];
      } else {
        sv = obs[idx];
      }
      normal = normal + sv * model->sv_coef[0][i];
    }

    distance /= normal.L2norm();
    normal = normal.normalized();

    hyperplane hp;
    hp.normal = normal;
    hp.distance = distance;

    /*cout << "testing svm" << endl;
    for(size_t i = 0; i < obs.size(); i++) {
      cout << "obs " << hp.dist(obs[i]) << endl;
    }
    for(size_t i = 0; i < pts.size(); i++) {
      cout << "pts " << hp.dist(pts[i]) << endl;
    }*/


    result.push_back(hp);


  }
  return result;
}
