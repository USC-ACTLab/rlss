#include "svm_seperator.h"
#include "libsvm/svm.h"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include "svmcvxwrapper_2pt_4obs.h"
#include "svmcvxwrapper_8pt_4obs.h"
#include "svmcvxwrapper_32pt_4obs.h"
#include <cassert>
#include <qpOASES.hpp>
#include <Eigen/Dense>

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

    /*auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane (obs.pts) normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl
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

vector<hyperplane> SvmSeperator::seperate() {
  vector<hyperplane> result;
  for(int i=0; i<obstacles->size(); i++) {
    obstacle2D& obs = (*obstacles)[i];
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A(obs.pts.size() + pts.size(), 3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> lbA(obs.pts.size() + pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> ubA(obs.pts.size() + pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H(3, 3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> lb(3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> ub(3);
    Eigen::Matrix<double, Eigen::Dynamic, 1> g(3);
    ub(0) = ub(1) = ub(2) = std::numeric_limits<double>::max();
    lb(0) = lb(1) = lb(2) = std::numeric_limits<double>::min();

    for(unsigned int i = 0; i < obs.pts.size(); i++) {
      A(i, 0) = -obs.pts[i][0];
      A(i, 1) = -obs.pts[i][1];
      A(i, 2) = 1.0;
      ubA(i) = -1.0;
      lbA(i) = std::numeric_limits<double>::min();
    }

    for(unsigned int i = 0; i < pts.size(); i++) {
      A(i + obs.pts.size(), 0) = pts[i][0];
      A(i + obs.pts.size(), 1) = pts[i][1];
      A(i + obs.pts.size(), 2) = -1.0;
      ubA(i) = -1.0;
      lbA(i) = std::numeric_limits<double>::min();
    }
    for(unsigned i = 0; i < 3; i++)
      for(unsigned j = 0; j < 3; j++)
        H(i, j) = (i == j ? 2.0 : 0.0);
    H(2, 2) = 0.0;
    g(0) = g(1) = g(2) = 0.0;

    Eigen::Matrix<double, Eigen::Dynamic, 1> y(3);
    y(0) = y(1) = y(2) = 0.0;

    qpOASES::QProblem qp(3, pts.size() + obs.pts.size());
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
      NULL,
      y.data());

    qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(status);

    if(simpleStatus != 0) {
      hyperplane pl;
      pl.normal[0] = y(0);
      pl.normal[1] = y(1);
      pl.distance = y(2) / pl.normal.L2norm();
      pl.normal = pl.normal.normalized();
      result.push_back(pl);
    } else {
      cout << "svm failed" << endl;
    }
  }

  return result;
}
