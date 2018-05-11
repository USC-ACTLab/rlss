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

    auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance - 1e-6) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }
    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance + 1e-6) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

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

    auto& hp = result.back();
    for (auto& pt : obs.pts) {
      if (pt.dot(hp.normal) < hp.distance - 1e-6) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance + 1e-6) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
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
      if (pt.dot(hp.normal) < hp.distance - 1e-6) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }

    for (auto& pt : pts) {
      if (pt.dot(hp.normal) > hp.distance + 1e-6) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
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
    obstacle2D& obs = (*obstacles)[i];
    int l = obs.ch.size()-1 + pts.size();
    double* y = (double*) malloc(sizeof(double) * l);
    struct svm_node** x = (struct svm_node**)malloc(sizeof(struct svm_node*) * l);
    int j;
    vector<vectoreuc> all_vecs;

    for(j = 0; j<pts.size(); j++) {
      struct svm_node* data = (struct svm_node*)malloc(sizeof(struct svm_node)*3);
      // cout << pts[j] << endl;
      data[0].index = 1;
      data[0].value = pts[j][0];
      data[1].index = 2;
      data[1].value = pts[j][1];
      data[2].index = -1;
      x[j] = data;
      y[j] = 1;
      all_vecs.push_back(pts[j]);
    }

    for(int k=0; k<obs.ch.size()-1; k++) {
      struct svm_node* data = (struct svm_node*)malloc(sizeof(struct svm_node) * 3);
      // cout << obs.pts[obs.ch[k]] << endl;
      data[0].index = 1;
      data[0].value = obs.pts[obs.ch[k]][0];
      data[1].index = 2;
      data[1].value = obs.pts[obs.ch[k]][1];
      data[2].index = -1;
      x[j+k] = data;
      y[j+k] = 0;
      all_vecs.push_back(obs.pts[obs.ch[k]]);
    }


    struct svm_problem problem;
    problem.l = l;
    problem.y = y;
    problem.x = x;

    struct svm_parameter parameter;
    parameter.svm_type = C_SVC;
    parameter.kernel_type = LINEAR;
    parameter.cache_size = 64; // dont have any idea
    parameter.eps = 0.01;
    parameter.C = 10000; // dunno
    //parameter.nu = 0.5;
    parameter.nr_weight = 0; // completely dunno


    struct svm_model* model = svm_train(&problem, &parameter);


    double* coefs = model->sv_coef[0];

    int totalsvcount = model->l;

    int idxs[totalsvcount];
    svm_get_sv_indices(model, idxs);

    vectoreuc normal(2);
    normal.zero();

    for(int i=0; i<totalsvcount; i++) {
      // cout << "SV " << i << ": " << all_vecs[idxs[i]-1] << endl;
      normal[0] += coefs[i] * all_vecs[idxs[i]-1][0];
      normal[1] += coefs[i] * all_vecs[idxs[i]-1][1];
    }

    double length = normal.L2norm();
    double distance = model->rho[0] / length;
    normal = normal.normalized();

    hyperplane hp;
    hp.normal = normal;
    hp.distance = distance;



    double closest_obs_sv = -1*numeric_limits<double>::infinity();
    for(int i=0; i<obs.ch.size()-1; i++) {
      double dst = hp.dist(obs.pts[obs.ch[i]]);

      closest_obs_sv = max(closest_obs_sv, hp.dist(obs.pts[obs.ch[i]]));

    }

    hp.distance += closest_obs_sv;

    hp.normal = hp.normal * -1;
    hp.distance = hp.distance * -1;

    // hp.distance -= 0.05;

    // safety check...
    for (auto& pt : pts) {
      if (pt.dot(hp.normal) - 1e-4 > hp.distance) {
        std::stringstream sstr;
        sstr << "Couldn't find hyperplane normal " << hp.normal << " dist: " << hp.distance << " pt: " << pt << " dot product: " << pt.dot(hp.normal);
        // throw runtime_error(sstr.str());
        std::cerr << sstr.str() << std::endl;
      }
    }


    result.push_back(hp);
    // std::cout << "normal: " << hp.normal << " dist: " << hp.distance << std::endl;
  }

  return result;
}
