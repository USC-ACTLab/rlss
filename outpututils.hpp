#include <Eigen/Dense>
#include "spline.h"
#include "bezier.h"
#include "json.hpp"

using Eigen::Hyperplane;
using Eigen::AlignedBox;
using std::string;
using std::to_string;
using nlohmann::json;
using splx::Spline;
using splx::Bezier;

template<class T, unsigned int DIM>
using Vector = Eigen::Matrix<T, DIM, 1U>;

namespace ACT {
  template<class T, unsigned int DIM>
  string string_hyperplane(const Hyperplane<T,DIM>& hp) {
    return string("[(") + to_string(hp.normal()(0)) + " " + to_string(hp.normal()(1)) + " "
      + to_string(hp.normal()(2)) + ") " + to_string(hp.offset()) + "]";
  }

  template<class T, unsigned int DIM>
  string string_vector(const Vector<T, DIM>& vec) {
    string res("(");
    for(unsigned int i = 0; i < DIM; i++) {
      res += to_string(vec(i));
      res += " ";
    }
    res.pop_back();
    res.push_back(')');
    return res;
  }

  template<class T, unsigned int DIM>
  json json_vectordim(const Vector<T, DIM>& vec) {
    json vecjsn;
    for(unsigned int i = 0; i < DIM; i++) {
      vecjsn.push_back(vec(i));
    }
    return vecjsn;
  }

  template<class T, unsigned int DIM>
  json json_bezier(const Bezier<T, DIM>& bez) {
    json bezjsn;
    bezjsn["min_parameter"] = 0;
    bezjsn["max_parameter"] = bez.m_a;
    bezjsn["type"] = "bezier";
    for(const auto& cpt: bez.m_controlPoints) {
      bezjsn["controlpoints"].push_back(json_vectordim<T, DIM>(cpt));
    }
    return bezjsn;
  }

  template<class T, unsigned int DIM>
  json json_spline(const Spline<T, DIM>& spl) {
    json res;
    for(unsigned int i = 0; i < spl.numPieces(); i++) {
      auto bezptr = std::static_pointer_cast<Bezier<T, DIM>>(spl.getPiece(i));
      res.push_back(json_bezier<T, DIM>(*bezptr));
    }
    return res;
  }


}
