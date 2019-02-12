#include <Eigen/Dense>
#include "spline.h"
#include "bezier.h"
#include "json.hpp"
#include "PointCloud.h"
#include <Eigen/StdVector>

using ACT::PointCloud;
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
  json json_vec_vectordim(const vector<Vector<T, DIM>,
      Eigen::aligned_allocator<Vector<T, DIM>>>& vectors) {
    json res;
    for(const auto& vec: vectors) {
      res.push_back(json_vectordim<T, DIM>(vec));
    }
    return res;
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

  template<class T, unsigned int DIM>
  json json_robot_position(const Spline<T, DIM>& spl, unsigned int robot_id, T t) {
    json res;
    res["robot_id"] = robot_id;
    res["position"] = json_vectordim<double, 3U>(spl.eval(t, 0));
    return res;
  }

  template<class T, unsigned int DIM>
  json json_obstacle(const PointCloud<T, DIM>& obs) {
    json res;
    for(const auto& pt: obs._pts) {
      res.push_back(json_vectordim<T, DIM>(pt));
    }
    return res;
  }

  template<class T, unsigned int DIM>
  json json_obstacles(const vector<PointCloud<T, DIM>>& obstacles) {
    json result;
    for(const auto& obs: obstacles) {
      result.push_back(json_obstacle<T, DIM>(obs));
    }
    return result;
  }

  template<class T, unsigned int DIM>
  json json_box(const AlignedBox<T, DIM>& box) {
    json res;
    res["min"] = json_vectordim<T, DIM>(box.min());
    res["max"] = json_vectordim<T, DIM>(box.max());
    return res;
  }

  template<class T, unsigned int DIM>
  json json_occupied_cells(const vector<AlignedBox<T, DIM>>& boxes) {
    json res;
    for(const auto& box: boxes) {
      res.push_back(json_box<T, DIM>(box));
    }
    return res;
  }

  template<class T, unsigned int DIM>
  json json_hyperplane(const Hyperplane<T, DIM>& hp) {
    json res;
    res["normal"] = json_vectordim<double, 3U>(hp.normal());
    res["distance"] = -hp.offset();
    return res;
  }

  template<class T, unsigned int DIM>
  void json_svm_hyperplanes_of_piece_insert(json& insert_to, const vector<Hyperplane<T, DIM>>& hps,
    unsigned int piece_id) {
    for(const auto& hp: hps) {
      json svmsingle;
      svmsingle["piece_id"] = piece_id;
      svmsingle["hyperplane"] = json_hyperplane<T, DIM>(hp);
      insert_to.push_back(svmsingle);
    }
  }

  template<class T, unsigned int DIM>
  json json_voronoi_hyperplanes_of_robot(const vector<Hyperplane<T, DIM>>& hps,
      unsigned int robot_id) {
    json res;
    res["robot_id"] = robot_id;
    for(const auto& hp: hps) {
      res["hyperplanes"].push_back(json_hyperplane<T, DIM>(hp));
    }
    return res;
  }


}
