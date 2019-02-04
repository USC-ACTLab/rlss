#include <Eigen/Dense>

using Eigen::Hyperplane;
using Eigen::AlignedBox;
using std::string;
using std::to_string;

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
}
