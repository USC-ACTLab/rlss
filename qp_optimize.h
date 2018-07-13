#ifndef QP_OPTIMIZE_H
#define QP_OPTIMIZE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <vector>

// qpOASES assumes row-major!
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
typedef Eigen::VectorXd Vector;



struct hyperplaneData
{
  unsigned int from_pt;
  unsigned int to_pt;
  Vector normal;
  double dist;
};

struct endCloseToData
{
  double time;
  double lambda;
  Vector value;
};

class ObjectiveBuilder
{
public:
  ObjectiveBuilder(
    size_t dimension,
    const std::vector<double>& pieceDurations)
    : m_dimension(dimension)
    , m_pieceDurations(pieceDurations)
    , m_numPieces(pieceDurations.size())
    , m_numVars(dimension * 8 * m_numPieces)
    , m_H(m_numVars, m_numVars)
    , m_g(m_numVars)
  {
    m_H.setZero();
    m_g.setZero();
  }

  static void Init()
  {
    // B.resize(8,8);
    // B <<   1,    0,    0,    0,    0,   0,  0, 0,
    //       -7,    7,    0,    0,    0,   0,  0, 0,
    //       21,  -42,   21,    0,    0,   0,  0, 0,
    //      -35,  105, -105,   35,    0,   0,  0, 0,
    //       35, -140,  210, -140,   35,   0,  0, 0,
    //      -21,  105, -210,  210, -105,  21,  0, 0,
    //        7,  -42,  105, -140,  105, -42,  7, 0,
    //       -1,    7,  -21,   35,  -35,  21, -7, 1;
  }

  void minDerivativeSquared(
    double lambda_vel, double lambda_acc, double lambda_jerk, double lambda_snap)
  {
    // std::cout << H_1d_curve << std::endl;

    // H is just blockdiagonal of H_1d_curve
    for (size_t i = 0; i < m_numVars; i+=8) {
      double T = m_pieceDurations[(i % (8 * m_numPieces)) /8];
      std::cout << "T " << T << std::endl;
      Matrix Q = lambda_vel * Qvel(T) + lambda_acc * Qacc(T) + lambda_jerk * Qjerk(T) + lambda_snap * Qsnap(T);
      Matrix Bmatrix = B(T);
      Matrix H_1d_curve = Bmatrix.transpose() * Q * Bmatrix;

      m_H.block(i, i, 8, 8) += H_1d_curve;
    }

    // std::cout << m_H;
  }

  // Expand[(Limit[D[f1[t], {t, 0}], t -> 1] - X)^2]
  // X^2 - 2 X y[7] + y[7]^2
  void endCloseTo(
    size_t piece,
    double lambda,
    const Vector& value)
  {
    for (size_t d = 0; d < m_dimension; ++d) {
      size_t idx = column(piece, d) + 7;
      m_H(idx, idx) += 2 * lambda; // objective function is 1/2 x'Hx => multiply by 2 here
      m_g(idx) += -2 * value(d) * lambda;
    }
  }

  // Maximize distance to the given hyperplane
  // d^2 - 2 d n1 x1 - 2 d n2 x2 + n1^2 x1^2 + 2 n1 n2 x1 x2 + n2^2 x2^2
  void maxHyperplaneDist(
    size_t piece,
    double lambda,
    const Vector& normal,
    double dist)
  {
    for (size_t i = 0; i < 8; ++i) {
      size_t idx_x = column(piece, 0) + i;
      size_t idx_y = column(piece, 1) + i;
     /* m_H(idx_x, idx_x) += -2 * normal[0] * normal[0] * lambda;
      m_H(idx_y, idx_y) += -2 * normal[1] * normal[1] * lambda;
      m_H(idx_x, idx_y) += -2 * normal[0] * normal[1] * lambda;
      m_H(idx_y, idx_x) += -2 * normal[0] * normal[1] * lambda;
      m_g(idx_x) += 2 * dist * normal[0] * lambda;
      m_g(idx_y) += 2 * dist * normal[1] * lambda;
    */
      m_g(idx_x) += normal[0] * lambda;
      m_g(idx_y) += normal[1] * lambda;
    }
  }

  const Matrix& H() const
  {
    return m_H;
  }

  const Vector& g() const
  {
    return m_g;
  }

private:
  size_t column(size_t piece, size_t dimension)
  {
    return dimension * 8 * m_numPieces + piece * 8;
  }

  Matrix Qvel(double T)
  {
    Matrix Qvel(8,8);
    Qvel <<
    0,0,0,0,0,0,0,0,
    0,T,pow(T,2),pow(T,3),pow(T,4),pow(T,5),pow(T,6),pow(T,7),
    0,pow(T,2),(4*pow(T,3))/3.,(3*pow(T,4))/2.,(8*pow(T,5))/5.,(5*pow(T,6))/3.,(12*pow(T,7))/7.,(7*pow(T,8))/4.,
    0,pow(T,3),(3*pow(T,4))/2.,(9*pow(T,5))/5.,2*pow(T,6),(15*pow(T,7))/7.,(9*pow(T,8))/4.,(7*pow(T,9))/3.,
    0,pow(T,4),(8*pow(T,5))/5.,2*pow(T,6),(16*pow(T,7))/7.,(5*pow(T,8))/2.,(8*pow(T,9))/3.,(14*pow(T,10))/5.,
    0,pow(T,5),(5*pow(T,6))/3.,(15*pow(T,7))/7.,(5*pow(T,8))/2.,(25*pow(T,9))/9.,3*pow(T,10),(35*pow(T,11))/11.,
    0,pow(T,6),(12*pow(T,7))/7.,(9*pow(T,8))/4.,(8*pow(T,9))/3.,3*pow(T,10),(36*pow(T,11))/11.,(7*pow(T,12))/2.,
    0,pow(T,7),(7*pow(T,8))/4.,(7*pow(T,9))/3.,(14*pow(T,10))/5.,(35*pow(T,11))/11.,(7*pow(T,12))/2.,(49*pow(T,13))/13.;
    return Qvel;
  }

  Matrix Qacc(double T)
  {
    Matrix Qacc(8,8);
    Qacc <<
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,4*T,6*pow(T,2),8*pow(T,3),10*pow(T,4),12*pow(T,5),14*pow(T,6),
    0,0,6*pow(T,2),12*pow(T,3),18*pow(T,4),24*pow(T,5),30*pow(T,6),36*pow(T,7),
    0,0,8*pow(T,3),18*pow(T,4),(144*pow(T,5))/5.,40*pow(T,6),(360*pow(T,7))/7.,63*pow(T,8),
    0,0,10*pow(T,4),24*pow(T,5),40*pow(T,6),(400*pow(T,7))/7.,75*pow(T,8),(280*pow(T,9))/3.,
    0,0,12*pow(T,5),30*pow(T,6),(360*pow(T,7))/7.,75*pow(T,8),100*pow(T,9),126*pow(T,10),
    0,0,14*pow(T,6),36*pow(T,7),63*pow(T,8),(280*pow(T,9))/3.,126*pow(T,10),(1764*pow(T,11))/11.;
    return Qacc;
  }

  Matrix Qjerk(double T)
  {
    Matrix Qjerk(8,8);
    Qjerk <<
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,36*T,72*pow(T,2),120*pow(T,3),180*pow(T,4),252*pow(T,5),
    0,0,0,72*pow(T,2),192*pow(T,3),360*pow(T,4),576*pow(T,5),840*pow(T,6),
    0,0,0,120*pow(T,3),360*pow(T,4),720*pow(T,5),1200*pow(T,6),1800*pow(T,7),
    0,0,0,180*pow(T,4),576*pow(T,5),1200*pow(T,6),(14400*pow(T,7))/7.,3150*pow(T,8),
    0,0,0,252*pow(T,5),840*pow(T,6),1800*pow(T,7),3150*pow(T,8),4900*pow(T,9);

    return Qjerk;
  }

  Matrix Qsnap(double T)
  {
    Matrix Qsnap(8,8);
    Qsnap <<
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,576*T,1440*pow(T,2),2880*pow(T,3),5040*pow(T,4),
    0,0,0,0,1440*pow(T,2),4800*pow(T,3),10800*pow(T,4),20160*pow(T,5),
    0,0,0,0,2880*pow(T,3),10800*pow(T,4),25920*pow(T,5),50400*pow(T,6),
    0,0,0,0,5040*pow(T,4),20160*pow(T,5),50400*pow(T,6),100800*pow(T,7);
    return Qsnap;
  }

  Matrix B(double T)
  {
    Matrix B(8,8);
    B <<
       1,    0,    0,    0,    0,   0,  0, 0,
      -7 / T,    7 / T,    0,    0,    0,   0,  0, 0,
      21 / pow(T,2),  -42/ pow(T,2),   21/ pow(T,2),    0,    0,   0,  0, 0,
     -35/ pow(T,3),  105/ pow(T,3), -105/ pow(T,3),   35/ pow(T,3),    0,   0,  0, 0,
      35/ pow(T,4), -140/ pow(T,4),  210/ pow(T,4), -140/ pow(T,4),   35/ pow(T,4),   0,  0, 0,
     -21/ pow(T,5),  105/ pow(T,5), -210/ pow(T,5),  210/ pow(T,5), -105/ pow(T,5),  21/ pow(T,5),  0, 0,
       7/ pow(T,6),  -42/ pow(T,6),  105/ pow(T,6), -140/ pow(T,6),  105/ pow(T,6), -42/ pow(T,6),  7/ pow(T,6), 0,
      -1/ pow(T,7),    7/ pow(T,7),  -21/ pow(T,7),   35/ pow(T,7),  -35/ pow(T,7),  21/ pow(T,7), -7/ pow(T,7), 1/ pow(T,7);
    return B;
  }

private:


private:
  size_t m_dimension;
  std::vector<double> m_pieceDurations;
  size_t m_numPieces;
  const size_t m_numVars;

  Matrix m_H;
  Vector m_g;
};

class ConstraintBuilder
{
public:
  static constexpr double EPS = 0;//1e-3;

  ConstraintBuilder(
    size_t dimension,
    const std::vector<double>& pieceDurations)
    : m_dimension(dimension)
    , m_pieceDurations(pieceDurations)
    , m_numPieces(pieceDurations.size())
    , m_numVars(dimension * 8 * m_numPieces)
    , m_lb(m_numVars)
    , m_ub(m_numVars)
  {
  }

  // adds a constraint for a fixed position at the beginning of the specified piece
  // derivative 0 => position; 1 => velocity, etc.
  void addConstraintBeginning(
    size_t piece,
    size_t derivative,
    const Vector& value)
  {
    assert(value.size() == m_dimension);
    assert(derivative < 8);
    size_t idx = addConstraints(m_dimension);

    double T = m_pieceDurations[piece];

    for (size_t d = 0; d < m_dimension; ++d) {
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(piece, d) + i) = derivativeBeginning[derivative][i] / pow(T, derivative);
      }
      std::stringstream sstr;
      sstr << "(constraintBeginning piece: " << piece << ", derivative: " << derivative << ")";
      m_info[idx + d] = sstr.str();
      m_lbA(idx + d) = value(d);// - EPS / 2.0;
      m_ubA(idx + d) = value(d);// + EPS / 2.0;
    }
  }

  // adds a constraint for a fixed position at the end of the specified piece
  // derivative 0 => position; 1 => velocity, etc.
  void addConstraintEnd(
    size_t piece,
    size_t derivative,
    const Vector& value)
  {
    assert(value.size() == m_dimension);
    assert(derivative < 8);
    size_t idx = addConstraints(m_dimension);

    double T = m_pieceDurations[piece];

    for (size_t d = 0; d < m_dimension; ++d) {
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(piece, d) + i) = derivativeEnd[derivative][i] / pow(T, derivative);
      }
      // std::cout << "addConstraint: " << idx +d << " (constraintEnd " << piece << "," << derivative << ")" << std::endl;
      std::stringstream sstr;
      sstr << "(constraintEnd piece: " << piece << ", derivative: " << derivative << ")";
      m_info[idx + d] = sstr.str();

      m_lbA(idx + d) = value(d);// - EPS / 2.0;
      m_ubA(idx + d) = value(d);// + EPS / 2.0;
    }
  }

  // ensures that the specified derivative is identical for firstPiece and firstPiece+1
  // derivative 0 => position; 1 => velocity, etc.
  void addContinuity(
    size_t firstPiece,
    size_t derivative)
  {
    assert(firstPiece < m_numPieces - 1);
    size_t idx = addConstraints(m_dimension);

    double Tfirst = m_pieceDurations[firstPiece];
    double Tsecond = m_pieceDurations[firstPiece+1];

    for (size_t d = 0; d < m_dimension; ++d) {
      // derivative of end of first piece
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(firstPiece, d) + i) = derivativeEnd[derivative][i] / pow(Tfirst, derivative);
      }
      // negative derivative of beginning of next piece
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(firstPiece+1, d) + i) = -derivativeBeginning[derivative][i] / pow(Tsecond, derivative);
      }
      // std::cout << "addConstraint: " << idx +d << " (continuity " << firstPiece << "," << derivative << ")" << std::endl;

      std::stringstream sstr;
      sstr << "(continuity piece: " << firstPiece << ", derivative: " << derivative << ")";
      m_info[idx + d] = sstr.str();

      m_lbA(idx + d) = 0;//-0.00001;
      m_ubA(idx + d) = 0;//0.00001;
    }
  }

  // ensures that all control points of the piece lie in half-space
  // True if cp . normal <= dist
  void addHyperplane(
    size_t piece,
    Vector normal,
    double dist)
  {
    assert(piece < m_numPieces);

    if (piece == 0) {
      // one constraint per control point
      size_t idx = addConstraints(7);

      for (size_t cp = 1; cp < 8; ++cp) {
        for (size_t d = 0; d < m_dimension; ++d) {
          m_A(idx + cp - 1, column(piece, d) + cp) = normal(d);
        }
        // std::cout << "addConstraint: " << idx+cp << " (hyperplane " << piece << ")" << std::endl;
        std::stringstream sstr;
        sstr << "(hyperspace piece: " << piece << ", cp: " << cp << ")";
        m_info[idx + cp - 1] = sstr.str();

        m_lbA(idx + cp - 1) = std::numeric_limits<double>::lowest();
        m_ubA(idx + cp - 1) = dist;// + 1e-3;// + EPS;
      }

    } else {
      // one constraint per control point
      size_t idx = addConstraints(8);

      for (size_t cp = 0; cp < 8; ++cp) {
        for (size_t d = 0; d < m_dimension; ++d) {
          m_A(idx + cp, column(piece, d) + cp) = normal(d);
        }
        // std::cout << "addConstraint: " << idx+cp << " (hyperplane " << piece << ")" << std::endl;
        std::stringstream sstr;
        sstr << "(hyperspace piece: " << piece << ", cp: " << cp << ")";
        m_info[idx + cp] = sstr.str();

        m_lbA(idx + cp) = std::numeric_limits<double>::lowest();
        m_ubA(idx + cp) = dist;// + 1e-3;// + EPS;
      }
    }



  }

  // adds bounds to the decision variables
  void addBounds(
    size_t piece,
    Vector min,
    Vector max)
  {
    assert(piece < m_numPieces);

    for (size_t d = 0; d < m_dimension; ++d) {
      size_t idx = column(piece, d);
      for (size_t cp = 0; cp < 8; ++cp) {
        m_lb(idx + cp) = min(d);
        m_ub(idx + cp) = max(d);
      }
    }
  }

  const Matrix& A() const
  {
    return m_A;
  }

  const Vector& lbA() const
  {
    return m_lbA;
  }

  const Vector& ubA() const
  {
    return m_ubA;
  }

  const Vector& lb() const
  {
    return m_lb;
  }

  const Vector& ub() const
  {
    return m_ub;
  }

  const std::string& info(size_t constraint) const
  {
    return m_info[constraint];
  }

private:
  size_t addConstraints(size_t numConstraints)
  {
    size_t idx = m_A.rows();
    m_A.conservativeResize(idx + numConstraints, m_numVars);
    m_lbA.conservativeResize(idx + numConstraints);
    m_ubA.conservativeResize(idx + numConstraints);
    m_A.block(idx, 0, numConstraints, m_numVars).setZero();
    m_info.resize(idx + numConstraints);
    return idx;
  }

  size_t column(size_t piece, size_t dimension)
  {
    return dimension * 8 * m_numPieces + piece * 8;
  }

private:
  // Limit[D[f1[t], {t, 0}], t -> 0]: y[0]
  // Limit[D[f1[t], {t, 1}], t -> 0]: -7 y[0] + 7 y[1]
  // Limit[D[f1[t], {t, 2}], t -> 0]: 42 y[0] - 84 y[1] + 42 y[2]
  // Limit[D[f1[t], {t, 3}], t -> 0]: -210 y[0] + 630 y[1] - 630 y[2] + 210 y[3]
  // Limit[D[f1[t], {t, 4}], t -> 0]: 840 y[0] - 3360 y[1] + 5040 y[2] - 3360 y[3] + 840 y[4]
  // Limit[D[f1[t], {t, 5}], t -> 0]:-2520 y[0] + 12600 y[1] - 25200 y[2] + 25200 y[3] - 12600 y[4] + 2520 y[5]
  // Limit[D[f1[t], {t, 6}], t -> 0]: 5040 y[0] - 30240 y[1] + 75600 y[2] - 100800 y[3] + 75600 y[4] - 30240 y[5] + 5040 y[6]
  // Limit[D[f1[t], {t, 7}], t -> 0]: -5040 y[0] + 35280 y[1] - 105840 y[2] + 176400 y[3] - 176400 y[4] + 105840 y[5] - 35280 y[6] + 5040 y[7]
  static constexpr const double derivativeBeginning[8][8] = {
    {1 , 0, 0, 0, 0, 0, 0, 0},
    {-7, 7, 0, 0, 0, 0, 0, 0},
    {42,-84, 42, 0, 0, 0, 0, 0},
    {-210, 630, -630, 210, 0, 0, 0, 0},
    {840, -3360, 5040, -3360, 840, 0, 0, 0},
    {-2520, 12600, -25200, 25200, -12600, 2520, 0, 0},
    {5040, -30240, 75600, -100800, 75600, -30240, 5040, 0},
    {-5040, 35280, -105840, 176400, -176400, 105840, -35280, 5040}
  };


  // Limit[D[f1[t], {t, 0}], t -> 1]: y[7]
  // Limit[D[f1[t], {t, 1}], t -> 1]: -7 y[6] + 7 y[7]
  // Limit[D[f1[t], {t, 2}], t -> 1]: 42 y[5] - 84 y[6] + 42 y[7]
  // Limit[D[f1[t], {t, 3}], t -> 1]: -210 y[4] + 630 y[5] - 630 y[6] + 210 y[7]
  // Limit[D[f1[t], {t, 4}], t -> 1]: 840 y[3] - 3360 y[4] + 5040 y[5] - 3360 y[6] + 840 y[7]
  // Limit[D[f1[t], {t, 5}], t -> 1]: -2520 y[2] + 12600 y[3] - 25200 y[4] + 25200 y[5] - 12600 y[6] + 2520 y[7]
  // Limit[D[f1[t], {t, 6}], t -> 1]: 5040 y[1] - 30240 y[2] + 75600 y[3] - 100800 y[4] + 75600 y[5] - 30240 y[6] + 5040 y[7]
  // Limit[D[f1[t], {t, 7}], t -> 1]: -5040 y[0] + 35280 y[1] - 105840 y[2] + 176400 y[3] - 176400 y[4] + 105840 y[5] - 35280 y[6] + 5040 y[7]
  static constexpr double derivativeEnd[8][8] = {
    {0 , 0, 0, 0, 0, 0, 0, 1},
    {0, 0, 0, 0, 0, 0, -7, 7},
    {0, 0, 0, 0, 0, 42,-84, 42},
    {0, 0, 0, 0, -210, 630, -630, 210},
    {0, 0, 0, 840, -3360, 5040, -3360, 840},
    {0, 0, -2520, 12600, -25200, 25200, -12600, 2520},
    {0, 5040, -30240, 75600, -100800, 75600, -30240, 5040},
    {-5040, 35280, -105840, 176400, -176400, 105840, -35280, 5040}
  };


private:
  size_t m_dimension;
  std::vector<double> m_pieceDurations;
  size_t m_numPieces;
  const size_t m_numVars;

  Matrix m_A;
  Vector m_lbA;
  Vector m_ubA;
  Vector m_lb;
  Vector m_ub;

  std::vector<std::string> m_info;
};

#endif
