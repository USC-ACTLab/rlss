#ifndef QP_OPTIMIZE_H
#define QP_OPTIMIZE_H

#include <Eigen/Core>
#include <Eigen/Dense>

// qpOASES assumes row-major!
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
typedef Eigen::VectorXd Vector;

class ObjectiveBuilder
{
public:
  ObjectiveBuilder(
    size_t dimension,
    size_t numPieces)
    : m_dimension(dimension)
    , m_numPieces(numPieces)
    , m_numVars(dimension * 8 * numPieces)
    , m_H(m_numVars, m_numVars)
    , m_g(m_numVars)
  {
    m_H.setZero();
    m_g.setZero();
  }

  static void Init()
  {
    Qvel.resize(8,8);
    Qvel <<   0, 0, 0, 0, 0, 0, 0, 0,
              0, 1, 1, 1, 1, 1, 1, 1,
              0, 1, 4/3., 3/2., 8/5., 5/3., 12/7., 7/4.,
              0, 1, 3/2., 9/5., 2, 15/7., 9/4., 7/3.,
              0, 1, 8/5., 2, 16/7., 5/2., 8/3., 14/5.,
              0, 1, 5/3., 15/7., 5/2., 25/9., 3, 35/11.,
              0, 1, 12/7., 9/4., 8/3., 3, 36/11., 7/2.,
              0, 1, 7/4., 7/3., 14/5., 35/11., 7/2., 49/13.;

    Qacc.resize(8,8);
    Qacc <<   0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 4, 6, 8, 10, 12, 14,
              0, 0, 6, 12, 18, 24, 30, 36,
              0, 0, 8, 18, (144/5.), 40, (360/7.), 63,
              0, 0, 10, 24, 40, (400/7.), 75, (280/3.),
              0, 0, 12, 30, (360/7.), 75, 100, 126,
              0, 0, 14, 36, 63, (280/3.), 126, (1764/11.);

    Qjerk.resize(8,8);
    Qjerk <<  0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 0, 0, 0, 0, 0,
              0, 0, 0, 36, 72, 120, 180, 252,
              0, 0, 0, 72, 192, 360, 576, 840,
              0, 0, 0, 120, 360, 720, 1200, 1800,
              0, 0, 0, 180, 576, 1200, 14400/7., 3150,
              0, 0, 0, 252, 840, 1800, 3150, 4900;

    Qsnap.resize(8,8);
    Qsnap <<    0,    0,    0,    0,    0,     0,     0,      0,
                0,    0,    0,    0,    0,     0,     0,      0,
                0,    0,    0,    0,    0,     0,     0,      0,
                0,    0,    0,    0,    0,     0,     0,      0,
                0,    0,    0,    0,  576,  1440,  2880,   5040,
                0,    0,    0,    0, 1440,  4800, 10800,  20160,
                0,    0,    0,    0, 2880, 10800, 25920,  50400,
                0,    0,    0,    0, 5040, 20160, 50400, 100800;

    B.resize(8,8);
    B <<   1,    0,    0,    0,    0,   0,  0, 0,
          -7,    7,    0,    0,    0,   0,  0, 0,
          21,  -42,   21,    0,    0,   0,  0, 0,
         -35,  105, -105,   35,    0,   0,  0, 0,
          35, -140,  210, -140,   35,   0,  0, 0,
         -21,  105, -210,  210, -105,  21,  0, 0,
           7,  -42,  105, -140,  105, -42,  7, 0,
          -1,    7,  -21,   35,  -35,  21, -7, 1;
  }

  void minDerivativeSquared(
    double lambda_vel, double lambda_acc, double lambda_jerk, double lambda_snap)
  {
    Matrix Q = lambda_vel * Qvel;// + lambda_acc * Qacc + lambda_jerk * Qjerk + lambda_snap * Qsnap;

    Matrix H_1d_curve = B.transpose() * Q * B;

    // std::cout << H_1d_curve << std::endl;

    // H is just blockdiagonal of H_1d_curve
    for (size_t i = 0; i < m_numVars; i+=8) {
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

private:
  // static Matrix Qvel = (Matrix(8, 8) <<
  //   0, 0, 0, 0, 0, 0, 0, 0,
  //   0, 1, 1, 1, 1, 1, 1, 1,
  //   0, 1, 4/3, 3/2, 8/5, 5/3, 12/7, 7/4,
  //   0, 1, 3/2, 9/5, 2, 15/7, 9/4, 7/3,
  //   0, 1, 8/5, 2, 16/7, 5/2, 8/3, 14/5,
  //   0, 1, 5/3, 15/7, 5/2, 25/9, 3, 35/11,
  //   0, 1, 12/7, 9/4, 8/3, 3, 36/11, 7/2,
  //   0, 1, 7/4, 7/3, 14/5, 35/11, 7/2, 49/13
  // ).finished();

  // static Matrix Qvel = [] {
  //   Matrix tmp(8, 8);
  //   tmp << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  //   return tmp;
  // }();


  static Matrix Qvel;
  static Matrix Qacc;
  static Matrix Qjerk;
  static Matrix Qsnap;
  static Matrix B;


private:
  size_t m_dimension;
  size_t m_numPieces;
  const size_t m_numVars;

  Matrix m_H;
  Vector m_g;
};

class ConstraintBuilder
{
public:
  ConstraintBuilder(
    size_t dimension,
    size_t numPieces)
    : m_dimension(dimension)
    , m_numPieces(numPieces)
    , m_numVars(dimension * 8 * numPieces)
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

    for (size_t d = 0; d < m_dimension; ++d) {
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(piece, d) + i) = derivativeBeginning[derivative][i];
      }
      m_lbA(idx + d) = value(d);
      m_ubA(idx + d) = value(d);
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

    for (size_t d = 0; d < m_dimension; ++d) {
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(piece, d) + i) = derivativeEnd[derivative][i];
      }
      m_lbA(idx + d) = value(d);
      m_ubA(idx + d) = value(d);
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

    for (size_t d = 0; d < m_dimension; ++d) {
      // derivative of end of first piece
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(firstPiece, d) + i) = derivativeEnd[derivative][i];
      }
      // negative derivative of beginning of next piece
      for (size_t i = 0; i < 8; ++i) {
        m_A(idx + d, column(firstPiece+1, d) + i) = -derivativeBeginning[derivative][i];
      }

      m_lbA(idx + d) = 0;
      m_ubA(idx + d) = 0;
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
    // one constraint per control point
    size_t idx = addConstraints(8);

    for (size_t cp = 0; cp < 8; ++cp) {
      for (size_t d = 0; d < m_dimension; ++d) {
        m_A(idx + cp, column(piece, d) + cp) = normal(d);
      }
      m_lbA(idx + cp) = std::numeric_limits<double>::lowest();
      m_ubA(idx + cp) = dist;
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

private:
  size_t addConstraints(size_t numConstraints)
  {
    size_t idx = m_A.rows();
    m_A.conservativeResize(idx + numConstraints, m_numVars);
    m_lbA.conservativeResize(idx + numConstraints);
    m_ubA.conservativeResize(idx + numConstraints);
    m_A.block(idx, 0, numConstraints, m_numVars).setZero();
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
  size_t m_numPieces;
  const size_t m_numVars;

  Matrix m_A;
  Vector m_lbA;
  Vector m_ubA;
};

#endif
