#include <iostream>
#include <fstream>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <qpOASES.hpp>

// using namespace Eigen;

// qpOASES assumes row-major!
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
typedef Eigen::VectorXd Vector;

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;

// See https://stackoverflow.com/questions/35227131/eigen-check-if-matrix-is-positive-semi-definite
bool isPositiveDefinite(const Matrix& A)
{
  Eigen::LLT<Matrix> lltOfA(A); // compute the Cholesky decomposition of A
  // std::cout << lltOfA.info() << std::endl;
  if(lltOfA.info() == Eigen::NumericalIssue) {
    // std::cout << "Oo";
    return false;
  }
  return true;
}

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

    std::cout << H_1d_curve << std::endl;

    // H is just blockdiagonal of H_1d_curve
    for (size_t i = 0; i < m_numVars; i+=8) {
      m_H.block(i, i, 8, 8) += H_1d_curve;
    }

    // std::cout << m_H;
  }

  // Expand[(Limit[D[f1[t], {t, 0}], t -> 1] - X)^2]
  // X^2 - 2 X y[7] + y[7]^2
  void endCloseTo(
    double lambda,
    const Vector& value)
  {
    for (size_t d = 0; d < m_dimension; ++d) {
      size_t idx = column(m_numPieces - 1, d) + 7;
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

Matrix ObjectiveBuilder::Qvel;
Matrix ObjectiveBuilder::Qacc;
Matrix ObjectiveBuilder::Qjerk;
Matrix ObjectiveBuilder::Qsnap;
Matrix ObjectiveBuilder::B;

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

constexpr double ConstraintBuilder::derivativeBeginning[8][8];
constexpr double ConstraintBuilder::derivativeEnd[8][8];

int main()
{
  // Matrix B: converts between Bezier and polynomial trajectories

  ObjectiveBuilder::Init();




  // Matrix BI = B.inverse();
  // std::cout << BI << std::endl;

  // Matrix Q: cost function (e.g., snap: D[p[t], {t, 4}]^2)





  // our previous paper uses: 1 * vel + 0 * acc + 5e-3 * jerk

  // Optimization matrix cost is (B^T Q B)

  // Matrix H = B.transpose() * Qsnap * B;
  // Vector g(8);
  // g.setZero();

  // std::cout << H << std::endl;

  // // y are our control points (decision variable)
  // Vector y(8);

  // // lower and upper bound for decision variables
  // Vector lb(8);
  // lb.setConstant(-100);
  // Vector ub(8);
  // ub.setConstant(100);

  // // constraint matrix A
  // const int numConstraints = 4;
  // const int numVariables = 8;
  // Matrix A(numConstraints, 8);
  // Vector lbA(numConstraints);
  // Vector ubA(numConstraints);

  // // const float e = 1e-2;

  // A.setZero();
  // lbA.setZero();
  // ubA.setZero();
  // // constrain start (position)
  // A(0, 0) = 1;
  // lbA(0) = -5;
  // ubA(0) = -5;

  // // constrain end (position)
  // A(1, 7) = 1;
  // lbA(1) = 5;
  // ubA(1) = 5;

  // // constrain start (velocity)
  // A(2, 0) = -7;
  // A(2, 1) =  7;
  // lbA(2) = 0;
  // ubA(2) = 0;

  // std::cout << A << std::endl;
  // std::cout << lbA << std::endl;
  // std::cout << ubA << std::endl;

  // // constrain end (velocity)
  // A(3, 6) = -7;
  // A(3, 7) =  7;
  // lbA(3) = 0;
  // ubA(3) = 0;



  const size_t dimension = 2;
  Matrix waypoints(4, dimension);
  waypoints << 0, 0,
               1, 2,
               2, 0,
               4, 3;

  // const size_t dimension = 1;
  // Matrix waypoints(3, dimension);
  // waypoints << -5,
  //               5,
  //              10;

  const size_t numPieces = waypoints.rows() - 1;
  const size_t continuity = 2; // up to acc

  // our previous paper uses: 1 * vel + 0 * acc + 5e-3 * jerk
  ObjectiveBuilder ob(dimension, numPieces);
  ob.minDerivativeSquared(1, 0, 5e-3, 0);
  ob.endCloseTo(100, waypoints.row(3));

  // y are our control points (decision variable)
  // Vector y(numVars);
  Matrix y(dimension, 8 * numPieces);

  // lower and upper bound for decision variables (i.e., workspace)
  const size_t numVars = dimension * 8 * numPieces;
  Vector lb(numVars);
  lb.setConstant(-20);
  Vector ub(numVars);
  ub.setConstant(20);

  Vector zeroVec(dimension);
  zeroVec.setZero();

  // constraint matrix A
  ConstraintBuilder cb(dimension, numPieces);

  cb.addConstraintBeginning(0, 0, waypoints.row(0)); // Position
  // cb.addConstraintEnd(2, 0, waypoints.row(3)); // Position

  Vector normal(dimension);
  normal << 0, 1;
  cb.addHyperplane(1, normal, -2);
  // for (size_t i = 0; i < 1000; ++i) {
  //   cb.addHyperplane(1, normal, -2 + i * 0.1);
  // }

  // test endCloseTo objective
  cb.addHyperplane(2, normal, -1);


  for (size_t i = 0; i < numPieces; ++i) {
    // cb.addPositionConstraintBeginning(i, waypoints.row(i));
    // if (i == numPieces - 1)
    //   cb.addPositionConstraintEnd(i, waypoints.row(i+1));

    if (i < numPieces - 1) {
      for (size_t c = 0; c <= continuity; ++c) {
        cb.addContinuity(i, c);
      }
    }

    // cb.addVelocityConstraintBeginning(i, zeroVec);
    // cb.addVelocityConstraintEnd(i, zeroVec);
  }

  // std::cout << "H: " << std::endl;
  // std::cout << H << std::endl;

  // std::cout << "A: " << std::endl;
  // std::cout << cb.A() << std::endl;

  // std::cout << "lbA: " << std::endl;
  // std::cout << cb.lbA() << std::endl;

  // std::cout << "ubA: " << std::endl;
  // std::cout << cb.ubA() << std::endl;


#if 0
  const int numConstraints = dimension * ((numPieces - 1) * 2 + 2);

  Matrix A(numConstraints, numVariables);
  Vector lbA(numConstraints);
  Vector ubA(numConstraints);

  // const float e = 1e-2;

  A.setZero();
  lbA.setZero();
  ubA.setZero();
  // constrain start (position)
  A(0, 0) = 1;
  lbA(0) = -5;
  ubA(0) = -5;

  // constrain end (position)
  A(1, 7) = 1;
  lbA(1) = 5;
  ubA(1) = 5;

  // constrain start (velocity)
  A(2, 0) = -7;
  A(2, 1) =  7;
  lbA(2) = 0;
  ubA(2) = 0;

  std::cout << A << std::endl;
  std::cout << lbA << std::endl;
  std::cout << ubA << std::endl;

  // constrain end (velocity)
  A(3, 6) = -7;
  A(3, 7) =  7;
  lbA(3) = 0;
  ubA(3) = 0;
#endif


  const size_t numConstraints = cb.A().rows();


  using namespace qpOASES;

  QProblem qp(numVars, numConstraints);//, HST_SEMIDEF);

  Options options;
  qp.setOptions(options);

  // The  integer  argument nWSR specifies  the  maximum  number  of  working  set
  // recalculations to be performed during the initial homotopy (on output it contains the number
  // of  working  set  recalculations  actually  performed!)
  int_t nWSR = 10000;

  auto t0 = Time::now();
  returnValue status = qp.init(ob.H().data(), ob.g().data(), cb.A().data(), lb.data(), ub.data(), cb.lbA().data(), cb.ubA().data(), nWSR);
  auto t1 = Time::now();
  fsec fs = t1 - t0;
  ms d = std::chrono::duration_cast<ms>(fs);
  std::cout << "optimization time: " << d.count() << " ms" << std::endl;

  qp.getPrimalSolution(y.data());

  std::cout << "status: " << status << std::endl;
  std::cout << "objective: " << qp.getObjVal() << std::endl;
  std::cout << "y: " << y << std::endl;

  std::ofstream output("curve.csv");
  output << "duration,p0x,p0y,p1x,p1y,p2x,p2y,p3x,p3y,p4x,p4y,p5x,p5y,p6x,p6y,p7x,p7y" << std::endl;
  for (size_t i = 0; i < numPieces; ++i) {
    output << "1.0";
    for (size_t j = 0; j < 8; ++j) {
      output << "," << y(0, i*8+j);
      output << "," << y(1, i*8+j);
    }
    output << std::endl;
  }

#if 0
  MatrixXd H(2, 2);
  H << 1.0, 0.0,
       0.0, 0.5;
  std::cout << H;

  USING_NAMESPACE_QPOASES

  /* Setup data of first QP. */
  // real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
  real_t A[1*2] = { 1.0, 1.0 };
  real_t g[2] = { 1.5, 1.0 };
  real_t lb[2] = { 0.5, -2.0 };
  real_t ub[2] = { 5.0, 2.0 };
  real_t lbA[1] = { -1.0 };
  real_t ubA[1] = { 2.0 };

  /* Setup data of second QP. */
  real_t g_new[2] = { 1.0, 1.5 };
  real_t lb_new[2] = { 0.0, -1.0 };
  real_t ub_new[2] = { 5.0, -0.5 };
  real_t lbA_new[1] = { -2.0 };
  real_t ubA_new[1] = { 1.0 };


  /* Setting up QProblem object. */
  QProblem example( 2,1 );

  Options options;
  example.setOptions( options );

  /* Solve first QP. */
  int_t nWSR = 10;
  example.init( H.data(),g,A,lb,ub,lbA,ubA, nWSR );

  /* Get and print solution of first QP. */
  real_t xOpt[2];
  real_t yOpt[2+1];
  example.getPrimalSolution( xOpt );
  example.getDualSolution( yOpt );
  printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
      xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

  /* Solve second QP. */
  nWSR = 10;
  example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

  /* Get and print solution of second QP. */
  example.getPrimalSolution( xOpt );
  example.getDualSolution( yOpt );
  printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
      xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

  example.printOptions();
  /*example.printProperties();*/

  /*getGlobalMessageHandler()->listAllMessages();*/
#endif
  return 0;
}
