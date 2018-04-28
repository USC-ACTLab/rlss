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
  void addPositionConstraintBeginning(
    size_t piece,
    const Vector& position)
  {
    assert(position.size() == m_dimension);
    size_t idx = addConstraints(m_dimension);

    for (size_t d = 0; d < m_dimension; ++d) {
      m_A(idx + d, column(piece, d) + 0) = 1;
      m_lbA(idx + d) = position(d);
      m_ubA(idx + d) = position(d);
    }
  }

  // adds a constraint for a fixed position at the end of the specified piece
  void addPositionConstraintEnd(
    size_t piece,
    const Vector& position)
  {
    assert(position.size() == m_dimension);
    size_t idx = addConstraints(m_dimension);

    for (size_t d = 0; d < m_dimension; ++d) {
      m_A(idx + d, column(piece, d) + 7) = 1;
      m_lbA(idx + d) = position(d);
      m_ubA(idx + d) = position(d);
    }
  }

  // adds a constraint for a fixed velocity at the beginning of the specified piece
  void addVelocityConstraintBeginning(
    size_t piece,
    const Vector& velocity)
  {
    assert(velocity.size() == m_dimension);
    size_t idx = addConstraints(m_dimension);

    for (size_t d = 0; d < m_dimension; ++d) {
      m_A(idx + d, column(piece, d) + 0) = -7;
      m_A(idx + d, column(piece, d) + 1) = 7;
      m_lbA(idx + d) = velocity(d);
      m_ubA(idx + d) = velocity(d);
    }
  }

  // adds a constraint for a fixed velocity at the end of the specified piece
  void addVelocityConstraintEnd(
    size_t piece,
    const Vector& velocity)
  {
    assert(velocity.size() == m_dimension);
    size_t idx = addConstraints(m_dimension);

    for (size_t d = 0; d < m_dimension; ++d) {
      m_A(idx + d, column(piece, d) + 6) = -7;
      m_A(idx + d, column(piece, d) + 7) = 7;
      m_lbA(idx + d) = velocity(d);
      m_ubA(idx + d) = velocity(d);
    }
  }

  // ensures that position is identical for firstPiece and firstPiece+1
  void addContPosition(size_t firstPiece)
  {
    assert(firstPiece < m_numPieces - 1);
    size_t idx = addConstraints(m_dimension);

    for (size_t d = 0; d < m_dimension; ++d) {
      // position of end of first piece
      m_A(idx + d, column(firstPiece, d) + 7) = 1;
      // negative position of beginning of next piece
      m_A(idx + d, column(firstPiece+1, d) + 0) = -1;

      m_lbA(idx + d) = 0;
      m_ubA(idx + d) = 0;
    }
  }

  // ensures that velocity is identical for firstPiece and firstPiece+1
  void addContVelocity(size_t firstPiece)
  {
    assert(firstPiece < m_numPieces - 1);
    size_t idx = addConstraints(m_dimension);

    for (size_t d = 0; d < m_dimension; ++d) {
      // velocity of end of first piece
      m_A(idx + d, column(firstPiece, d) + 6) = -7;
      m_A(idx + d, column(firstPiece, d) + 7) = 7;
      // negative velocity of beginning of next piece
      m_A(idx + d, column(firstPiece+1, d) + 0) = 7;
      m_A(idx + d, column(firstPiece+1, d) + 1) = -7;

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
  size_t m_dimension;
  size_t m_numPieces;
  const size_t m_numVars;

  Matrix m_A;
  Vector m_lbA;
  Vector m_ubA;
};

int main()
{
  // Matrix B: converts between Bezier and polynomial trajectories
  Matrix B(8, 8);
  B <<   1,    0,    0,    0,    0,   0,  0, 0,
        -7,    7,    0,    0,    0,   0,  0, 0,
        21,  -42,   21,    0,    0,   0,  0, 0,
       -35,  105, -105,   35,    0,   0,  0, 0,
        35, -140,  210, -140,   35,   0,  0, 0,
       -21,  105, -210,  210, -105,  21,  0, 0,
         7,  -42,  105, -140,  105, -42,  7, 0,
        -1,    7,  -21,   35,  -35,  21, -7, 1;

  // Matrix BI = B.inverse();
  // std::cout << BI << std::endl;

  // Matrix Q: cost function (e.g., snap: D[p[t], {t, 4}]^2)
  Matrix Qsnap(8, 8);
  Qsnap <<    0,    0,    0,    0,    0,     0,     0,      0,
              0,    0,    0,    0,    0,     0,     0,      0,
              0,    0,    0,    0,    0,     0,     0,      0,
              0,    0,    0,    0,    0,     0,     0,      0,
              0,    0,    0,    0,  576,  1440,  2880,   5040,
              0,    0,    0,    0, 1440,  4800, 10800,  20160,
              0,    0,    0,    0, 2880, 10800, 25920,  50400,
              0,    0,    0,    0, 5040, 20160, 50400, 100800;

  // TODO: not yet verified
  Matrix Qvel(8, 8);
  Qvel <<    0,    0,     0,     0,     0,     0,     0,      0,
              0,    1,     1,     1,     1,     1,     1,      1,
              0,    1,  4/3.,  3/2.,16/10., 10/6.,24/14.,   7/4.,
              0,    1,  3/2.,  9/5.,     2,30/14.,  9/4.,  14/6.,
              0,    1,16/10.,     2, 16/7.,  5/2., 16/6., 28/10.,
              0,    1, 10/6.,30/14.,  5/2., 25/9.,     3, 70/22.,
              0,    1,24/14.,  9/4., 16/6.,     3,36/11.,   7/2.,
              0,    1,  7/4., 14/6.,28/10.,70/22.,  7/2., 49/13.;

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

  Matrix H_1d_curve = B.transpose() * (1 * Qvel + 1e-5 * Qsnap) * B;

  // H is just blockdiagonal of H_1d_curve
  const size_t numVars = dimension * 8 * numPieces;
  Matrix H(numVars, numVars);
  H.setZero();
  for (size_t i = 0; i < numVars; i+=8) {
    H.block(i, i, 8, 8) = H_1d_curve;
  }
  Vector g(numVars);
  g.setZero();

  // y are our control points (decision variable)
  // Vector y(numVars);
  Matrix y(dimension, 8 * numPieces);

  // lower and upper bound for decision variables (i.e., workspace)
  Vector lb(numVars);
  lb.setConstant(-20);
  Vector ub(numVars);
  ub.setConstant(20);

  Vector zeroVec(dimension);
  zeroVec.setZero();

  // constraint matrix A
  ConstraintBuilder cb(dimension, numPieces);

  cb.addPositionConstraintBeginning(0, waypoints.row(0));
  cb.addPositionConstraintEnd(2, waypoints.row(3));

  Vector normal(dimension);
  normal << 0, 1;
  cb.addHyperplane(1, normal, -2);
  for (size_t i = 0; i < 1000; ++i) {
    cb.addHyperplane(1, normal, -2 + i * 0.1);
  }


  for (size_t i = 0; i < numPieces; ++i) {
    // cb.addPositionConstraintBeginning(i, waypoints.row(i));
    // if (i == numPieces - 1)
    //   cb.addPositionConstraintEnd(i, waypoints.row(i+1));

    if (i < numPieces - 1) {
      cb.addContPosition(i);
      cb.addContVelocity(i);
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
  returnValue status = qp.init(H.data(), g.data(), cb.A().data(), lb.data(), ub.data(), cb.lbA().data(), cb.ubA().data(), nWSR);
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
