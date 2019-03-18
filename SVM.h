#include "PointCloud.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include "utility.h"


#include "cvxgen/64robot8obstacle/solver.h"

namespace _64_8_svm_solver {
  Vars vars;
  Params params;
  Workspace work;
  Settings settings;
}

#include "cvxgen/16robot8obstacle/solver.h"

namespace _16_8_svm_solver {
  Vars vars;
  Params params;
  Workspace work;
  Settings settings;
}


#include "lib/libsvm/svm.h"
/*
* creates a svm node with given index and value
*/
svm_node createSvmNode(int idx, double val) {
        svm_node svx;    
        svx.index = idx;
        svx.value = val;
        return svx;
}

/*
* Takes x,y,z points and creates an svm node. Adding -1
* at the end to specify termination
*/
svm_node* createSNodeArray(double px, double py, double pz) {
    svm_node* arr = new svm_node[4];
    arr[0] = createSvmNode(1,px);
    arr[1] = createSvmNode(2,py);
    arr[2] = createSvmNode(3,pz);
    arr[3] = createSvmNode(-1,0);
    return arr;
}
/*
* Utility function to create param for svm 
*/
svm_parameter createSVMParam() {
  svm_parameter param;
  param.svm_type = C_SVC;
  param.kernel_type = LINEAR;
  param.degree = 3;
  param.cache_size = 64;
  param.eps = 0.01;
  param.C = 10000;
  param.nr_weight= 0;
  return param;
}


/*
  *Function that takes the model, a (0,0,0) vector reference (nv)
  *and a double reference (rho), and fills these parameters with 
  *appropriate values after applying linear kernel method.  
*/
void getHyperplanes(const svm_model* mx, std::vector<double> &nv, double &rho) {
	const double * const *sv_coef = mx->sv_coef;
	const svm_node * const *SV = mx->SV;
  rho = *(mx->rho);
  double cur_sv;
   for (int i=0;i<mx->l;++i) {
       const svm_node *p = SV[i];
       std::cout << "SV Coef : " << sv_coef[0][i] << std::endl;
       cur_sv = sv_coef[0][i];
       int idx = 0;
        while(p->index != -1)
        {
            printf("%d:%.8g ",p->index,p->value);
            nv[idx]+= cur_sv*p->value;
            idx++;
            p++;
        }
        printf("\n");
   }
}

using std::cout;
using std::endl;
using std::vector;

namespace ACT {
/*
* Seperate robot_points from obstacle_points.
* robot_points will be in the negative side
*
* 16 robot points, 8 obstacle points
*/
template<class T>
Eigen::Hyperplane<T, 3U> _16_8_seperate(
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >& robot_points,
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >& obstacle_points)
{
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;
  using Hyperplane = Eigen::Hyperplane<T, 3U>;


  const unsigned int number_of_constraints = 24;
  const unsigned int robot_constraints = 16;
  const unsigned int obstacle_constraints = 8;

  for(unsigned int i = 0; i < robot_points.size(); i++) {
    for(unsigned int d = 0; d < 3U; d++) {
      _16_8_svm_solver::params.A[number_of_constraints * d + i] = robot_points[i](d);
    }
    _16_8_svm_solver::params.A[number_of_constraints * 3 + i] = 1;
  }

  for(unsigned int i = 0; i < obstacle_points.size(); i++) {
    for(unsigned int d = 0; d < 3U; d++) {
      _16_8_svm_solver::params.A[number_of_constraints * d + robot_constraints + i]
              = -obstacle_points[i](d);
    }
    _16_8_svm_solver::params.A[number_of_constraints * 3 + robot_constraints + i] = -1;
  }

  _16_8_svm_solver::params.H[0] = 1;
  _16_8_svm_solver::params.H[1] = 1;
  _16_8_svm_solver::params.H[2] = 1;
  _16_8_svm_solver::params.H[3] = 0;

  _16_8_svm_solver::set_defaults();
  _16_8_svm_solver::setup_indexing();

  // _16_8_svm_solver::settings.eps;
  //_16_8_svm_solver::settings.resid_tol;
  //_16_8_svm_solver::settings.max_iters;
  _16_8_svm_solver::settings.verbose = 0;
  //_16_8_svm_solver::settings.kkt_reg;
  //_16_8_svm_solver::settings.refine_steps;
  //_16_8_svm_solver::settings.verbose_refinement;



  long num_iters = _16_8_svm_solver::solve();


  //cout << "# SVM _16_8 Converged: " << _16_8_svm_solver::work.converged << endl;


  VectorDIM normal(_16_8_svm_solver::vars.w[0],
     _16_8_svm_solver::vars.w[1], _16_8_svm_solver::vars.w[2]);
   T norm = normal.norm();
   normal /= norm;
  
  std::cout << "Orig " << normal(0) << " " << normal(1) << " " << normal(2) << "Dist:" << _16_8_svm_solver::vars.w[3]/norm << std::endl;
  return Hyperplane(normal, _16_8_svm_solver::vars.w[3] / norm);
}



/*
* Seperate robot_points from obstacle_points.
* robot_points will be in the negative side
*/
template<class T>
Eigen::Hyperplane<T, 3U> _64_8_seperate(
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >& robot_points,
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >& obstacle_points)
{
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;
  using Hyperplane = Eigen::Hyperplane<T, 3U>;


  const unsigned int number_of_constraints = 72;
  const unsigned int robot_constraints = 64;
  const unsigned int obstacle_constraints = 8;

  for(unsigned int i = 0; i < robot_points.size(); i++) {
    for(unsigned int d = 0; d < 3U; d++) {
      _64_8_svm_solver::params.A[number_of_constraints * d + i] = robot_points[i](d);
    }
    _64_8_svm_solver::params.A[number_of_constraints * 3 + i] = 1;
  }

  for(unsigned int i = 0; i < obstacle_points.size(); i++) {
    for(unsigned int d = 0; d < 3U; d++) {
      _64_8_svm_solver::params.A[number_of_constraints * d + robot_constraints + i]
              = -obstacle_points[i](d);
    }
    _64_8_svm_solver::params.A[number_of_constraints * 3 + robot_constraints + i] = -1;
  }

  _64_8_svm_solver::params.H[0] = 1;
  _64_8_svm_solver::params.H[1] = 1;
  _64_8_svm_solver::params.H[2] = 1;
  _64_8_svm_solver::params.H[3] = 0;

  _64_8_svm_solver::set_defaults();
  _64_8_svm_solver::setup_indexing();

  // _64_8_svm_solver::settings.eps;
  //_64_8_svm_solver::settings.resid_tol;
  //_64_8_svm_solver::settings.max_iters;
  _64_8_svm_solver::settings.verbose = 0;
  //_64_8_svm_solver::settings.kkt_reg;
  //_64_8_svm_solver::settings.refine_steps;
  //_64_8_svm_solver::settings.verbose_refinement;



  long num_iters = _64_8_svm_solver::solve();

  //cout << "# SVM _64_8 Converged: " << _64_8_svm_solver::work.converged << endl;


  VectorDIM normal(_64_8_svm_solver::vars.w[0],
     _64_8_svm_solver::vars.w[1], _64_8_svm_solver::vars.w[2]);
   T norm = normal.norm();
   normal /= norm;
  std::cout << "Orig " << normal(0) << " " << normal(1) << " " << normal(2) << "Dist:" << _64_8_svm_solver::vars.w[3]/norm << std::endl;
  return Hyperplane(normal, _64_8_svm_solver::vars.w[3] / norm);
}


/*
* Seperate robot_points from 
* obstacle_points in generic case
*/
template<class T>
Eigen::Hyperplane<T, 3U> _generic_seperate(
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >& robot_points,
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >& obstacle_points)
{
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;
  using Hyperplane = Eigen::Hyperplane<T, 3U>;

  //create problem with total size
  int problem_size = robot_points.size() + obstacle_points.size();
  svm_problem prob;
  prob.l = problem_size;
  //dynamicly allocate array for it
  double* y_vect = new double[problem_size];
  svm_node** x_vect = new svm_node*[problem_size];

  //fill arrays with points
  for(unsigned int i = 0; i < robot_points.size(); i++) {
    x_vect[i]= createSNodeArray(robot_points[i](0),robot_points[i](1),robot_points[i](2));
    y_vect[i]= 1;  
  }

  for(unsigned int i = 0; i < obstacle_points.size(); i++) {
    x_vect[robot_points.size()+i]= createSNodeArray(obstacle_points[i](0),obstacle_points[i](1),obstacle_points[i](2));
    y_vect[robot_points.size()+i] = 2;
  }
  prob.x = x_vect;
  prob.y = y_vect;

  //create svm param
  svm_parameter param;
  param = createSVMParam();

  //check if there is any errors
  const char *error_msg;
  error_msg = svm_check_parameter(&prob,&param);
	if(error_msg)
	{
		fprintf(stderr,"ERROR: %s\n",error_msg);
		exit(1);
	}

  //train model
  svm_model* model;
  model = svm_train(&prob,&param);
  
  //extract normal_vect and hyperplanes, writes it to argument
  std::vector<double> normal_vect{0,0,0};
  double rho = 0;
  getHyperplanes(model,normal_vect,rho);

  for (int i=0;i<3;i++) {
    std::cout<<normal_vect[i] << " " ;
  }
  std::cout << std::endl;
  std::cout << "Rho " << rho << std::endl;

  //normalize vector and return it
  VectorDIM normal(-normal_vect[0],-normal_vect[1],-normal_vect[2]);
  T norm = normal.norm();
  normal /= norm;
  std::cout << "MY " << normal(0) << " " << normal(1) << " " << normal(2) << "Dist:" << rho/norm << std::endl;
  
  //deallocate memory
  for (int i=0;i<problem_size;i++) {
    delete[] x_vect[i]; //deallocate the array that each ptr refers to
  }
  delete[] x_vect; //deallocate the ptr array
  delete[] y_vect; //deallocate y's
  return Hyperplane(normal,rho/norm);  
}

/*
* Get corner points of a box fill the result to vect
*/
template<class T>
void getCornerPoints(const Eigen::AlignedBox<T, 3U>& box,
  vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>>>& vect)
{
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;

  const VectorDIM& min = box.min();
  const VectorDIM& max = box.max();


  vect.push_back(VectorDIM(min(0), min(1), min(2)));
  vect.push_back(VectorDIM(min(0), min(1), max(2)));
  vect.push_back(VectorDIM(min(0), max(1), min(2)));
  vect.push_back(VectorDIM(min(0), max(1), max(2)));
  vect.push_back(VectorDIM(max(0), min(1), min(2)));
  vect.push_back(VectorDIM(max(0), min(1), max(2)));
  vect.push_back(VectorDIM(max(0), max(1), min(2)));
  vect.push_back(VectorDIM(max(0), max(1), max(2)));
}

/*
* Return corner points of aligned box
*/
template<class T>
vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >
  getCornerPoints(const Eigen::AlignedBox<T, 3U>& box)
{
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;

  const VectorDIM& min = box.min();
  const VectorDIM& max = box.max();

  vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> res;

  getCornerPoints(box, res);

  return res;
}



/*
* Get corner points of all boxes fill the results to vect
*/
template<class T>
void getCornerPoints(const vector<Eigen::AlignedBox<T, 3U>>& boxes,
  vector<Eigen::Matrix<T,3U,1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>>>& vect) {

  using VectorDIM = Eigen::Matrix<T, 3U, 1>;


  for(const auto& box: boxes) {
    getCornerPoints(box, vect);
  }
}

/*
* Return corner points of all boxes
*/
template<class T>
vector<Eigen::Matrix<T, 3U, 1>, Eigen::aligned_allocator<Eigen::Matrix<T, 3U, 1>> >
  getCornerPoints(const vector<Eigen::AlignedBox<T, 3U>>& boxes)
{
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;

  vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> res;

  getCornerPoints(boxes, res);

  return res;
}

/*
* for each obstacle o, find the svm hyperplane that seperates o from all of the elements of
* robot_boxes
*/
template<class T>
vector<Eigen::Hyperplane<T, 3U> > svm3d(const vector<Eigen::AlignedBox<T, 3U>>& robot_boxes,
                                       const vector<Eigen::AlignedBox<T, 3U>>& obstacles)
{

  using Hyperplane = Eigen::Hyperplane<T, 3U>;
  using AlignedBox = typename PointCloud<T, 3U>::AlignedBox;
  using VectorDIM = Eigen::Matrix<T, 3U, 1>;


  vector<Hyperplane> hyperplanes;

  /*
  * Add robot points to a vector
  */
  vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> robot_points = getCornerPoints<T>(robot_boxes);

  for(auto& o: obstacles) {
    vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> obstacle_points = getCornerPoints<T>(o);
    Hyperplane hp;
    //SEPERATING
    if(robot_boxes.size() == 8) {
      // continuous case with 8 points
      hp = _64_8_seperate<T>(robot_points, obstacle_points);
      hp  = _generic_seperate<T>(robot_points,obstacle_points);
      exit(0);
    } else if(robot_boxes.size() == 2) {
      // discrete case
      hp = _16_8_seperate<T>(robot_points, obstacle_points);
      hp  = _generic_seperate<T>(robot_points,obstacle_points);
      exit(0);
    } else {
      // generic case
      hp  = _generic_seperate<T>(robot_points,obstacle_points);

      cout << "[svm3d] NOT IMPLEMENTED: " << robot_points.size() << " " << obstacle_points.size() << endl;
      exit(0);
    }

    // SHIFTING

    // first shift back to obstacle
    double min_dist = std::numeric_limits<double>::infinity();
    const vector<VectorDIM, Eigen::aligned_allocator<VectorDIM> > vec{o.min(), o.max()};

    for(int i = 0; i < 2; i++) {
      for(int j = 0; j < 2; j++) {
        for(int k = 0; k < 2; k++) {
          VectorDIM corner(vec[i](0), vec[j](1), vec[k](2));
          min_dist = std::min(min_dist, hp.signedDistance(corner));
        }
      }
    }
    hp.offset() -= min_dist;

    // now shift according to radius
    shiftHyperplane(robot_boxes[0], hp);

    hyperplanes.push_back(hp);
  }

  return hyperplanes;
}

}
