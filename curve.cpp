#include <vector>
#include "vectoreuc.h"
#include "curve.h"
#include <chrono>
#include "bezier_cost_integral_grad_mathematica.h"
#include <gsl/gsl_sf_hyperg.h>
#include <cassert>
#include <iostream>

using namespace std;

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::microseconds us;
typedef std::chrono::duration<float> fsec;

curve::curve(double dur, int dim): duration(dur), dimension(dim) {

}

void curve::add_cpt(vectoreuc& cpt) {
  cpts.push_back(cpt);
}

int curve::size() {
  return cpts.size();
}

vectoreuc& curve::operator[](int idx) {
  return cpts[idx];
}

void curve::set_duration(double dur) {
  duration = dur;
}

vectoreuc curve::eval(double t) {
  t = t/duration;
  if(t >= 1.0) {
    return cpts[cpts.size()-1];
  }

  vectoreuc res(dimension);

  double bern = pow((1-t), cpts.size()-1);

  for(int i=0; i<=cpts.size()-1; i++) {
    for(int q=0; q<dimension; q++) {
      res[q] += cpts[i][q] * bern;
    }
    /*t update*/
    bern *= t;
    bern /= (1-t);

    /*combination update*/
    bern *= (cpts.size()-1-i);
    bern /= (i+1);
  }

  return res;
}

int curve::comb(int n, int i) {
  if(i==0) {
    return 1;
  }

  int result = 1;
  int denom = 1;

  while(i) {
    result *= n;
    denom *= i;
    n--; i--;
  }

  return result/denom;
}

double curve::integrate(double from, double to, vector<double>& grad) {
  double result = 0;
  int d = cpts.size() - 1;
  from /= duration;
  to /= duration;


  if(grad.size() > 0)
    grad = bezier_gradient_2d_8pts(from, to, cpts);

  //auto t0 = Time::now();

  // my code
  /*int combb = comb(d, 0);
  int combcomb = combb * combb;

  double topwr2i = to;
  double topwr2iinc = to * to;
  double frompwr2i = from;
  double frompwr2iinc = from * from;

  for(int i=0; i<=cpts.size()-1; i++) {
    for(int q=0; q<dimension; q++) {
      cout << to << " " << from << " " << 2*i+1 << " " << 2*i-2*d << " " << 2*i+2 << gsl_sf_hyperg_2F1(2*i+1, 2*i-2*d, 2*i+2, to) << endl;
      result += cpts[i][q] * cpts[i][q] * combcomb *
                ((topwr2i * gsl_sf_hyperg_2F1(2*i+1, 2*i-2*d, 2*i+2, to) / (2*i+1)) -
                 (frompwr2i * gsl_sf_hyperg_2F1(2*i+1, 2*i-2*d, 2*i+2, from) / (2*i+1)));
      int jcomb = combb;
      jcomb *= (d-i);
      jcomb /= (i+1);
      double toijpwr = topwr2i * to;
      double fromijpwr = frompwr2i * from;
      for(int j=i+1; j<=cpts.size()-1; j++) {
        result += 2*cpts[i][q]*cpts[j][q]*combb * jcomb *
                  ((toijpwr * gsl_sf_hyperg_2F1(i+j+1, i+j-2*d, i+j+2, to) / (i+j+1)) -
                   (fromijpwr * gsl_sf_hyperg_2F1(i+j+1, i+j-2*d, i+j+2, from) / (i+j+1)));
        jcomb *= (d-j);
        jcomb /= (j+1);
        toijpwr *= to;
        fromijpwr *= from;
      }
      cout << "done" << endl;
    }
    combb *= (d-i);
    combb /= (i+1);
    combcomb = combb * combb;
    topwr2i *= topwr2iinc;
    frompwr2i *= frompwr2iinc;
  }
  return result;*/

  /*auto t1 = Time::now();

  double result2 = bezier_integrate_2d_8pts(from, to, cpts);

  auto t2 = Time::now();

  fsec fs1 = t1 - t0;
  fsec fs2 = t2 - t1;
  us d1 = std::chrono::duration_cast<us>(fs1);
  us d2 = std::chrono::duration_cast<us>(fs2);

  cout << ">>>> My implementation:" << result << " time:" << d1.count() << " Mathematica:" << result2 << " time:" << d2.count() << endl;*/
  return bezier_integrate_2d_8pts(from, to, cpts);
}

curve& curve::operator-=(const curve& rhs) {
  assert(cpts.size() == rhs.cpts.size());
  for(int i=0; i<size(); i++) {
    cpts[i] = cpts[i] - rhs.cpts[i];
  }

  return *this;
}
