#include <vector>
#include "vectoreuc.h"
#include "curve.h"
#include <chrono>
#include "bezier_mathematica.h"
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

/*DEPENDENT ON 2D 8PTS CAUTION!!!*/
vectoreuc curve::neval(double t, int n) {
  if(n==0) {
    vectoreuc res(2);
    res[0] = pow(1 - t/duration,7)*cpts[0][0] +
    (7*t*pow(1 - t/duration,6)*cpts[1][0])/duration +
    (21*pow(t,2)*pow(1 - t/duration,5)*cpts[2][0])/pow(duration,2) +
    (35*pow(t,3)*pow(1 - t/duration,4)*cpts[3][0])/pow(duration,3) +
    (35*pow(t,4)*pow(1 - t/duration,3)*cpts[4][0])/pow(duration,4) +
    (21*pow(t,5)*pow(1 - t/duration,2)*cpts[5][0])/pow(duration,5) +
    (7*pow(t,6)*(1 - t/duration)*cpts[6][0])/pow(duration,6) +
    (pow(t,7)*cpts[7][0])/pow(duration,7);
    res[1] = pow(1 - t/duration,7)*cpts[0][1] +
    (7*t*pow(1 - t/duration,6)*cpts[1][1])/duration +
    (21*pow(t,2)*pow(1 - t/duration,5)*cpts[2][1])/pow(duration,2) +
    (35*pow(t,3)*pow(1 - t/duration,4)*cpts[3][1])/pow(duration,3) +
    (35*pow(t,4)*pow(1 - t/duration,3)*cpts[4][1])/pow(duration,4) +
    (21*pow(t,5)*pow(1 - t/duration,2)*cpts[5][1])/pow(duration,5) +
    (7*pow(t,6)*(1 - t/duration)*cpts[6][1])/pow(duration,6) +
    (pow(t,7)*cpts[7][1])/pow(duration,7);
    return res;
  } else if(n==1) {
    vectoreuc res(2);
    res[0] = (-7*pow(1 - t/duration,6)*cpts[0][0])/duration - (42*t*pow(1 - t/duration,5)*cpts[1][0])/pow(duration,2) + (7*pow(1 - t/duration,6)*cpts[1][0])/duration -
   (105*pow(t,2)*pow(1 - t/duration,4)*cpts[2][0])/pow(duration,3) + (42*t*pow(1 - t/duration,5)*cpts[2][0])/pow(duration,2) - (140*pow(t,3)*pow(1 - t/duration,3)*cpts[3][0])/pow(duration,4) +
   (105*pow(t,2)*pow(1 - t/duration,4)*cpts[3][0])/pow(duration,3) - (105*pow(t,4)*pow(1 - t/duration,2)*cpts[4][0])/pow(duration,5) +
   (140*pow(t,3)*pow(1 - t/duration,3)*cpts[4][0])/pow(duration,4) - (42*pow(t,5)*(1 - t/duration)*cpts[5][0])/pow(duration,6) + (105*pow(t,4)*pow(1 - t/duration,2)*cpts[5][0])/pow(duration,5) -
   (7*pow(t,6)*cpts[6][0])/pow(duration,7) + (42*pow(t,5)*(1 - t/duration)*cpts[6][0])/pow(duration,6) + (7*pow(t,6)*cpts[7][0])/pow(duration,7);
   res[1] = (-7*pow(1 - t/duration,6)*cpts[0][1])/duration - (42*t*pow(1 - t/duration,5)*cpts[1][1])/pow(duration,2) + (7*pow(1 - t/duration,6)*cpts[1][1])/duration -
   (105*pow(t,2)*pow(1 - t/duration,4)*cpts[2][1])/pow(duration,3) + (42*t*pow(1 - t/duration,5)*cpts[2][1])/pow(duration,2) - (140*pow(t,3)*pow(1 - t/duration,3)*cpts[3][1])/pow(duration,4) +
   (105*pow(t,2)*pow(1 - t/duration,4)*cpts[3][1])/pow(duration,3) - (105*pow(t,4)*pow(1 - t/duration,2)*cpts[4][1])/pow(duration,5) +
   (140*pow(t,3)*pow(1 - t/duration,3)*cpts[4][1])/pow(duration,4) - (42*pow(t,5)*(1 - t/duration)*cpts[5][1])/pow(duration,6) + (105*pow(t,4)*pow(1 - t/duration,2)*cpts[5][1])/pow(duration,5) -
   (7*pow(t,6)*cpts[6][1])/pow(duration,7) + (42*pow(t,5)*(1 - t/duration)*cpts[6][1])/pow(duration,6) + (7*pow(t,6)*cpts[7][1])/pow(duration,7);
   return res;
 } else if(n==2) {
   vectoreuc res(2);
   res[0] = (42*pow(1 - t/duration,5)*cpts[0][0])/pow(duration,2) + (210*t*pow(1 - t/duration,4)*cpts[1][0])/pow(duration,3) - (84*pow(1 - t/duration,5)*cpts[1][0])/pow(duration,2) +
   (420*pow(t,2)*pow(1 - t/duration,3)*cpts[2][0])/pow(duration,4) - (420*t*pow(1 - t/duration,4)*cpts[2][0])/pow(duration,3) + (42*pow(1 - t/duration,5)*cpts[2][0])/pow(duration,2) +
   (420*pow(t,3)*pow(1 - t/duration,2)*cpts[3][0])/pow(duration,5) - (840*pow(t,2)*pow(1 - t/duration,3)*cpts[3][0])/pow(duration,4) + (210*t*pow(1 - t/duration,4)*cpts[3][0])/pow(duration,3) +
   (210*pow(t,4)*(1 - t/duration)*cpts[4][0])/pow(duration,6) - (840*pow(t,3)*pow(1 - t/duration,2)*cpts[4][0])/pow(duration,5) + (420*pow(t,2)*pow(1 - t/duration,3)*cpts[4][0])/pow(duration,4) +
   (42*pow(t,5)*cpts[5][0])/pow(duration,7) - (420*pow(t,4)*(1 - t/duration)*cpts[5][0])/pow(duration,6) + (420*pow(t,3)*pow(1 - t/duration,2)*cpts[5][0])/pow(duration,5) -
   (84*pow(t,5)*cpts[6][0])/pow(duration,7) + (210*pow(t,4)*(1 - t/duration)*cpts[6][0])/pow(duration,6) + (42*pow(t,5)*cpts[7][0])/pow(duration,7);
   res[1] = (42*pow(1 - t/duration,5)*cpts[0][1])/pow(duration,2) + (210*t*pow(1 - t/duration,4)*cpts[1][1])/pow(duration,3) - (84*pow(1 - t/duration,5)*cpts[1][1])/pow(duration,2) +
   (420*pow(t,2)*pow(1 - t/duration,3)*cpts[2][1])/pow(duration,4) - (420*t*pow(1 - t/duration,4)*cpts[2][1])/pow(duration,3) + (42*pow(1 - t/duration,5)*cpts[2][1])/pow(duration,2) +
   (420*pow(t,3)*pow(1 - t/duration,2)*cpts[3][1])/pow(duration,5) - (840*pow(t,2)*pow(1 - t/duration,3)*cpts[3][1])/pow(duration,4) + (210*t*pow(1 - t/duration,4)*cpts[3][1])/pow(duration,3) +
   (210*pow(t,4)*(1 - t/duration)*cpts[4][1])/pow(duration,6) - (840*pow(t,3)*pow(1 - t/duration,2)*cpts[4][1])/pow(duration,5) + (420*pow(t,2)*pow(1 - t/duration,3)*cpts[4][1])/pow(duration,4) +
   (42*pow(t,5)*cpts[5][1])/pow(duration,7) - (420*pow(t,4)*(1 - t/duration)*cpts[5][1])/pow(duration,6) + (420*pow(t,3)*pow(1 - t/duration,2)*cpts[5][1])/pow(duration,5) -
   (84*pow(t,5)*cpts[6][1])/pow(duration,7) + (210*pow(t,4)*(1 - t/duration)*cpts[6][1])/pow(duration,6) + (42*pow(t,5)*cpts[7][1])/pow(duration,7);
   return res;
 }
 throw "not implemented";
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


  if(grad.size() > 0)
    grad = bezier_gradient_2d_8pts(from, to, cpts, duration);

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
  result *= duration;
  return result;*/

  /*auto t1 = Time::now();

  double result2 = bezier_integrate_2d_8pts(from, to, cpts);

  auto t2 = Time::now();

  fsec fs1 = t1 - t0;
  fsec fs2 = t2 - t1;
  us d1 = std::chrono::duration_cast<us>(fs1);
  us d2 = std::chrono::duration_cast<us>(fs2);

  cout << ">>>> My implementation:" << result << " time:" << d1.count() << " Mathematica:" << result2 << " time:" << d2.count() << endl;*/
  return bezier_integrate_2d_8pts(from, to, cpts, duration);
}

curve& curve::operator-=(const curve& rhs) {
  assert(cpts.size() == rhs.cpts.size());
  for(int i=0; i<size(); i++) {
    cpts[i] = cpts[i] - rhs.cpts[i];
  }

  return *this;
}

void curve::print() {
  for(int i=0; i<size(); i++) {
    cout << cpts[i] << " ";
  }
  cout << endl;
}


double curve::velnormsqintegrate() {
  return (7*(462*pow(cpts[0][0],2) + 462*pow(cpts[0][1],2) + 252*pow(cpts[1][0],2) + 252*pow(cpts[1][1],2) + 84*pow(cpts[2][0],2) + 84*pow(cpts[2][1],2) + 84*cpts[2][0]*cpts[3][0] + 60*pow(cpts[3][0],2) + 84*cpts[2][1]*cpts[3][1] +
       60*pow(cpts[3][1],2) - 6*cpts[2][0]*cpts[4][0] + 75*cpts[3][0]*cpts[4][0] + 60*pow(cpts[4][0],2) - 6*cpts[2][1]*cpts[4][1] + 75*cpts[3][1]*cpts[4][1] + 60*pow(cpts[4][1],2) - 51*cpts[2][0]*cpts[5][0] - 6*cpts[3][0]*cpts[5][0] + 84*cpts[4][0]*cpts[5][0] +
       84*pow(cpts[5][0],2) - 51*cpts[2][1]*cpts[5][1] - 6*cpts[3][1]*cpts[5][1] + 84*cpts[4][1]*cpts[5][1] + 84*pow(cpts[5][1],2) - 48*cpts[2][0]*cpts[6][0] - 63*cpts[3][0]*cpts[6][0] - 28*cpts[4][0]*cpts[6][0] + 126*cpts[5][0]*cpts[6][0] + 252*pow(cpts[6][0],2) -
       48*cpts[2][1]*cpts[6][1] - 63*cpts[3][1]*cpts[6][1] - 28*cpts[4][1]*cpts[6][1] + 126*cpts[5][1]*cpts[6][1] + 252*pow(cpts[6][1],2) + cpts[1][0]*(126*cpts[2][0] - 28*cpts[3][0] - 63*cpts[4][0] - 48*cpts[5][0] - 23*cpts[6][0] - 6*cpts[7][0]) -
       7*(3*cpts[2][0] + 8*cpts[3][0] + 18*cpts[4][0] + 36*cpts[5][0] + 66*cpts[6][0])*cpts[7][0] + 462*pow(cpts[7][0],2) - cpts[0][0]*(462*cpts[1][0] + 252*cpts[2][0] + 126*cpts[3][0] + 56*cpts[4][0] + 21*cpts[5][0] + 6*cpts[6][0] + cpts[7][0]) +
       cpts[1][1]*(126*cpts[2][1] - 28*cpts[3][1] - 63*cpts[4][1] - 48*cpts[5][1] - 23*cpts[6][1] - 6*cpts[7][1]) - 7*(3*cpts[2][1] + 8*cpts[3][1] + 18*cpts[4][1] + 36*cpts[5][1] + 66*cpts[6][1])*cpts[7][1] + 462*pow(cpts[7][1],2) -
       cpts[0][1]*(462*cpts[1][1] + 252*cpts[2][1] + 126*cpts[3][1] + 56*cpts[4][1] + 21*cpts[5][1] + 6*cpts[6][1] + cpts[7][1])))/(858.*duration);
}

double curve::accnormsqintegrate() {
  return (14*(126*pow(cpts[0][0],2) + 126*pow(cpts[0][1],2) + 322*pow(cpts[1][0],2) + 322*pow(cpts[1][1],2) + 60*pow(cpts[2][0],2) + 60*pow(cpts[2][1],2) + 20*cpts[2][0]*cpts[3][0] + 20*pow(cpts[3][0],2) + 20*cpts[2][1]*cpts[3][1] +
       20*pow(cpts[3][1],2) - 20*cpts[2][0]*cpts[4][0] + 15*cpts[3][0]*cpts[4][0] + 20*pow(cpts[4][0],2) - 20*cpts[2][1]*cpts[4][1] + 15*cpts[3][1]*cpts[4][1] + 20*pow(cpts[4][1],2) - 21*cpts[2][0]*cpts[5][0] - 20*cpts[3][0]*cpts[5][0] + 20*cpts[4][0]*cpts[5][0] +
       60*pow(cpts[5][0],2) - 21*cpts[2][1]*cpts[5][1] - 20*cpts[3][1]*cpts[5][1] + 20*cpts[4][1]*cpts[5][1] + 60*pow(cpts[5][1],2) - 4*cpts[2][0]*cpts[6][0] - 30*cpts[3][0]*cpts[6][0] - 80*cpts[4][0]*cpts[6][0] - 161*cpts[5][0]*cpts[6][0] + 322*pow(cpts[6][0],2) -
       4*cpts[2][1]*cpts[6][1] - 30*cpts[3][1]*cpts[6][1] - 80*cpts[4][1]*cpts[6][1] - 161*cpts[5][1]*cpts[6][1] + 322*pow(cpts[6][1],2) - cpts[1][0]*(161*cpts[2][0] + 80*cpts[3][0] + 30*cpts[4][0] + 4*cpts[5][0] - 5*cpts[6][0] - 4*cpts[7][0]) +
       (10*cpts[2][0] + 20*cpts[3][0] + 7*(5*cpts[4][0] + 8*cpts[5][0] - 54*cpts[6][0]))*cpts[7][0] + 126*pow(cpts[7][0],2) + cpts[0][0]*(-378*cpts[1][0] + 56*cpts[2][0] + 35*cpts[3][0] + 20*cpts[4][0] + 10*cpts[5][0] + 4*cpts[6][0] + cpts[7][0]) -
       cpts[1][1]*(161*cpts[2][1] + 80*cpts[3][1] + 30*cpts[4][1] + 4*cpts[5][1] - 5*cpts[6][1] - 4*cpts[7][1]) + (10*cpts[2][1] + 20*cpts[3][1] + 7*(5*cpts[4][1] + 8*cpts[5][1] - 54*cpts[6][1]))*cpts[7][1] + 126*pow(cpts[7][1],2) +
       cpts[0][1]*(-378*cpts[1][1] + 56*cpts[2][1] + 35*cpts[3][1] + 20*cpts[4][1] + 10*cpts[5][1] + 4*cpts[6][1] + cpts[7][1])))/(11.*pow(duration,3));
}


double curve::jerknormsqintegrate() {
  return (140*(35*pow(cpts[0][0],2) + 35*pow(cpts[0][1],2) + 230*pow(cpts[1][0],2) + 230*pow(cpts[1][1],2) + 153*pow(cpts[2][0],2) + 153*pow(cpts[2][1],2) - 51*cpts[2][0]*cpts[3][0] + 20*pow(cpts[3][0],2) - 51*cpts[2][1]*cpts[3][1] +
       20*pow(cpts[3][1],2) - 30*cpts[2][0]*cpts[4][0] + 5*cpts[3][0]*cpts[4][0] + 20*pow(cpts[4][0],2) - 30*cpts[2][1]*cpts[4][1] + 5*cpts[3][1]*cpts[4][1] + 20*pow(cpts[4][1],2) - 9*cpts[2][0]*cpts[5][0] - 30*cpts[3][0]*cpts[5][0] - 51*cpts[4][0]*cpts[5][0] +
       153*pow(cpts[5][0],2) - 9*cpts[2][1]*cpts[5][1] - 30*cpts[3][1]*cpts[5][1] - 51*cpts[4][1]*cpts[5][1] + 153*pow(cpts[5][1],2) + 12*cpts[2][0]*cpts[6][0] + 19*cpts[3][0]*cpts[6][0] + 26*cpts[4][0]*cpts[6][0] - 345*cpts[5][0]*cpts[6][0] + 230*pow(cpts[6][0],2) +
       12*cpts[2][1]*cpts[6][1] + 19*cpts[3][1]*cpts[6][1] + 26*cpts[4][1]*cpts[6][1] - 345*cpts[5][1]*cpts[6][1] + 230*pow(cpts[6][1],2) + cpts[1][0]*(-345*cpts[2][0] + 26*cpts[3][0] + 19*cpts[4][0] + 12*cpts[5][0] + 5*cpts[6][0] - 2*cpts[7][0]) -
       (3*cpts[2][0] + 4*cpts[3][0] + 5*(cpts[4][0] - 24*cpts[5][0] + 35*cpts[6][0]))*cpts[7][0] + 35*pow(cpts[7][0],2) - cpts[0][0]*(175*cpts[1][0] - 120*cpts[2][0] + 5*cpts[3][0] + 4*cpts[4][0] + 3*cpts[5][0] + 2*cpts[6][0] + cpts[7][0]) +
       cpts[1][1]*(-345*cpts[2][1] + 26*cpts[3][1] + 19*cpts[4][1] + 12*cpts[5][1] + 5*cpts[6][1] - 2*cpts[7][1]) - (3*cpts[2][1] + 4*cpts[3][1] + 5*(cpts[4][1] - 24*cpts[5][1] + 35*cpts[6][1]))*cpts[7][1] + 35*pow(cpts[7][1],2) -
       cpts[0][1]*(175*cpts[1][1] - 120*cpts[2][1] + 5*cpts[3][1] + 4*cpts[4][1] + 3*cpts[5][1] + 2*cpts[6][1] + cpts[7][1])))/pow(duration,5);
}
