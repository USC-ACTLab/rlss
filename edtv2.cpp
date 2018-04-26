#include "edtv2.h"
#include "edt.h"
#include <iostream>
using namespace std;

edtv2::edtv2(double ss, double xm, double xM, double ym, double yM): step_size(ss), x_min(xm), x_max(xM), y_min(ym), y_max(yM) {

}


void edtv2::construct(vector<obstacle2D>* obstacles) {

  for(double y = y_min; y<=y_max; y+=step_size) {
    vector<double> distRow;
    for(double x = x_min; x<=x_max; x+=step_size) {
      vectoreuc pt(2);
      pt[0] = x;
      pt[1] = y;
      double dist = 0;
      bool dist_changed = false;
      for(int i=0; i<obstacles->size(); i++) {
        obstacle2D& obs = (*obstacles)[i];
        if(obs.point_inside(pt)) {
          if(!dist_changed) {
            dist = obs.closest_distance(pt);
            dist_changed = true;
          } else {
            if(dist > 0) {
              dist = min(dist, obs.closest_distance(pt));
            } else {
              dist = obs.closest_distance(pt);
            }
          }
        } else {
          if(!dist_changed) {
            dist = obs.farthest_distance(pt);
            dist_changed = true;
          } else {
            dist = max(dist, obs.farthest_distance(pt));
          }
        }
      }
      distRow.push_back(dist);
    }
    grid.push_back(distRow);
  }

  /*for(int i=0; i<grid.size(); i++) {
    for(int j=0; j<grid.size(); j++) {
      if(grid[i][j] != 0)
      cout << i << " " << j << " " << grid[i][j] << endl;
    }
  }
  cout << endl;
  cin >> a;*/
}


double edtv2::interpolate_for(vector<double>& P, double dur, double ct, vector<double>& grad) {
  double ss = step_size;

  double x = pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) +
   (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7);
  double y = pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
   (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) + (pow(ct,7)*P[7*2+1])/pow(dur,7);



  int left_idx = (x + fabs(x_min)) / step_size + 0.5; // index of min x
  int bottom_idx = (y + fabs(y_min)) / step_size + 0.5; // index of min y

  double x1 = x_min + left_idx * step_size;
  double y1 = y_min + bottom_idx * step_size;
  double x2 = x1+step_size;
  double y2 = y1+step_size;



  double q11 = grid[bottom_idx][left_idx];
  double q12 = grid[bottom_idx+1][left_idx];
  double q21 = grid[bottom_idx][left_idx+1];
  double q22 = grid[bottom_idx+1][left_idx+1];

  //cout << q11 << " " << q12 << " " << q21 << " " << q22 << endl;

  if(grad.size() > 0) {
    grad = vector<double> {
      ((-((pow(1 - ct/dur,7)*q11)/ss) + (pow(1 - ct/dur,7)*q21)/ss)*(y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) -
         (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) -
         (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) - (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss +
    ((-((pow(1 - ct/dur,7)*q12)/ss) + (pow(1 - ct/dur,7)*q22)/ss)*(-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) +
         (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) +
         (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) + (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,
   -((pow(1 - ct/dur,7)*((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
                (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
                (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
           (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
                (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
                (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/ss) +
    (pow(1 - ct/dur,7)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/ss,
   (((-7*ct*pow(1 - ct/dur,6)*q11)/(dur*ss) + (7*ct*pow(1 - ct/dur,6)*q21)/(dur*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((-7*ct*pow(1 - ct/dur,6)*q12)/(dur*ss) + (7*ct*pow(1 - ct/dur,6)*q22)/(dur*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,(-7*ct*pow(1 - ct/dur,6)*((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur -
              (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) -
              (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(dur*ss) +
    (7*ct*pow(1 - ct/dur,6)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(dur*ss),
   (((-21*pow(ct,2)*pow(1 - ct/dur,5)*q11)/(pow(dur,2)*ss) + (21*pow(ct,2)*pow(1 - ct/dur,5)*q21)/(pow(dur,2)*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((-21*pow(ct,2)*pow(1 - ct/dur,5)*q12)/(pow(dur,2)*ss) + (21*pow(ct,2)*pow(1 - ct/dur,5)*q22)/(pow(dur,2)*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,(-21*pow(ct,2)*pow(1 - ct/dur,5)*
       ((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) -
              (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) -
              (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss + (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,2)*ss) +
    (21*pow(ct,2)*pow(1 - ct/dur,5)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,2)*ss),
   (((-35*pow(ct,3)*pow(1 - ct/dur,4)*q11)/(pow(dur,3)*ss) + (35*pow(ct,3)*pow(1 - ct/dur,4)*q21)/(pow(dur,3)*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((-35*pow(ct,3)*pow(1 - ct/dur,4)*q12)/(pow(dur,3)*ss) + (35*pow(ct,3)*pow(1 - ct/dur,4)*q22)/(pow(dur,3)*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,(-35*pow(ct,3)*pow(1 - ct/dur,4)*
       ((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) -
              (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) -
              (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss + (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,3)*ss) +
    (35*pow(ct,3)*pow(1 - ct/dur,4)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,3)*ss),
   (((-35*pow(ct,4)*pow(1 - ct/dur,3)*q11)/(pow(dur,4)*ss) + (35*pow(ct,4)*pow(1 - ct/dur,3)*q21)/(pow(dur,4)*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((-35*pow(ct,4)*pow(1 - ct/dur,3)*q12)/(pow(dur,4)*ss) + (35*pow(ct,4)*pow(1 - ct/dur,3)*q22)/(pow(dur,4)*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,(-35*pow(ct,4)*pow(1 - ct/dur,3)*
       ((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) -
              (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) -
              (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss + (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,4)*ss) +
    (35*pow(ct,4)*pow(1 - ct/dur,3)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,4)*ss),
   (((-21*pow(ct,5)*pow(1 - ct/dur,2)*q11)/(pow(dur,5)*ss) + (21*pow(ct,5)*pow(1 - ct/dur,2)*q21)/(pow(dur,5)*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((-21*pow(ct,5)*pow(1 - ct/dur,2)*q12)/(pow(dur,5)*ss) + (21*pow(ct,5)*pow(1 - ct/dur,2)*q22)/(pow(dur,5)*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,(-21*pow(ct,5)*pow(1 - ct/dur,2)*
       ((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) -
              (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) -
              (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss + (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,5)*ss) +
    (21*pow(ct,5)*pow(1 - ct/dur,2)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,5)*ss),
   (((-7*pow(ct,6)*(1 - ct/dur)*q11)/(pow(dur,6)*ss) + (7*pow(ct,6)*(1 - ct/dur)*q21)/(pow(dur,6)*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((-7*pow(ct,6)*(1 - ct/dur)*q12)/(pow(dur,6)*ss) + (7*pow(ct,6)*(1 - ct/dur)*q22)/(pow(dur,6)*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,(-7*pow(ct,6)*(1 - ct/dur)*((q11*
            (x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) -
              (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) -
              (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss + (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,6)*ss) +
    (7*pow(ct,6)*(1 - ct/dur)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,6)*ss),
   ((-((pow(ct,7)*q11)/(pow(dur,7)*ss)) + (pow(ct,7)*q21)/(pow(dur,7)*ss))*
       (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + ((-((pow(ct,7)*q12)/(pow(dur,7)*ss)) + (pow(ct,7)*q22)/(pow(dur,7)*ss))*
       (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
         (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
         (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss,-((pow(ct,7)*((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
                (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
                (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
           (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
                (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
                (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,7)*ss)) +
    (pow(ct,7)*((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
         (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
              (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
              (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss))/(pow(dur,7)*ss)
    };
  }


  return (((q11*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) -
             (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) -
             (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss + (q21*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
             (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
             (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss)*
      (y2 - pow(1 - ct/dur,7)*P[0*2+1] - (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) - (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) -
        (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) - (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) -
        (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss + (((q12*(x2 - pow(1 - ct/dur,7)*P[0*2+0] - (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur - (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) -
             (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) - (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) - (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) -
             (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) - (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss +
        (q22*(-x1 + pow(1 - ct/dur,7)*P[0*2+0] + (7*ct*pow(1 - ct/dur,6)*P[1*2+0])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+0])/pow(dur,2) +
             (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+0])/pow(dur,3) + (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+0])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+0])/pow(dur,5) +
             (7*pow(ct,6)*(1 - ct/dur)*P[6*2+0])/pow(dur,6) + (pow(ct,7)*P[7*2+0])/pow(dur,7)))/ss)*
      (-y1 + pow(1 - ct/dur,7)*P[0*2+1] + (7*ct*pow(1 - ct/dur,6)*P[1*2+1])/dur + (21*pow(ct,2)*pow(1 - ct/dur,5)*P[2*2+1])/pow(dur,2) + (35*pow(ct,3)*pow(1 - ct/dur,4)*P[3*2+1])/pow(dur,3) +
        (35*pow(ct,4)*pow(1 - ct/dur,3)*P[4*2+1])/pow(dur,4) + (21*pow(ct,5)*pow(1 - ct/dur,2)*P[5*2+1])/pow(dur,5) + (7*pow(ct,6)*(1 - ct/dur)*P[6*2+1])/pow(dur,6) +
        (pow(ct,7)*P[7*2+1])/pow(dur,7)))/ss;

}
