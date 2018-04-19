#ifndef PATHREPLAN_EDT_H
#define PATHREPLAN_EDT_H

class edt {
  private:
    double step_size = 0.01;
    double x_max;
    double y_max;
    double x_min;
    double y_min;
    vector<vector<double> > grid;

  public:

    class line_segment {
      public:
        vectoreuc p1;
        vectoreuc p2;
    };

    edt(double ss, double xm, double xM, double ym, double yM);
    void construct(vector<line_segment*>);
};

#endif
