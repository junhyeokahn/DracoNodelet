#ifndef SEJONG_SPLINE
#define SEJONG_SPLINE

class Spline{
 public:
  Spline(){}
  virtual ~Spline(){}

  virtual bool SetParam(double * initial, double * final, double ** middle_pt, double fin_time ) = 0;
  virtual bool getCurvePoint(double curr_time, double* pos) = 0;
  virtual bool getCurveDerPoint(double curr_time, int level, double * ret) = 0;
};

#endif
