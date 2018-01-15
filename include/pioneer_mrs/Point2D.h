#ifndef PIONEER_MRS_POINT2D_H_
#define PIONEER_MRS_POINT2D_H_

class Point2D
{
  public:
    Point2D() {x = 0; y = 0;}
    Point2D(double a, double b) {x = a; y = b;}
    
    Point2D operator + (Point2D &p) {return Point2D(x + p.x, y + p.y);}
    Point2D operator - (Point2D &p) {return Point2D(x - p.x, y - p.y);}
    Point2D operator * (double p) {return Point2D(x * p, y * p);}

  public:
    double x;
    double y;
};

#endif /* PIONEER_MRS_POINT2D_H_ */