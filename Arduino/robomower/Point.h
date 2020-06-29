#ifndef Point_h
#define Point_h
//allows two numbers to be stored as an x,y point
struct Point {
  double x, y;
  Point(double a, double b) {
    this->x = a;
    this->y = b;
  }

 
};

 Point addPoint(Point, double ,double);


#endif
