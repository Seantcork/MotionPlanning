#ifndef __geom_h
#define __geom_h

#include <vector>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

struct point2D{
    double x, y;

    bool operator==(const point2D &o) const {
        return x == o.x && y == o.y;
    }

    bool operator<(const point2D &o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
};

typedef struct _point2DD {
  double x;
  double y;
  double angle;
  int distance;
  char type;
} point2DD; 



struct point3D{
    point2D pt;
    float theta;

    bool operator==(const point3D &o) const{
        return pt == o.pt && theta == o.theta;
    }

    bool operator<(const point3D &o) const {
        return pt < o.pt || (pt == o.pt && theta < o.theta);
    }

};





#endif
