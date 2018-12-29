#ifndef TRIGS_H
#define TRIGS_H

#include <math.h>
#include "Eigen-3.3/Eigen/Dense"

class Trigs{
  public:
    static constexpr double pi() { return M_PI; }
    static double deg2rad(double x) { return x * pi() / 180; }
    static double rad2deg(double x) { return x * 180 / pi(); }
    static double distance(double x1, double y1, double x2, double y2){
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    static Eigen::Vector3d homo(){Eigen::Vector3d h; h<<0.0, 0.0, 1.0; return h;};
    static Eigen::MatrixXd translate(Eigen::MatrixXd source, double dx, double dy){
      Eigen::Vector3d translation(dx, dy, 0);
      return source.colwise() + translation;
    }
    static Eigen::MatrixXd rotate(Eigen::MatrixXd source, double theta){
      Eigen::Matrix3d rotation;
      rotation << cos(theta), -sin(theta), 0,
                  sin(theta), cos(theta), 0,
                  0, 0, 1;
      return rotation * source;
    }

    static Eigen::MatrixXd rigid(Eigen::MatrixXd source, double theta, double dx, double dy){
      Eigen::Matrix3d transform;
      transform << cos(theta), -sin(theta), dx,
                  sin(theta), cos(theta), dy,
                  0, 0, 1;
      return transform * source;
    }

    static Eigen::MatrixXd rigid_reverse(Eigen::MatrixXd source, double theta, double dx, double dy){
      Eigen::MatrixXd result = Trigs::translate(source, dx, dy);
      result = Trigs::rotate(result, theta);
      return result;
    }


};

#endif
