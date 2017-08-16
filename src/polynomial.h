//
// Created by Joey Liu on 2017/08/16.
//

#ifndef PATH_PLANNING_POLYNOMIAL_H
#define PATH_PLANNING_POLYNOMIAL_H

#include <vector>
#include <cmath>

class Polynomial {
public:
  Polynomial();
  Polynomial(const vector<double> &coeffs);
  virtual ~Polynomial();

  void set_coeff(const vector<double> &coeffs);
  vector<double> get_coeff(string order);
  double eval(const double x, string order);

private:
  vector<double> coeff_;
  // first derivation
  vector<double> coeff_fd_;
  // second derivation
  vector<double> coeff_sd_;
  // third derivation
  vector<double> coeff_td_;

  double _eval(const double x, const vector<double> &coeffs);
};

#endif //PATH_PLANNING_POLYNOMIAL_H
