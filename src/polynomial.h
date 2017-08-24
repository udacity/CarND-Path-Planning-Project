//
// Created by Joey Liu on 2017/08/16.
//

#ifndef PATH_PLANNING_POLYNOMIAL_H
#define PATH_PLANNING_POLYNOMIAL_H

#include <vector>
#include <string>
#include <cmath>

using namespace std;

class Polynomial {
public:
  Polynomial() = default;
  explicit Polynomial(const vector<double> &coeffs);
  virtual ~Polynomial() = default;

  void set_coeff(const vector<double> &coeffs);
  double eval(double x, string order="origin") const;

private:
  vector<double> coeff_;
  // first derivation
  vector<double> coeff_fd_;
  // second derivation
  vector<double> coeff_sd_;
  // third derivation
  vector<double> coeff_td_;

  double eval_(double x, const vector<double> &coeffs) const;
};

#endif //PATH_PLANNING_POLYNOMIAL_H
