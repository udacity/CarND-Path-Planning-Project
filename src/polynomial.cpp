//
// Created by Joey Liu on 2017/08/16.
//
#include <vector>
#include "polynomial.h"

Polynomial::Polynomial(const vector<double> &coeffs) {
  set_coeff(coeffs);
}

void Polynomial::set_coeff(const vector<double> &coeffs) {

  coeff_.clear();
  coeff_fd_.clear();
  coeff_sd_.clear();
  coeff_td_.clear();

  for (int i = 0; i < coeffs.size(); i++) {
    coeff_.push_back(coeffs[i]);
    if (i > 0) {
      double val = i*coeffs[i];
      coeff_fd_.push_back(val);
      if (i > 1) {
        coeff_sd_.push_back((i - 1)*val);
        if (i > 2) {
          coeff_td_.push_back((i - 2)*val);
        }
      }
    }
  }

}

double Polynomial::eval(double x, string order) const {

  double output;
  if (order == "first") {
    output = eval_(x, coeff_fd_);
  } else if (order == "second") {
    output = eval_(x, coeff_sd_);
  } else if (order == "third") {
    output = eval_(x, coeff_td_);
  } else {
    output = eval_(x, coeff_);
  }

  return output;

}

double Polynomial::eval_(double x, const vector<double> &coeffs) const {
  double output = 0.0;

  for (int i = 0; i < coeffs.size(); i++) {
    output += coeffs[i] * pow(x, i);
  }

  return output;
}