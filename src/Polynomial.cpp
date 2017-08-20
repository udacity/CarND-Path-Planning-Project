//
// Created by Joey Liu on 2017/08/16.
//
#include <vector>
#include "Polynomial.h"

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
      coeff_fd_.push_back(i*coeffs[i]);
    }
    if (i > 1) {
      coeff_sd_.push_back(i*coeff_fd_[i - 1]);
    }
    if (i > 2) {
      coeff_td_.push_back(i*coeff_sd_[i - 2]);
    }
  }

}

vector<double> Polynomial::get_coeff(string order) {

  vector<double> output;

  if (order == "first") {
    output = coeff_fd_;
  } else if (order == "second") {
    output = coeff_sd_;
  } else if (order == "third") {
    output = coeff_td_;
  } else {
    output = coeff_;
  }

  return output;
}

double Polynomial::eval(double x, string order) {

  double output;
  if (order == "first") {
    output = _eval(x, coeff_fd_);
  } else if (order == "second") {
    output = _eval(x, coeff_sd_);
  } else if (order == "third") {
    output = _eval(x, coeff_td_);
  } else {
    output = _eval(x, coeff_);
  }

  return output;

}

double Polynomial::_eval(double x, const vector<double> &coeffs) {
  double output = 0.0;

  for (int i = 0; i < coeffs.size(); i++) {
    output += coeffs[i] * pow(x, i);
  }

  return output;
}