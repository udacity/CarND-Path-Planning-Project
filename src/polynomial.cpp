//
// Created by Joey Liu on 2017/08/16.
//
#include <vector>
#include "polynomial.h"

Polynomial::Polynomial() {}
Polynomial::Polynomial(const vector<double> &coeffs) {
  set_coeff(coeffs);
}

Polynomial::~Polynomial() {}

void Polynomial::set_coeff(const vector<double> &coeffs) {

  coeff_.clear();
  coeff_fd_.clear();
  coeff_dd_.clear();
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

vector<double> Polynomial::get_coeff(string order="origin") {

  auto output;

  if (order.compare("first") == 0) {
    output = coeff_fd_;
  } else if (order.compare("second") == 0) {
    output = coeff_sd_;
  } else if (order.compare("third") == 0) {
    output = coeff_td_;
  } else {
    output = coeff_;
  }

  return output;
}

double Polynomial::eval(const double x, string order="origin") {

  double output = 0.0;
  if (order.compare("first") == 0) {
    output = _eval(x, coeff_fd_);
  } else if (order.compare("second") == 0) {
    output = _eval(x, coeff_sd_);
  } else if (order.compare("third") == 0) {
    output = _eval(x, coeff_td_);
  } else {
    output = _eval(x, coeff_);
  }

  return output;

}

double Polynomial::_eval(const double x, const vector<double> &coeffs) {
  double output = 0.0;

  for (int i = 0; i < coeffs.size(); i++) {
    output += coeffs[i] * pow(x, i);
  }

  return output;
}