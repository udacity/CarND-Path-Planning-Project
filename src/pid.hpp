#ifndef PPP_PID
#define PPP_PID

class PID {
  public:
  double kp;
  double kd;
  double ki;

  double cte;
  double prev_cte;
  double cte_sum;
  bool has_prev;

  PID(double kp, double kd, double ki) {
    this->kp = 0.4;
    this->kd = 0.05;
    this->ki = 0.01;
    this->cte = 0;
    this->prev_cte = 0;
    this->cte_sum = 0;
    this->has_prev = false;
  }

  PID(const PID& rhs) {
    this->kp = rhs.kp;
    this->kd = rhs.kd;
    this->ki = rhs.ki;
    this->cte = rhs.cte;
    this->prev_cte = rhs.prev_cte;
    this->cte_sum = rhs.cte_sum;
    this->has_prev = rhs.has_prev;
  }

  void Reset() {
    this->cte = 0;
    this->prev_cte = 0;
    this->cte_sum = 0;
    this->has_prev = false;
  }

  double Update(double cte) {
    this->prev_cte = this->cte;
    this->cte = cte;
    this->cte_sum += cte;
    double d_param = 0;
    if (!has_prev) {
      has_prev = true;
    } else {
      d_param = (this->cte - this->prev_cte) * kd;
    }
    return (this->cte * kp) + d_param + (this->cte_sum * ki);
  }
};

#endif