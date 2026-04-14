#ifndef BSPLINE_OPT_NLOPT_WRAPPER_HPP
#define BSPLINE_OPT_NLOPT_WRAPPER_HPP

#include <nlopt.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace nlopt {

using algorithm = nlopt_algorithm;
using result = nlopt_result;
using func = double (*)(const std::vector<double>& x, std::vector<double>& grad, void* data);

inline const char* resultMessage(result code) {
  switch (code) {
    case NLOPT_FAILURE:
      return "nlopt failure";
    case NLOPT_INVALID_ARGS:
      return "nlopt invalid args";
    case NLOPT_OUT_OF_MEMORY:
      return "nlopt out of memory";
    case NLOPT_ROUNDOFF_LIMITED:
      return "nlopt roundoff limited";
    case NLOPT_FORCED_STOP:
      return "nlopt forced stop";
    default:
      return "nlopt error";
  }
}

class opt {
public:
  opt(algorithm algo, unsigned n) : opt_(nlopt_create(algo, n)) {
    if (opt_ == nullptr) {
      throw std::runtime_error("failed to create nlopt optimizer");
    }
  }

  ~opt() {
    if (opt_ != nullptr) {
      nlopt_destroy(opt_);
    }
  }

  opt(const opt&) = delete;
  opt& operator=(const opt&) = delete;

  void set_min_objective(func objective, void* data) {
    objective_ = objective;
    objective_data_ = data;
    check(nlopt_set_min_objective(opt_, &opt::objectiveThunk, this));
  }

  void set_maxeval(int maxeval) {
    check(nlopt_set_maxeval(opt_, maxeval));
  }

  void set_maxtime(double maxtime) {
    check(nlopt_set_maxtime(opt_, maxtime));
  }

  void set_xtol_rel(double tol) {
    check(nlopt_set_xtol_rel(opt_, tol));
  }

  void set_lower_bounds(const std::vector<double>& lower_bounds) {
    check(nlopt_set_lower_bounds(opt_, lower_bounds.data()));
  }

  void set_upper_bounds(const std::vector<double>& upper_bounds) {
    check(nlopt_set_upper_bounds(opt_, upper_bounds.data()));
  }

  result optimize(std::vector<double>& x, double& optimum_value) {
    result res = nlopt_optimize(opt_, x.data(), &optimum_value);
    if (res < 0) {
      throw std::runtime_error(resultMessage(res));
    }
    return res;
  }

private:
  static double objectiveThunk(unsigned n, const double* x, double* grad, void* data) {
    auto* self = static_cast<opt*>(data);
    if (self->objective_ == nullptr) {
      throw std::runtime_error("nlopt objective is not set");
    }

    std::vector<double> x_vec(x, x + n);
    std::vector<double> grad_vec;
    if (grad != nullptr) {
      grad_vec.assign(n, 0.0);
    }

    const double value = self->objective_(x_vec, grad_vec, self->objective_data_);

    if (grad != nullptr && grad_vec.size() == n) {
      for (unsigned i = 0; i < n; ++i) {
        grad[i] = grad_vec[i];
      }
    }
    return value;
  }

  void check(result res) const {
    if (res < 0) {
      throw std::runtime_error(resultMessage(res));
    }
  }

  nlopt_opt opt_;
  func objective_ = nullptr;
  void* objective_data_ = nullptr;
};

}  // namespace nlopt

#endif
