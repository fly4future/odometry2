#ifndef ODOMETRY2_HEADING_ESTIMATOR_H
#define ODOMETRY2_HEADING_ESTIMATOR_H

#include <rclcpp/rclcpp.hpp>

#include <lib/lkf.h>

#include <string>
#include <vector>
#include <mutex>

#include "types.h"

#define HDG_DT 0.01
/* #define HDG_INPUT_COEFF 0.2 */
#define HDG_INPUT_COEFF 0.0

namespace odometry2
{

class HeadingEstimator {

public:
  HeadingEstimator(const std::string &estimator_name, const hdg_Q_t &Q, const std::vector<hdg_R_t> &R_multi);

  bool        doPrediction(const double input, double dt);
  bool        doPrediction(const double input);
  bool        doCorrection(const double measurement, int measurement_type);
  bool        getStates(hdg_x_t &states);
  bool        getState(int state_id, double &state);
  std::string getName(void);
  bool        setState(int state_id, const double state);
  bool        setR(double cov, int measurement_type);
  bool        getR(double &cov, int measurement_type);
  bool        getCovariance(hdg_P_t &cov);
  bool        setCovariance(const hdg_P_t &cov);
  bool        reset(const hdg_x_t &states);

private:
  std::string m_estimator_name;
  int         m_n_states;
  size_t      m_n_measurement_types;

  // State transition matrix
  hdg_A_t m_A;

  // Input matrix
  hdg_B_t m_B;

  // Input coefficient
  double m_b = HDG_INPUT_COEFF;

  // Array with mapping matrices for each fused measurement
  std::vector<hdg_H_t> m_H_multi;

  // Process covariance matrix
  hdg_Q_t m_Q;

  // Array with covariances of each fused measurement
  std::vector<hdg_R_t> m_R_multi;

  // Default dt
  double m_dt    = HDG_DT;
  double m_dt_sq = m_dt * m_dt / 2;

  // Kalman filter - the core of the estimator
  std::unique_ptr<lkf_hdg_t> mp_lkf;

  // Variable for holding the current state and covariance
  hdg_statecov_t m_sc;

  std::mutex mutex_lkf;

  bool m_is_initialized = false;
};

}  // namespace odometry2

#endif
