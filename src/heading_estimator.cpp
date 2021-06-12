#include "heading_estimator.h"

namespace odometry2
{

/*  //{ HeadingEstimator() */

// clang-format off
HeadingEstimator::HeadingEstimator(
    const std::string &estimator_name,
    const hdg_Q_t &Q,
    const std::vector<hdg_R_t> &R_multi)
    :
    m_estimator_name(estimator_name),
    m_Q(Q),
    m_R_multi(R_multi)
  {

  // clang-format on

  // Number of states
  m_n_states = m_A.rows();

  // Number of measurement types
  m_n_measurement_types = 2;

  alt_H_t pos_H, vel_H, acc_H;
  pos_H << 1, 0, 0;
  vel_H << 0, 1, 0;
  m_H_multi = {pos_H, vel_H};

  /*  //{ sanity checks */

  // Check size of m_R
  if (m_Q.rows() != m_n_states) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".HeadingEstimator()"
              << "): wrong size of \"R.rows()\". Should be: " << m_n_states << " is:" << m_Q.rows() << std::endl;
    return;
  }

  if (m_Q.cols() != m_n_states) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".HeadingEstimator()"
              << "): wrong size of \"R.cols()\". Should be: " << m_n_states << " is:" << m_Q.cols() << std::endl;
    return;
  }

  // Check size of m_H_multi
  if (m_H_multi.size() != m_n_measurement_types) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".HeadingEstimator()"
              << "): wrong size of \"m_H_multi\". Should be: " << m_n_measurement_types << " is:" << m_H_multi.size() << std::endl;
    return;
  }

  // Check size of m_H_multi elements
  for (size_t i = 0; i < m_H_multi.size(); i++) {
    if (m_H_multi[i].rows() != 1 || m_H_multi[i].cols() != m_n_states) {
      std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".HeadingEstimator()"
                << "): wrong size of \"m_H_multi[" << i << "]\". Should be: (1, " << m_n_states << ") is: (" << m_H_multi[i].rows() << ", "
                << m_H_multi[i].cols() << ")" << std::endl;
      return;
    }
  }

  // Check size of m_R_multi
  if (m_R_multi.size() != m_n_measurement_types) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".HeadingEstimator()"
              << "): wrong size of \"m_R_multi\". Should be: " << m_n_measurement_types << " is:" << m_R_multi.size() << std::endl;
    return;
  }

  // Check size of m_R_multi elements
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    if (m_R_multi[i].rows() != 1 || m_R_multi[i].cols() != 1) {
      std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".HeadingEstimator()"
                << "): wrong size of \"m_R_multi[" << i << "]\". Should be: (1, 1) is: (" << m_R_multi[i].rows() << ", " << m_R_multi[i].cols() << ")"
                << std::endl;
      return;
    }
  }

  //}

  // clang-format off
  m_A <<
      1.0, m_dt, m_dt_sq,
      0, 1-m_b, m_dt,
      0, 0, 1.0;

  m_B << 0, 0, // TODO try with heading input
         0, m_b,
         0, 0;
  // clang-format on

  // set measurement mapping matrix H to zero, it will be set later during each correction step
  hdg_H_t m_H_zero = m_H_zero.Zero();

  // Initialize all states to 0
  const hdg_x_t        x0    = hdg_x_t::Zero();
  hdg_P_t              P_tmp = hdg_P_t::Identity();
  const hdg_P_t        P0    = 1000.0 * P_tmp * P_tmp.transpose();
  const hdg_statecov_t sc0({x0, P0});
  m_sc               = sc0;
  const hdg_u_t   u0 = hdg_u_t::Zero();
  const rclcpp::Time t0 = rclcpp::Time(0);

    // Initialize a single LKF
    mp_lkf = std::make_unique<lkf_hdg_t>(m_A, m_B, m_H_zero);

  std::cout << "[HeadingEstimator]: New HeadingEstimator initialized " << std::endl;
  std::cout << "name: " << m_estimator_name << std::endl;

  std::cout << std::endl << " H_multi: " << std::endl;
  for (size_t i = 0; i < m_H_multi.size(); i++) {
    std::cout << m_H_multi[i] << std::endl;
  }
  std::cout << std::endl << " R_multi: " << std::endl;
  for (size_t i = 0; i < m_R_multi.size(); i++) {
    std::cout << m_R_multi[i] << std::endl;
  }
  std::cout << std::endl << " A: " << std::endl << m_A << std::endl << " B: " << std::endl << m_B << std::endl << " Q: " << std::endl << m_Q << std::endl;

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool HeadingEstimator::doPrediction(const double input, double dt) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;


  // Check for NaNs
    if (!std::isfinite(input)) {
      std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".doPrediction(const double input=" << input << ", double dt=" << dt
                << "): NaN detected in variable \"input(0)\"." << std::endl;
      return false;
    }

  if (!std::isfinite(dt)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}


  hdg_A_t A = m_A;
  /* hdg_B_t B = m_B; */

  double dt_sq = std::pow(dt, 2) / 2;

  /* B(0, 0)           = dt; */
  /* B(1, 1)           = dt; */

  A(0, 1) = dt;
  A(1, 2) = dt;

  A(0, 2) = dt_sq;

  hdg_u_t u;
  u << input;

  {
    std::scoped_lock lock(mutex_lkf);

    try {
      // Apply the prediction step
         
        mp_lkf->A = A;
        m_sc      = mp_lkf->predict(m_sc, u, m_Q, dt);
      /* mp_lkf->B = B; */
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      std::cerr << "[HeadingEstimator]: LKF prediction step failed: " << e.what();
    }
  }
  return true;
}

//}

/*  //{ doPrediction() */

bool HeadingEstimator::doPrediction(const double input) {

  return doPrediction(input, m_dt);
}

//}

/*  //{ doCorrection() */

bool HeadingEstimator::doCorrection(const double measurement, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".doCorrection(const double measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".doCorrection(const double &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement\"." << std::endl;
    return false;
  }

  //}

  // Prepare the measurement vector
  hdg_z_t z;
  z << measurement;

  hdg_R_t R;
  R << m_R_multi[measurement_type];

  // Fuse the measurement
  std::scoped_lock lock(mutex_lkf);
  {

    try {
        mp_lkf->H = m_H_multi[measurement_type];
        m_sc      = mp_lkf->correct(m_sc, z, R);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      std::cerr << "[HeadingEstimator]: LKF correction step failed: " <<  e.what();
    }
  }

  return true;
}

//}

/*  //{ getStates() */

bool HeadingEstimator::getStates(hdg_x_t &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  states = m_sc.x;

  return true;
}

//}

/*  //{ getState() */

bool HeadingEstimator::getState(int state_id, double &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << ", Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    state = m_sc.x(state_id);
  }

  return true;
}

//}

/*  //{ getName() */

std::string HeadingEstimator::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool HeadingEstimator::setState(int state_id, const double state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x(state_id) = state;
  }

  return true;
}

//}

/*  //{ setR() */

bool HeadingEstimator::setR(double cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  //}

  double old_cov = m_R_multi[measurement_type](0, 0);

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_multi[measurement_type](0, 0) = cov;
  }

  std::cout << "[HeadingEstimator]: " << m_estimator_name << ".setQ(double cov=" << cov << ", int measurement_type=" << measurement_type << ")"
            << " Changed covariance from: " << old_cov << " to: " << m_R_multi[measurement_type](0, 0) << std::endl;

  return true;
}

//}

/*  //{ getR() */

bool HeadingEstimator::getR(double &cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_R_multi[measurement_type](0, 0);
  }

  return true;
}

//}

/*  //{ getCovariance() */

bool HeadingEstimator::getCovariance(hdg_P_t &cov) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_sc.P;
  }

  return true;
}

//}

/*  //{ setCovariance() */

bool HeadingEstimator::setCovariance(const hdg_P_t &cov) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of measurement
  if (cov.rows() != m_n_states || cov.cols() != m_n_states) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &cov=" << cov << "): wrong size of \"cov\". Should be: ("
              << m_n_states << "," << m_n_states << ") is: (" << cov.rows() << "," << cov.cols() << ")" << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < m_n_states; i++) {
    if (!std::isfinite(cov(i, i))) {
      std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".setCovariance(const Eigen::MatrixXd &cov=" << cov << "): NaN detected in variable \"cov("
                << i << "," << i << ")\"." << std::endl;
      return false;
    }
  }

  //}

  // Set the covariance
  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.P = cov;
  }

  return true;
}
//}

/*  //{ reset() */

bool HeadingEstimator::reset(const hdg_x_t &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;
  // Check size of states
  if ((int)states.rows() != m_n_states) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.rows()\". Should be: " << m_n_states << " is:" << states.rows() << std::endl;
    return false;
  }

  if (states.cols() != 1) {
    std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.cols()\". Should be: " << 1 << " is:" << states.cols() << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < states.rows(); i++) {
    for (int j = 0; j < states.cols(); j++) {
      if (!std::isfinite(states(i, j))) {
        std::cerr << "[HeadingEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
                  << "): NaN detected in variable \"states(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_sc.x = (states.col(0));
  }

  return true;
}

//}

}  // namespace odometry2
