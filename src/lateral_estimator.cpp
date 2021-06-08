#include "StateEstimator.h"

namespace mrs_uav_odometry
{

/*  //{ StateEstimator() */

// clang-format off
StateEstimator::StateEstimator(
    const std::string &estimator_name,
    const std::vector<bool> &fusing_measurement,
    const LatMat &Q,
    const std::vector<LatStateCol1D> &H,
    const std::vector<Mat1> &R_arr,
    const bool use_repredictor)
    :
    m_estimator_name(estimator_name),
    m_fusing_measurement(fusing_measurement),
    m_Q(Q),
    m_H(H),
    m_R_arr(R_arr),
    m_use_repredictor(use_repredictor)
  {

  // clang-format on

  // Number of states
  m_n_states = sc_x.x.size();

  // Number of measurement types
  m_n_measurement_types = m_fusing_measurement.size();

  /*  //{ sanity checks */

  // Check size of m_Q
  if (m_Q.rows() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"Q.rows()\". Should be: " << m_n_states << " is:" << m_Q.rows() << std::endl;
    return;
  }

  if (m_Q.cols() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"Q.cols()\". Should be: " << m_n_states << " is:" << m_Q.cols() << std::endl;
    return;
  }

  // Check size of m_R_arr
  if (m_R_arr.size() != m_n_measurement_types) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
              << "): wrong size of \"m_R_arr\". Should be: " << m_n_measurement_types << " is:" << m_R_arr.size() << std::endl;
    return;
  }

  // Check size of m_R_arr elements
  for (size_t i = 0; i < m_R_arr.size(); i++) {
    if (m_R_arr[i].rows() != 1 || m_R_arr[i].cols() != 1) {
      std::cerr << "[StateEstimator]: " << m_estimator_name << ".StateEstimator()"
                << "): wrong size of \"m_R_arr[" << i << "]\". Should be: (1, " << 1 << ") is: (" << m_R_arr[i].rows() << ", " << m_R_arr[i].cols() << ")"
                << std::endl;
      return;
    }
  }


  //}

  // Initialize all states to 0
  const x_t        x0    = x_t::Zero();
  P_t              P_tmp = P_t::Identity();
  const P_t        P0    = 1000.0 * P_tmp;
  const statecov_t scx0({x0, P0});
  const statecov_t scy0({x0, P0});
  sc_x               = scx0;
  sc_y               = scy0;
  const u_t       u0 = u_t::Zero();
  const ros::Time t0 = ros::Time(0);

  if (m_use_repredictor) {
    std::cout << "[StateEstimator]: Using repredictor in " << m_estimator_name.c_str() << " estimator." << std::endl;
    // Initialize separate LKF models for each H matrix
    for (size_t i = 0; i < m_H.size(); i++) {
      std::vector<H_t> curr_H{m_H[i]};
      mp_lkf_vector.push_back(std::make_shared<mrs_lib::LKF_MRS_odom>(curr_H, 0.01));
    }
    // Initialize repredictor
    mp_rep_x = std::make_unique<rep_lat_t>(x0, P0, u0, Q, t0, mp_lkf_vector.at(0), m_buf_sz);
    mp_rep_y = std::make_unique<rep_lat_t>(x0, P0, u0, Q, t0, mp_lkf_vector.at(0), m_buf_sz);
  } else {
    // Initialize a single LKF
    mp_lkf_x = std::make_unique<mrs_lib::LKF_MRS_odom>(m_H, 0.01);
    mp_lkf_y = std::make_unique<mrs_lib::LKF_MRS_odom>(m_H, 0.01);
  }

  std::cout << "[StateEstimator]: New StateEstimator initialized " << std::endl;
  std::cout << "name: " << m_estimator_name << std::endl;
  std::cout << " fusing measurements: " << std::endl;
  for (size_t i = 0; i < m_fusing_measurement.size(); i++) {
    std::cout << m_fusing_measurement[i] << " ";
  }
  std::cout << std::endl << " R_arr: " << std::endl;
  for (size_t i = 0; i < m_R_arr.size(); i++) {
    std::cout << m_R_arr[i] << std::endl;
  }
  std::cout << std::endl << " H_arr: " << std::endl;
  for (size_t i = 0; i < m_H.size(); i++) {
    std::cout << m_H[i] << std::endl;
  }
  std::cout << std::endl << " Q: " << std::endl << m_Q << std::endl;

  m_is_initialized = true;
}

//}

/*  //{ doPrediction() */

bool StateEstimator::doPrediction(const Vec2 &input, double dt, const ros::Time &input_stamp, const ros::Time &predict_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of input
  if (input.size() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): wrong size of \"input\". Should be: " << 2 << " is:" << input.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(input(0))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(input(1))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"input(1)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(dt)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): NaN detected in variable \"dt\"." << std::endl;
    return false;
  }

  // Check for non-positive dt
  if (dt <= 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doPrediction(const Eigen::VectorXd &input=" << input << ", double dt=" << dt
              << "): \"dt\" should be > 0." << std::endl;
    return false;
  }

  //}

  /* std::cout << "[StateEstimator]: " << m_estimator_name << " fusing input: " << input << " with time step: " << dt << std::endl; */

  u_t u_x;
  u_t u_y;

  u_x << input(0);
  u_y << input(1);

  /* LatMat newA = m_A; */
  /* newA(0, 1)           = dt; */
  /* newA(1, 2)           = dt; */
  /* newA(1, 3)           = std::pow(dt,2); */

  {
    std::scoped_lock lock(mutex_lkf);

    /* mp_lkf_x->setA(newA); */
    /* mp_lkf_x->setInput(input_vec_x); */
    /* mp_lkf_x->iterateWithoutCorrection(); */
    /* mp_lkf_y->setA(newA); */
    /* mp_lkf_y->setInput(input_vec_y); */
    /* mp_lkf_y->iterateWithoutCorrection(); */
    try {
      // Apply the prediction step
      if (m_use_repredictor) {
        mp_rep_x->addInputChangeWithNoise(u_x, m_Q, input_stamp, mp_lkf_vector[0]);
        sc_x = mp_rep_x->predictTo(predict_stamp);
        mp_rep_y->addInputChangeWithNoise(u_y, m_Q, input_stamp, mp_lkf_vector[0]);
        sc_y = mp_rep_y->predictTo(predict_stamp);
      } else {
        sc_x = mp_lkf_x->predict(sc_x, u_x, m_Q, dt);
        sc_y = mp_lkf_y->predict(sc_y, u_y, m_Q, dt);
      }
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[Odometry]: LKF prediction step failed: %s", e.what());
    }
  }

  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: prediction step" << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: input  x:" << u_x << std::endl << "y: " << u_y << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: m_Q:" << m_Q << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: dt:" << dt << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: state  x:" << sc_x.x << std::endl << "y: " << sc_y.x << std::endl << std::endl); */

  return true;
}

//}

/*  //{ doCorrection() */

bool StateEstimator::doCorrection(const Vec2 &measurement, int measurement_type, const ros::Time &meas_stamp, const ros::Time &predict_stamp) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of measurement
  if (measurement.size() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): wrong size of \"input\". Should be: " << 2 << " is:" << measurement.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(measurement(0))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement(1))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(measurement_type)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): NaN detected in variable \"measurement(0)\"." << std::endl;
    return false;
  }

  // Check for valid value of measurement
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".doCorrection(const Eigen::VectorXd &measurement=" << measurement
              << ", int measurement_type=" << measurement_type << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  // Check whether the measurement type is fused by this estimator
  if (!m_fusing_measurement[measurement_type]) {
    return false;
  }

  //}

  z_t z_x;
  z_t z_y;
  R_t R;

  z_x << measurement(0);
  z_y << measurement(1);
  R << m_R_arr[measurement_type];

  {
    std::scoped_lock lock(mutex_lkf);

    /* mp_lkf_x->setP(m_P_arr[measurement_type]); */
    /* mp_lkf_x->setMeasurement(mes_vec_x, m_R_arr[measurement_type]); */
    /* mp_lkf_x->doCorrection(); */
    /* mp_lkf_y->setP(m_P_arr[measurement_type]); */
    /* mp_lkf_y->setMeasurement(mes_vec_y, m_R_arr[measurement_type]); */
    /* mp_lkf_y->doCorrection(); */

    try {
      if (m_use_repredictor) {
        mp_rep_x->addMeasurement(z_x, R, meas_stamp, mp_lkf_vector[measurement_type]);
        sc_x = mp_rep_x->predictTo(predict_stamp);
        mp_rep_y->addMeasurement(z_y, R, meas_stamp, mp_lkf_vector[measurement_type]);
        sc_y = mp_rep_y->predictTo(predict_stamp);
      } else {
        sc_x = mp_lkf_x->correct(sc_x, z_x, R, measurement_type);
        sc_y = mp_lkf_y->correct(sc_y, z_y, R, measurement_type);
      }
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[Odometry]: LKF correction step failed: %s", e.what());
    }
  }

  /* if (measurement_type==1) { */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: correction type: " << measurement_type << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: corr  x:" << z_x << std::endl << "y: " << z_y << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: R: " << R << std::endl << std::endl); */
  /* ROS_INFO_STREAM_THROTTLE(1.0,  "[StateEstimator]: state  x:" << sc_x.x << std::endl << "y: " << sc_y.x << std::endl << std::endl); */
  /* } */

  return true;
}

//}

/*  //{ getStates() */

bool StateEstimator::getStates(LatState2D &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  //}

  std::scoped_lock lock(mutex_lkf);

  states.col(0) = sc_x.x;
  states.col(1) = sc_y.x;

  return true;
}

//}

/*  //{ getState() */

bool StateEstimator::getState(int state_id, Vec2 &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(state_id)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".getState(int state_id=" << state_id << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    /* std::cout << "[StateEstimator]: " << m_estimator_name << " getting value: " << mp_lkf_x->getState(state_id) << " of state: " << state_id << std::endl;
     */
    state(0) = sc_x.x(state_id);
    state(1) = sc_y.x(state_id);
  }

  return true;
}

//}

/*  //{ getName() */

std::string StateEstimator::getName(void) {
  return m_estimator_name;
}

//}

/*  //{ setState() */

bool StateEstimator::setState(int state_id, const Vec2 &state) {

  /*  //{ sanity checks */

  if (!m_is_initialized) {
    /* ROS_WARN("[%s]: Lateral estimator %s method setState() called before init.", ros::this_node::getName().c_str(), m_estimator_name.c_str()); */
    return false;
  }

  // Check the size of state
  if (state.size() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): wrong size of \"state.size()\". Should be: " << 2 << " is:" << state.size() << std::endl;
    return false;
  }

  // Check for NaNs
  if (!std::isfinite(state(0))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state(0)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state(1))) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state(1)\"." << std::endl;
    return false;
  }

  if (!std::isfinite(state_id)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): NaN detected in variable \"state_id\"." << std::endl;
    return false;
  }

  // Check validity of state_id
  if (state_id < 0 || state_id > m_n_states - 1) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setState(int state_id=" << state_id << ", const Eigen::VectorXd &state=" << state
              << "): Invalid value of \"state_id\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    sc_x.x(state_id) = state(0);
    sc_y.x(state_id) = state(1);
  }
  /* ROS_INFO("[%s]: Set state %d of %s to x: %f y: %f. State after x: %f y: %f", ros::this_node::getName().c_str(), state_id, m_estimator_name.c_str(),
   * state(0), state(1), sc_x.x(state_id), sc_y.x(state_id)); */

  return true;
}

//}

/*  //{ setStates() */

bool StateEstimator::setStates(LatState2D &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check size of states
  if (states.rows() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setStates()"
              << ": wrong size of \"states\". Should be: " << m_n_states << " is: " << states.cols() << std::endl;
    return false;
  }

  if (states.cols() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setStates()"
              << ": wrong size of \"states\". Should be: " << 2 << " is:" << states.rows() << std::endl;
    return false;
  }
  //}

  {
    std::scoped_lock lock(mutex_lkf);
    sc_x.x = states.col(0);
    sc_y.x = states.col(1);
  }

  return true;
}

//}

/*  //{ setR() */

bool StateEstimator::setR(double cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  /* double old_cov = m_R_arr[measurement_type](0, 0); */

  {
    std::scoped_lock lock(mutex_lkf);

    m_R_arr[measurement_type](0, 0) = cov;
  }

  /* std::cout << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_R_arr[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ getR() */

bool StateEstimator::getR(double &cov, int measurement_type) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(measurement_type)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): NaN detected in variable \"measurement_type\"." << std::endl;
    return false;
  }

  // Check for invalid measurement type
  if (measurement_type > (int)m_fusing_measurement.size() || measurement_type < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".getCovariance(int measurement_type=" << measurement_type
              << "): invalid value of \"measurement_type\"." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_R_arr[measurement_type](0, 0);
  }

  return true;
}

//}

/*  //{ getQ() */

bool StateEstimator::getQ(double &cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    cov = m_Q(idx(0), idx(1));
  }

  return true;
}

//}

/*  //{ setQ() */

bool StateEstimator::setQ(double cov, const Eigen::Vector2i &idx) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;

  // Check for NaNs
  if (!std::isfinite(cov)) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): NaN detected in variable \"cov\"." << std::endl;
    return false;
  }

  // Check for non-positive covariance
  if (cov <= 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"cov\" should be > 0." << std::endl;
    return false;
  }

  // Check for index validity
  if (idx(0) > m_n_states || idx(1) > m_n_states || idx(0) < 0 || idx(1) < 0) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".setR(double cov=" << cov << ", int"
              << "): \"idx\" should be < " << m_n_states << "." << std::endl;
    return false;
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    m_Q(idx(0), idx(1)) = cov;
  }

  /* std::cout << "[StateEstimator]: " << m_estimator_name << ".setCovariance(double cov=" << cov << ", int measurement_type=" << measurement_type << ")" */
  /* << " Changed covariance from: " << old_cov << " to: " << m_Q_arr[measurement_type](0, 0) << std::endl; */

  return true;
}

//}

/*  //{ reset() */

bool StateEstimator::reset(const LatState2D &states) {

  /*  //{ sanity checks */

  if (!m_is_initialized)
    return false;
  // Check size of states
  if ((int)states.rows() != m_n_states) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.rows()\". Should be: " << m_n_states << " is:" << states.rows() << std::endl;
    return false;
  }

  if (states.cols() != 2) {
    std::cerr << "[StateEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
              << "): wrong size of \"states.cols()\". Should be: " << 2 << " is:" << states.cols() << std::endl;
    return false;
  }

  // Check for NaNs
  for (int i = 0; i < states.rows(); i++) {
    for (int j = 0; j < states.cols(); j++) {
      if (!std::isfinite(states(i, j))) {
        std::cerr << "[StateEstimator]: " << m_estimator_name << ".reset(const Eigen::MatrixXd &states="  // << states
                  << "): NaN detected in variable \"states(" << i << ", " << j << ")\"." << std::endl;
        return false;
      }
    }
  }

  //}

  {
    std::scoped_lock lock(mutex_lkf);

    sc_x.x = states.col(0);
    sc_y.x = states.col(1);

    if (m_use_repredictor) {
      const u_t       u0 = u_t::Zero();
      const ros::Time t0 = ros::Time(0);
      mp_rep_x           = std::make_unique<rep_lat_t>(sc_x.x, sc_x.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
      mp_rep_y           = std::make_unique<rep_lat_t>(sc_y.x, sc_y.P, u0, m_Q, t0, mp_lkf_vector.at(0), m_buf_sz);
    }
  }

  return true;
}

//}

}  // namespace mrs_uav_odometry
