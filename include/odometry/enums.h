#ifndef ODOMETRY_ENUMS
#define ODOMETRY_ENUMS

#include <cassert>
#include <fog_msgs/msg/odometry_state.hpp>
#include <fog_msgs/msg/estimator_state.hpp>

namespace odometry2
{
#define ODOM_ERR_MSG "Inconsistent message received, please rebuild the package that crashed!"

/* odometry_state_t enumeration type //{ */

enum struct odometry_state_t
{
  invalid,
  not_connected,
  init,
  switching,
  gps,
  hector,
  missing_odometry,
  landed,
};


static inline odometry_state_t to_enum(const fog_msgs::msg::OdometryState msg) {
  switch (msg.state) {
    case fog_msgs::msg::OdometryState::NOT_CONNECTED:
      return odometry_state_t::not_connected;
    case fog_msgs::msg::OdometryState::INIT:
      return odometry_state_t::init;
    case fog_msgs::msg::OdometryState::SWITCHING:
      return odometry_state_t::switching;
    case fog_msgs::msg::OdometryState::GPS:
      return odometry_state_t::gps;
    case fog_msgs::msg::OdometryState::HECTOR:
      return odometry_state_t::hector;
    case fog_msgs::msg::OdometryState::MISSING_ODOMETRY:
      return odometry_state_t::missing_odometry;
    case fog_msgs::msg::OdometryState::LANDED:
      return odometry_state_t::landed;
    default:
      assert(false && ODOM_ERR_MSG);
      return odometry_state_t::invalid;
  }
}

static inline fog_msgs::msg::OdometryState to_msg(const odometry_state_t enum_val) {
  fog_msgs::msg::OdometryState msg;
  switch (enum_val) {
    case odometry_state_t::not_connected:
      msg.state = fog_msgs::msg::OdometryState::NOT_CONNECTED;
      break;
    case odometry_state_t::init:
      msg.state = fog_msgs::msg::OdometryState::INIT;
      break;
    case odometry_state_t::switching:
      msg.state = fog_msgs::msg::OdometryState::SWITCHING;
      break;
    case odometry_state_t::gps:
      msg.state = fog_msgs::msg::OdometryState::GPS;
      break;
    case odometry_state_t::hector:
      msg.state = fog_msgs::msg::OdometryState::HECTOR;
      break;
    case odometry_state_t::missing_odometry:
      msg.state = fog_msgs::msg::OdometryState::MISSING_ODOMETRY;
      break;
    case odometry_state_t::landed:
      msg.state = fog_msgs::msg::OdometryState::LANDED;
      break;
    default:
      assert(false && ODOM_ERR_MSG);
      msg.state = fog_msgs::msg::OdometryState::INVALID;
      break;
  }
  return msg;
}

static inline std::string to_string(const odometry_state_t enum_val) {
  switch (enum_val) {
    case odometry_state_t::not_connected:
      return "not_connected";
    case odometry_state_t::init:
      return "init";
    case odometry_state_t::switching:
      return "switching";
    case odometry_state_t::gps:
      return "gps";
    case odometry_state_t::hector:
      return "hector";
    case odometry_state_t::missing_odometry:
      return "missing_odometry";
    case odometry_state_t::landed:
      return "landed";
    default:
      assert(false && ODOM_ERR_MSG);
      return "invalid";
  }
}

//}

/* estimator_state_t enumeration type //{ */

enum struct estimator_state_t
{
  invalid,
  init,
  reliable,
  not_reliable,
  restart,
  inactive,
};


static inline estimator_state_t to_enum(const fog_msgs::msg::EstimatorState msg) {
  switch (msg.state) {
    case fog_msgs::msg::EstimatorState::INIT:
      return estimator_state_t::init;
    case fog_msgs::msg::EstimatorState::RELIABLE:
      return estimator_state_t::reliable;
    case fog_msgs::msg::EstimatorState::NOT_RELIABLE:
      return estimator_state_t::not_reliable;
    case fog_msgs::msg::EstimatorState::RESTART:
      return estimator_state_t::restart;
    case fog_msgs::msg::EstimatorState::INACTIVE:
      return estimator_state_t::inactive;
    default:
      assert(false && ODOM_ERR_MSG);
      return estimator_state_t::invalid;
  }
}

static inline fog_msgs::msg::EstimatorState to_msg(const estimator_state_t enum_val) {
  fog_msgs::msg::EstimatorState msg;
  switch (enum_val) {
    case estimator_state_t::init:
      msg.state = fog_msgs::msg::EstimatorState::INIT;
      break;
    case estimator_state_t::reliable:
      msg.state = fog_msgs::msg::EstimatorState::RELIABLE;
      break;
    case estimator_state_t::not_reliable:
      msg.state = fog_msgs::msg::EstimatorState::NOT_RELIABLE;
      break;
    case estimator_state_t::restart:
      msg.state = fog_msgs::msg::EstimatorState::RESTART;
      break;
    case estimator_state_t::inactive:
      msg.state = fog_msgs::msg::EstimatorState::INACTIVE;
      break;
    default:
      assert(false && ODOM_ERR_MSG);
      msg.state = fog_msgs::msg::EstimatorState::INVALID;
      break;
  }
  return msg;
}

static inline std::string to_string(const estimator_state_t enum_val) {
  switch (enum_val) {
    case estimator_state_t::init:
      return "init";
    case estimator_state_t::reliable:
      return "reliable";
    case estimator_state_t::not_reliable:
      return "not_reliable";
    case estimator_state_t::restart:
      return "restart";
    case estimator_state_t::inactive:
      return "inactive";
    default:
      assert(false && ODOM_ERR_MSG);
      return "invalid";
  }
}

//}

}  // namespace odometry2

#endif  // ODOMETRY_ENUMS
