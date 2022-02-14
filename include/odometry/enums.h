#ifndef ODOMETRY_ENUMS
#define ODOMETRY_ENUMS

#include <cassert>
#include <fog_msgs/msg/odometry_vehicle_state.hpp>
#include <fog_msgs/msg/odometry_estimator_state.hpp>

namespace odometry
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
};


static inline odometry_state_t to_enum(const fog_msgs::msg::OdometryVehicleState msg) {
  switch (msg.state) {
    case fog_msgs::msg::OdometryVehicleState::NOT_CONNECTED:
      return odometry_state_t::not_connected;
    case fog_msgs::msg::OdometryVehicleState::INIT:
      return odometry_state_t::init;
    case fog_msgs::msg::OdometryVehicleState::SWITCHING:
      return odometry_state_t::switching;
    case fog_msgs::msg::OdometryVehicleState::GPS:
      return odometry_state_t::gps;
    case fog_msgs::msg::OdometryVehicleState::HECTOR:
      return odometry_state_t::hector;
    case fog_msgs::msg::OdometryVehicleState::MISSING_ODOMETRY:
      return odometry_state_t::missing_odometry;
    default:
      assert(false && ERR_MSG);
      return odometry_state_t::invalid;
  }
}

static inline fog_msgs::msg::OdometryVehicleState to_msg(const odometry_state_t enum_val) {
  fog_msgs::msg::OdometryVehicleState msg;
  switch (enum_val) {
    case odometry_state_t::not_connected:
      msg.state = fog_msgs::msg::OdometryVehicleState::NOT_CONNECTED;
      break;
    case odometry_state_t::init:
      msg.state = fog_msgs::msg::OdometryVehicleState::INIT;
      break;
    case odometry_state_t::switching:
      msg.state = fog_msgs::msg::OdometryVehicleState::SWITCHING;
      break;
    case odometry_state_t::gps:
      msg.state = fog_msgs::msg::OdometryVehicleState::GPS;
      break;
    case odometry_state_t::hector:
      msg.state = fog_msgs::msg::OdometryVehicleState::HECTOR;
      break;
    case odometry_state_t::missing_odometry:
      msg.state = fog_msgs::msg::OdometryVehicleState::MISSING_ODOMETRY;
      break;
    default:
      assert(false && ERR_MSG);
      msg.state = fog_msgs::msg::OdometryVehicleState::INVALID;
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
    default:
      assert(false && ERR_MSG);
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


static inline estimator_state_t to_enum(const fog_msgs::msg::OdometryEstimatorState msg) {
  switch (msg.state) {
    case fog_msgs::msg::OdometryEstimatorState::INIT:
      return estimator_state_t::init;
    case fog_msgs::msg::OdometryEstimatorState::RELIABLE:
      return estimator_state_t::reliable;
    case fog_msgs::msg::OdometryEstimatorState::NOT_RELIABLE:
      return estimator_state_t::not_reliable;
    case fog_msgs::msg::OdometryEstimatorState::RESTART:
      return estimator_state_t::restart;
    case fog_msgs::msg::OdometryEstimatorState::INACTIVE:
      return estimator_state_t::inactive;
    default:
      assert(false && ERR_MSG);
      return estimator_state_t::invalid;
  }
}

static inline fog_msgs::msg::OdometryEstimatorState to_msg(const estimator_state_t enum_val) {
  fog_msgs::msg::OdometryEstimatorState msg;
  switch (enum_val) {
    case estimator_state_t::init:
      msg.state = fog_msgs::msg::OdometryEstimatorState::INIT;
      break;
    case estimator_state_t::reliable:
      msg.state = fog_msgs::msg::OdometryEstimatorState::RELIABLE;
      break;
    case estimator_state_t::not_reliable:
      msg.state = fog_msgs::msg::OdometryEstimatorState::NOT_RELIABLE;
      break;
    case estimator_state_t::restart:
      msg.state = fog_msgs::msg::OdometryEstimatorState::RESTART;
      break;
    case estimator_state_t::inactive:
      msg.state = fog_msgs::msg::OdometryEstimatorState::INACTIVE;
      break;
    default:
      assert(false && ERR_MSG);
      msg.state = fog_msgs::msg::OdometryEstimatorState::INVALID;
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
      assert(false && ERR_MSG);
      return "invalid";
  }
}

//}

}  // namespace odometry

#endif  // ODOMETRY_ENUMS
