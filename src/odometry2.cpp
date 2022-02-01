/* includes//{*/

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>

#include <fog_msgs/msg/control_interface_diagnostics.hpp>

#include <fog_lib/scope_timer.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'

#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>/*//}*/

typedef std::tuple<std::string, int>   px4_int;
typedef std::tuple<std::string, float> px4_float;


using namespace std::placeholders;
using namespace fog_lib;

namespace odometry2
{

enum struct odometry_state_t
{
  invalid,
  not_connected,
  gps,
  hector,
  missing_odometry,
};

/* class Odometry2 //{ */
class Odometry2 : public rclcpp::Node {
public:
  Odometry2(rclcpp::NodeOptions options);

private:
  std::atomic_bool is_initialized_ = false;

  // | --------------------- Odometry states -------------------- |
  std::recursive_mutex state_mutex_;
  odometry_state_t     odometry_state_ = odometry_state_::not_connected

      // | ----------------- Scope timer parameters ----------------- |
      bool         scope_timer_enable_   = false;
  rclcpp::Duration scope_timer_throttle_ = rclcpp::Duration(std::chrono::seconds(1));
  rclcpp::Duration scope_timer_min_dur_  = rclcpp::Duration::from_seconds(0.020);

  // | ------------------------ TF frames ----------------------- |
  std::string uav_name_         = "";
  std::string world_frame_      = "";
  std::string ned_origin_frame_ = "";
  std::string frd_fcu_frame_    = "";
  std::string fcu_frame_        = "";

  // | ---------------------- TF variables ---------------------- |
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  geometry_msgs::msg::TransformStamped                 tf_world_to_ned_origin_frame_;

  // | ---------------------- PX parameters --------------------- |
  std::vector<px4_int>     px4_params_int_;
  std::vector<px4_float>   px4_params_float_;
  std::vector<std::future> future_responses_;
  std::atomic_bool         set_initial_px4_params_                = false;
  bool                     param_upload_success_                  = true;
  std::atomic_bool         getting_pixhawk_odom_                  = false;
  std::atomic_bool         getting_control_interface_diagnostics_ = false;
  std::mutex               param_loaded_mutex_;

  float      px4_position_[3];
  float      px4_orientation_[4];
  std::mutex px4_pose_mutex_;

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr            local_odom_publisher_;
  rclcpp::Publisher<fog_msgs::msg::OdometryDiagnostics>::SharedPtr diagnostics_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr             pixhawk_odom_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_interface_diagnostics_subscriber_;

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void ControlInterfaceDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);

  // | ---------------- Service clients handlers ---------------- |
  bool setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future);
  bool setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future);

  // | ------------------- Internal functions ------------------- |
  bool setInitialPx4Params();

  void publishStaticTF();
  void publishLocalOdomAndTF();
  void publishDiagnostics();

  void update_odometry_state();

  void state_odometry_not_connected();
  void state_odometry_init();
  void state_odometry_not_connected();
  void state_odometry_not_connected();
  void state_odometry_not_connected();

  // | -------------------- Routine handling -------------------- |
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     odometry_timer_;
  double                           odometry_loop_rate_;

  void odometryRoutine(void);

  // utils
  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest);
};
//}

/* constructor //{ */
Odometry2::Odometry2(rclcpp::NodeOptions options) : Node("odometry2", options) {

  RCLCPP_INFO(get_logger(), "[%s]: Initializing...", get_name());

  // Getting
  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_WARN(get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", get_name());
  }
  RCLCPP_INFO(get_logger(), "[%s]: UAV name is: '%s'", get_name(), uav_name_.c_str());

  /* parse general params from config file //{ */
  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  loaded_successfully &= parse_param("odometry_loop_rate", odometry_loop_rate_);

  int   param_int;
  float param_float;

  loaded_successfully &= parse_param("px4.EKF2_AID_MASK", param_int);
  px4_params_int_.push_back(px4_int("EKF2_AID_MASK", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_AID", param_int);
  px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));

  loaded_successfully &= parse_param("px4.EKF2_HGT_MODE", param_int);
  px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_A_HMAX", param_float);
  px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));

  loaded_successfully &= parse_param("world_frame", world_frame_);

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(get_logger(), "%s", str.c_str());
    rclcpp::shutdown();
    return;
  }
  //}

  /* frame definition */
  fcu_frame_        = uav_name_ + "/fcu";         // FLU frame (Front-Left-Up) match also ENU frame (East-North-Up)
  frd_fcu_frame_    = uav_name_ + "/frd_fcu";     // FRD frame (Front-Right-Down)
  ned_origin_frame_ = uav_name_ + "/ned_origin";  // NED frame (North-East-Down)

  // | ----------------------- Publishers ----------------------- |
  rclcpp::QoS qos(rclcpp::KeepLast(3));
  local_odom_publisher_  = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", qos);
  diagnostics_publisher_ = this->create_publisher<fog_msgs::msg::OdometryDiagnostics>("~/diagnostics_out", qos);

  // subscribers
  pixhawk_odom_subscriber_                  = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  control_interface_diagnostics_subscriber_ = this->create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
      "~/control_interface_diagnostics_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::ControlInterfaceDiagnosticsCallback, this, _1));

  // service clients
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate_), std::bind(&Odometry2::odometryRoutine, this), callback_group_);

  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "[%s]: Initialized", get_name());
}
//}

/* pixhawkOdomCallback //{ */
void Odometry2::pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
  std::scoped_lock lock(px4_pose_mutex_);

  if (!is_initialized_) {
    return;
  }

  px4_position_[0]    = msg->x;
  px4_position_[1]    = msg->y;
  px4_position_[2]    = msg->z;
  px4_orientation_[0] = msg->q[0];  // q.w
  px4_orientation_[1] = msg->q[1];  // q.x
  px4_orientation_[2] = msg->q[2];  // q.y
  px4_orientation_[3] = msg->q[3];  // q.z

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "[%s]: Getting pixhawk odometry!", get_name());
}
//}

/* ControlInterfaceDiagnosticsCallback //{ */
void Odometry2::ControlInterfaceDiagnosticsCallback([[maybe_unused]] const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg) {

  // Check if the Control Interface is in the valid state to cooperate with
  if (msg->vehicle_state.state == fog_msgs::msg::ControlInterfaceVehicleState::NOT_CONNECTED ||
      msg->vehicle_state.state == fog_msgs::msg::ControlInterfaceVehicleState::INVALID) {
    return;
  }

  getting_control_interface_diagnostics_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "[%s]: Getting control diagnostics!", get_name());
}
//}

/* setPx4ParamIntCallback //{ */
bool Odometry2::setPx4IntParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response> result = future.get();

  // Check if all parameters were loaded successfully
  std::scoped_lock lock(param_loaded_mutex_);
  param_upload_success_ = param_upload_success_ && result->success;

  if (result->success) {
    RCLCPP_INFO(get_logger(), "[%s]: Parameter %s has been set to value: %ld", get_name(), result->param_name.c_str(), result->value);
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[%s]: Cannot set the parameter %s with message: %s", get_name(), result->param_name.c_str(), result->message.c_str());
    return false;
  }
}
//}

/* setPx4ParamFloatCallback //{ */
bool Odometry2::setPx4FloatParamCallback(rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Response> result = future.get();

  // Check if all parameters were loaded successfully
  std::scoped_lock lock(param_loaded_mutex_);
  param_upload_success_ = param_upload_success_ && result->success;

  if (result->success) {
    RCLCPP_INFO(get_logger(), "[%s]: Parameter %s has been set to value: %f", get_name(), result->param_name.c_str(), result->value);
    return true;
  } else {
    RCLCPP_ERROR(get_logger(), "[%s]: Cannot set the parameter %s with message: %s", get_name(), result->param_name.c_str(), result->message.c_str());
    return false;
  }
}
//}

// --------------------------------------------------------------
// |                      Odometry routines                     |
// --------------------------------------------------------------

/* odometryRoutine//{ */
void Odometry2::odometryRoutine() {
  scope_timer      tim(scope_timer_enable_, "odometryRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(state_mutex_);

  const auto prev_state = odometry_state_;
  update_odometry_state();
  if (prev_state != odometry_state_)
    publishDiagnostics();
}
//}

/* diagnosticsRoutine(); //{ */
void Odometry2::diagnosticsRoutine() {
  scope_timer      tim(scope_timer_enable_, "diagnosticsRoutine", get_logger(), scope_timer_throttle_, scope_timer_min_dur_);
  std::scoped_lock lock(state_mutex_);

  // publish some diags
  publishDiagnostics();
}
//}

/* update_odometry_state//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::update_odometry_state() {

  // process the vehicle's state
  switch (odometry_state_) {
    case odometry_state_t::not_connected:
      state_odometry_not_connected();
      break;
    case odometry_state_t::gps:
      state_odometry_gps();
      break;
    case odometry_state_t::hector:
      state_odometry_hector();
      break;
    case odometry_state_t::missing_odometry:
      state_odometry_missing_odometry();
      break;
    default:
      assert(false);
      RCLCPP_ERROR(get_logger(), "Invalid odometry state, this should never happen!");
      return;
  }
}
//}


/* state_odometry_not_connected//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_not_connected() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Waiting to initialize Pixhawk and Control Interface.");

  if (getting_pixhawk_odom_ && getting_control_interface_diagnostics_) {
    RCLCPP_INFO(get_logger(), "[%s]: Pixhawk and Control Interface is ready", get_name());
    odometry_state_ = odometry_state_t::init;
  }
}
//}

/* state_odometry_init//{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::state_odometry_init() {

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry state: Initialize odometry module.");

  if (!set_initial_px4_params_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Setting initial PX4 parameters.");
    if (setInitialPx4Params()) {
      set_initial_px4_params_ = true;
    }
  }

  // Check if all PX4 parameters were set successfully
  for (const std::future response : future_responses_) {
    if (response.wait_for(set_initial_px4_params_timeout_) == std::future_status::timeout) {
      RCLCPP_ERROR(get_logger(), "[%s]: Could not set param '%s'", get_name(), param_name.c_str());

    } else {
      set_initial_px4_params_ = true;
    }
  }

  if (static_tf_broadcaster_ == nullptr) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
    publishStaticTF();
  }

  publishLocalOdomAndTF();
  else {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[%s]: Getting PX4 odometry: %s, Getting control_interface diagnostics: %s", get_name(),
                         getting_pixhawk_odom_.load() ? "TRUE" : "FALSE", getting_control_interface_diagnostics_ ? "TRUE" : "FALSE");
  }
}
//}

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {
  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion                      q;

  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = frd_fcu_frame_;
  tf.child_frame_id          = fcu_frame_;
  tf.transform.translation.x = 0;
  tf.transform.translation.y = 0;
  tf.transform.translation.z = 0;
  q.setRPY(-M_PI, 0, 0);
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  tf.header.frame_id         = world_frame_;
  tf.child_frame_id          = ned_origin_frame_;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  q.setRPY(M_PI, 0, M_PI_2);
  q                       = q.inverse();
  tf.transform.rotation.x = q.getX();
  tf.transform.rotation.y = q.getY();
  tf.transform.rotation.z = q.getZ();
  tf.transform.rotation.w = q.getW();
  v_transforms.push_back(tf);

  tf_world_to_ned_origin_frame_ = tf;

  static_tf_broadcaster_->sendTransform(v_transforms);
}
//}

/* publishLocalOdomAndTF //{ */
void Odometry2::publishLocalOdomAndTF() {
  std::scoped_lock lock(px4_pose_mutex_);

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp            = this->get_clock()->now();
  tf.header.frame_id         = ned_origin_frame_;
  tf.child_frame_id          = frd_fcu_frame_;
  tf.transform.translation.x = px4_position_[0];
  tf.transform.translation.y = px4_position_[1];
  tf.transform.translation.z = px4_position_[2];
  tf.transform.rotation.w    = px4_orientation_[0];
  tf.transform.rotation.x    = px4_orientation_[1];
  tf.transform.rotation.y    = px4_orientation_[2];
  tf.transform.rotation.z    = px4_orientation_[3];
  tf_broadcaster_->sendTransform(tf);

  // frd -> flu (enu) is rotation 180 degrees around x
  tf2::Quaternion q_orig, q_rot, q_new;

  q_orig.setW(px4_orientation_[0]);
  q_orig.setX(px4_orientation_[1]);
  q_orig.setY(px4_orientation_[2]);
  q_orig.setZ(px4_orientation_[3]);

  q_rot.setRPY(M_PI, 0, 0);
  q_new = q_orig * q_rot;
  q_new.normalize();

  geometry_msgs::msg::PoseStamped pose_ned, pose_enu;

  pose_ned.pose.position.x = px4_position_[0];
  pose_ned.pose.position.y = px4_position_[1];
  pose_ned.pose.position.z = px4_position_[2];
  tf2::convert(q_new, pose_ned.pose.orientation);

  tf2::doTransform(pose_ned, pose_enu, tf_world_to_ned_origin_frame_);

  nav_msgs::msg::Odometry msg;

  msg.header.stamp    = this->get_clock()->now();
  msg.header.frame_id = world_frame_;
  msg.child_frame_id  = frd_fcu_frame_;
  msg.pose.pose       = pose_enu.pose;

  local_odom_publisher_->publish(msg);
}
//}

/*setInitialPx4Params//{*/
bool Odometry2::setInitialPx4Params() {

  if (!uploadPx4Parameters(fog_msgs::srv::SetPx4ParamInt::Request, px4_params_int_, set_px4_param_int_) ||
      !uploadPx4Parameters(fog_msgs::srv::SetPx4ParamFloat::Request, px4_params_float, set_px4_param_float_)) {
    return false;
  }
  return true;
}  // namespace odometry2

/*//}*/

// | ----------------------- Diagnostics ---------------------- |

/* publishDiagnostics //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
void Odometry2::publishDiagnostics() {
  fog_msgs::msg::OdometryDiagnostics msg;

  msg.header.stamp    = get_clock()->now();
  msg.header.frame_id = world_frame_;

  msg.getting_pixhawk_odom      = getting_pixhawk_odom_;
  msg.getting_control_interface = getting_control_interface_;
  /* msg.vehicle_state = to_msg(vehicle_state_); */
  /* msg.mission_state = to_msg(mission_mgr_ ? mission_mgr_->state() : mission_state_t::finished); */

  /* msg.mission_id       = mission_mgr_ ? mission_mgr_->mission_id() : 0; */
  /* msg.mission_size     = mission_mgr_ ? mission_mgr_->mission_size() : 0; */
  /* msg.mission_waypoint = mission_mgr_ ? mission_mgr_->mission_waypoint() : 0; */

  /* msg.gps_origin_set       = gps_origin_set_; */
  /* msg.getting_odom         = getting_odom_; */
  /* msg.getting_control_mode = getting_control_mode_; */

  diagnostics_publisher_->publish(msg);
}
//}


// | -------------------------- Utils ------------------------- |

/* uploadPx4Parameters //{ */
template <typename T, typename U, typename V>
bool uploadPx4Parameters(const T &service_name, const U &param_array, const V &service_client) {
  for (const auto item : param_array) {
    auto request        = std::make_shared<service_name>();
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(get_logger(), "[%s]: Setting %s, value: %f", get_name(), std::get<0>(item).c_str(), std::get<1>(item));
    auto future_response = service_client->async_send_request(request);

    // If parameter was not set, return false and repeat
    if (!checkPx4ParamOutput(future_response)) {
      return false;
    }
  }
  return true;
}
//}

/* checkPx4ParamOutput //{ */
template <typename T>
bool checkPx4ParamOutput(const std::shared_future<T> &f) {
  assert(f.valid());
  if (f.wait_for(set_initial_px4_params_timeout_) == std::future_status::timeout || f.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "[%s]: Cannot set the parameter %s with message: %s", get_name(), f.get()->param_name.c_str(), f.get()->message.c_str());
    return false;
  } else {
    RCLCPP_INFO(get_logger(), "[%s]: Parameter %s has been set to value: %ld", get_name(), f.get()->param_name.c_str(), f.get()->value);
    return true;
  }
}
//}

/* parse_param //{ */
template <class T>
bool Odometry2::parse_param(const std::string &param_name, T &param_dest) {
  this->declare_parameter(param_name);
  if (!this->get_parameter(param_name, param_dest)) {
    RCLCPP_ERROR(get_logger(), "[%s]: Could not load param '%s'", get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "[" << get_name() << "]: Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}
//}

}  // namespace odometry2


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odometry2::Odometry2)
