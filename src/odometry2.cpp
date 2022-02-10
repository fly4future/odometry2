#include <mutex>
#include <chrono>
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
#include <fog_msgs/msg/heading.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/home_position.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/string.hpp>

#include <fog_lib/params.h>
#include <fog_lib/gps_conversions.h>
#include <fog_lib/geometry/misc.h>

using namespace std::placeholders;
using namespace fog_lib;
using namespace fog_lib::geometry;

namespace odometry2
{

typedef std::tuple<std::string, int>   px4_int;
typedef std::tuple<std::string, float> px4_float;

/* class Odometry2 //{ */
class Odometry2 : public rclcpp::Node {
public:
  Odometry2(rclcpp::NodeOptions options);

private:
  std::atomic_bool is_initialized_ = false;

  // | ------------------------ TF frames ----------------------- |
  std::string uav_name_           = "";
  std::string utm_origin_frame_   = "";
  std::string local_origin_frame_ = "";
  std::string ned_origin_frame_   = "";
  std::string frd_fcu_frame_      = "";
  std::string fcu_frame_          = "";

  // | ---------------------- TF variables ---------------------- |
  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  geometry_msgs::msg::TransformStamped                 tf_local_origin_to_ned_origin_frame_;

  // | ---------------------- PX parameters --------------------- |
  std::vector<px4_int>   px4_params_int_;
  std::vector<px4_float> px4_params_float_;
  std::atomic_bool       set_initial_px4_params_                = false;
  std::atomic_bool       getting_pixhawk_odom_                  = false;
  std::atomic_bool       getting_px4_utm_position_              = false;
  std::atomic_bool       getting_control_interface_diagnostics_ = false;
  int                    px4_param_set_timeout_                 = 0;

  float      px4_position_[3];
  float      px4_orientation_[4];
  std::mutex px4_pose_mutex_;

  double           px4_utm_position_[2];
  std::atomic_bool republish_static_tf_ = false;
  std::mutex       px4_utm_position_mutex_;

  px4_msgs::msg::HomePosition  px4_home_position_;
  rclcpp::TimerBase::SharedPtr home_position_timer_;
  std::mutex                   px4_home_position_mutex_;

  // | ----------------------- Publishers ----------------------- |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr     local_odom_publisher_;
  rclcpp::Publisher<px4_msgs::msg::HomePosition>::SharedPtr home_position_publisher_;
  rclcpp::Publisher<fog_msgs::msg::Heading>::SharedPtr      heading_publisher_;

  // | ----------------------- Subscribers ---------------------- |
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr             pixhawk_odom_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr                home_position_subscriber_;
  rclcpp::Subscription<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr control_interface_diagnostics_subscriber_;

  // callback groups
  // a shared pointer to each callback group has to be saved or the callbacks will never get called
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  // new callback groups have to be initialized using this function to be saved into callback_groups_
  rclcpp::CallbackGroup::SharedPtr new_cbk_grp();

  // | --------------------- Service clients -------------------- |
  rclcpp::Client<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;

  // | ------------------ Subscriber callbacks ------------------ |
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg);
  void ControlInterfaceDiagnosticsCallback(const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg);

  // | ------------------- Internal functions ------------------- |

  void publishStaticTF();
  void publishLocalOdomAndTF();

  // | -------------------- Routine handling -------------------- |
  rclcpp::TimerBase::SharedPtr odometry_timer_;

  void odometryRoutine(void);
  void homePositionPublisher(void);

  // | --------------------- Utils function --------------------- |

  bool setInitialPx4Params();

  template <typename T, typename U, typename V>
  bool uploadPx4Parameters(const std::shared_ptr<T> &request, const std::vector<U> &param_array, const V &service_client);

  template <typename T>
  bool checkPx4ParamSetOutput(const std::shared_future<T> &f);
};
//}

/* constructor //{ */
Odometry2::Odometry2(rclcpp::NodeOptions options) : Node("odometry2", options) {

  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());

  // Getting
  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", this->get_name());
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: UAV name is: '%s'", this->get_name(), uav_name_.c_str());

  /* parse general params from config file //{ */
  RCLCPP_INFO(this->get_logger(), "-------------- Loading parameters --------------");
  bool loaded_successfully = true;

  double odometry_loop_rate;
  double home_position_publish_rate;

  loaded_successfully &= parse_param("odometry_loop_rate", odometry_loop_rate, *this);
  loaded_successfully &= parse_param("home_position_publish_rate", home_position_publish_rate, *this);
  loaded_successfully &= parse_param("px4.param_set_timeout", px4_param_set_timeout_, *this);


  int   param_int;
  float param_float;

  loaded_successfully &= parse_param("px4.EKF2_AID_MASK", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_AID_MASK", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_AID", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_RNG_AID", param_int));

  loaded_successfully &= parse_param("px4.EKF2_HGT_MODE", param_int, *this);
  px4_params_int_.push_back(px4_int("EKF2_HGT_MODE", param_int));

  loaded_successfully &= parse_param("px4.EKF2_RNG_A_HMAX", param_float, *this);
  px4_params_float_.push_back(px4_float("EKF2_RNG_A_HMAX", param_float));

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(this->get_logger(), "%s", str.c_str());
    rclcpp::shutdown();
    return;
  }
  //}

  /* frame definition */
  utm_origin_frame_   = "utm_origin";                 // ENU frame (East-North-Up)
  local_origin_frame_ = uav_name_ + "/local_origin";  // ENU frame (East-North-Up)
  fcu_frame_          = uav_name_ + "/fcu";           // FLU frame (Front-Left-Up)
  frd_fcu_frame_      = uav_name_ + "/frd_fcu";       // FRD frame (Front-Right-Down)
  ned_origin_frame_   = uav_name_ + "/ned_origin";    // NED frame (North-East-Down)

  // publishers
  local_odom_publisher_    = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);
  home_position_publisher_ = this->create_publisher<px4_msgs::msg::HomePosition>("~/home_position_out", 10);
  heading_publisher_ = this->create_publisher<fog_msgs::msg::Heading>("~/heading_out", 10);

  // subscribers
  pixhawk_odom_subscriber_                  = this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&Odometry2::pixhawkOdomCallback, this, _1));
  home_position_subscriber_                 = this->create_subscription<px4_msgs::msg::HomePosition>("~/home_position_in", rclcpp::SystemDefaultsQoS(),
                                                                                     std::bind(&Odometry2::homePositionCallback, this, _1));
  control_interface_diagnostics_subscriber_ = this->create_subscription<fog_msgs::msg::ControlInterfaceDiagnostics>(
      "~/control_interface_diagnostics_in", rclcpp::SystemDefaultsQoS(), std::bind(&Odometry2::ControlInterfaceDiagnosticsCallback, this, _1));

  // service clients
  set_px4_param_int_   = this->create_client<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int");
  set_px4_param_float_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");

  odometry_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / odometry_loop_rate), std::bind(&Odometry2::odometryRoutine, this), new_cbk_grp());
  home_position_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / home_position_publish_rate),
                                                 std::bind(&Odometry2::homePositionPublisher, this), new_cbk_grp());

  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
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
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting pixhawk odometry!", this->get_name());
}
//}

/* homePositionCallback //{ */
void Odometry2::homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(px4_home_position_mutex_);

    px4_home_position_ = *msg;
  }

  double out_x, out_y;

  UTM(msg->lat, msg->lon, &out_x, &out_y);

  if (!std::isfinite(out_x)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: NaN detected in UTM variable \"out_x\"!!!", this->get_name());
    return;
  }

  if (!std::isfinite(out_y)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: NaN detected in UTM variable \"out_y\"!!!", this->get_name());
    return;
  }

  {
    std::scoped_lock lock(px4_utm_position_mutex_);

    px4_utm_position_[0] = out_x;
    px4_utm_position_[1] = out_y;
    republish_static_tf_ = true;
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: GPS origin set! UTM x: %.2f, y: %.2f", this->get_name(), out_x, out_y);

  getting_px4_utm_position_ = true;
}
//}

/* ControlInterfaceDiagnosticsCallback //{ */
void Odometry2::ControlInterfaceDiagnosticsCallback([[maybe_unused]] const fog_msgs::msg::ControlInterfaceDiagnostics::UniquePtr msg) {

  if (!is_initialized_ || (msg->vehicle_state.state == fog_msgs::msg::ControlInterfaceVehicleState::NOT_CONNECTED)) {
    return;
  }

  getting_control_interface_diagnostics_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting control diagnostics!", this->get_name());

  // TODO: there is no proper way how to unsubscribe from a topic yet. This will at least delete subscriber.
  // However DDS layer is still receiving msgs until new timer/subsriber/service is registered.
  // See: https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/
  control_interface_diagnostics_subscriber_.reset();
}
//}

/* odometryRoutine //{ */
void Odometry2::odometryRoutine(void) {

  if (is_initialized_ && getting_pixhawk_odom_ && getting_control_interface_diagnostics_) {

    // Set and handle initial PX4 parameters setting
    if (!set_initial_px4_params_) {
      RCLCPP_INFO(get_logger(), "[%s]: Setting initial PX4 parameters", this->get_name());
      if (setInitialPx4Params()) {
        set_initial_px4_params_ = true;
      } else {
        RCLCPP_WARN(get_logger(), "[%s]: Fail to set all PX4 parameters. Will repeat in next cycle", this->get_name());
        return;
      }
    }

    if (static_tf_broadcaster_ == nullptr) {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
      publishStaticTF();
    }

    if (republish_static_tf_) {
      republish_static_tf_ = false;
      publishStaticTF();
    }

    publishLocalOdomAndTF();

    RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Everything ready -> Publishing odometry", this->get_name());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s]: Publishing odometry", this->get_name());

  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[%s]: Getting PX4 odometry: %s, Getting control_interface diagnostics: %s",
                         this->get_name(), getting_pixhawk_odom_ ? "TRUE" : "FALSE", getting_control_interface_diagnostics_ ? "TRUE" : "FALSE");
  }
}
//}

/* homePositionPublisher //{ */
void Odometry2::homePositionPublisher(void) {
  std::scoped_lock lock(px4_home_position_mutex_);
  home_position_publisher_->publish(px4_home_position_);
}
//}

/* publishStaticTF //{ */
void Odometry2::publishStaticTF() {

  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  std::scoped_lock lock(px4_utm_position_mutex_);

  geometry_msgs::msg::TransformStamped tf;
  tf2::Quaternion                      q;

  tf.header.stamp = this->get_clock()->now();

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

  tf.header.frame_id         = local_origin_frame_;
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

  tf_local_origin_to_ned_origin_frame_ = tf;

  tf.header.frame_id         = utm_origin_frame_;
  tf.child_frame_id          = local_origin_frame_;
  tf.transform.translation.x = px4_utm_position_[0];
  tf.transform.translation.y = px4_utm_position_[1];
  tf.transform.translation.z = 0;
  tf.transform.rotation.w    = 1;
  tf.transform.rotation.x    = 0;
  tf.transform.rotation.y    = 0;
  tf.transform.rotation.z    = 0;
  v_transforms.push_back(tf);

  static_tf_broadcaster_->sendTransform(v_transforms);
}
//}

/* publishLocalOdomAndTF //{ */
void Odometry2::publishLocalOdomAndTF() {
  std::scoped_lock lock(px4_pose_mutex_);

  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  std::vector<geometry_msgs::msg::TransformStamped> v_transforms;

  geometry_msgs::msg::TransformStamped tf;

  tf.header.stamp = this->get_clock()->now();

  tf.header.frame_id         = ned_origin_frame_;
  tf.child_frame_id          = frd_fcu_frame_;
  tf.transform.translation.x = px4_position_[0];
  tf.transform.translation.y = px4_position_[1];
  tf.transform.translation.z = px4_position_[2];
  tf.transform.rotation.w    = px4_orientation_[0];
  tf.transform.rotation.x    = px4_orientation_[1];
  tf.transform.rotation.y    = px4_orientation_[2];
  tf.transform.rotation.z    = px4_orientation_[3];
  v_transforms.push_back(tf);

  tf_broadcaster_->sendTransform(v_transforms);

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

  tf2::doTransform(pose_ned, pose_enu, tf_local_origin_to_ned_origin_frame_);

  // Local odometry message
  nav_msgs::msg::Odometry odom_msg;

  odom_msg.header.stamp    = this->get_clock()->now();
  odom_msg.header.frame_id = local_origin_frame_;
  odom_msg.child_frame_id  = frd_fcu_frame_;
  odom_msg.pose.pose       = pose_enu.pose;

  // Heading message
  fog_msgs::msg::Heading heading_msg;

  heading_msg.header.stamp = odom_msg.header.stamp;
  heading_msg.header.frame_id = local_origin_frame_;
  heading_msg.heading = quat2heading(pose_enu.pose.orientation);

  // Publish messages
  local_odom_publisher_->publish(odom_msg);
  heading_publisher_->publish(heading_msg);
}
//}

// | -------------------------- Utils ------------------------- |

/*setInitialPx4Params//{*/
bool Odometry2::setInitialPx4Params() {

  if (!uploadPx4Parameters(std::make_shared<fog_msgs::srv::SetPx4ParamInt::Request>(), px4_params_int_, set_px4_param_int_) ||
      !uploadPx4Parameters(std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Request>(), px4_params_float_, set_px4_param_float_)) {
    return false;
  }
  return true;
}
/*//}*/

/* uploadPx4Parameters //{ */
template <typename T, typename U, typename V>
bool Odometry2::uploadPx4Parameters(const std::shared_ptr<T> &request, const std::vector<U> &param_array, const V &service_client) {
  // Iterate over all parameters and try to set them
  for (const auto &item : param_array) {
    request->param_name = std::get<0>(item);
    request->value      = std::get<1>(item);
    RCLCPP_INFO(get_logger(), "[%s]: Setting PX4 parameter: %s to value: %f", get_name(), std::get<0>(item).c_str(), (float)std::get<1>(item));
    auto future_response = service_client->async_send_request(request);

    // If parameter was not set, return false
    if (!checkPx4ParamSetOutput(future_response)) {
      return false;
    }
  }
  return true;
}
//}

/* checkPx4ParamSetOutput //{ */
template <typename T>
bool Odometry2::checkPx4ParamSetOutput(const std::shared_future<T> &f) {
  assert(f.valid());
  if (f.wait_for(std::chrono::seconds(px4_param_set_timeout_)) == std::future_status::timeout || f.get() == nullptr) {
    RCLCPP_ERROR(get_logger(), "[%s]: Cannot set the parameter %s with message: %s", get_name(), f.get()->param_name.c_str(), f.get()->message.c_str());
    return false;
  } else {
    RCLCPP_INFO(get_logger(), "[%s]: Parameter %s has been set to value: %f", get_name(), f.get()->param_name.c_str(), (float)f.get()->value);
    return true;
  }
}
//}

/* new_cbk_grp() method //{ */
// just a util function that returns a new mutually exclusive callback group to shorten the call
rclcpp::CallbackGroup::SharedPtr Odometry2::new_cbk_grp() {
  const rclcpp::CallbackGroup::SharedPtr new_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_groups_.push_back(new_group);
  return new_group;
}
//}

}  // namespace odometry2


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(odometry2::Odometry2)
