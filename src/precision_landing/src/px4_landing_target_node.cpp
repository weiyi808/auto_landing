#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <Eigen/Geometry>

#include <cerrno>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "precision_landing/landing_accuracy.hpp"
#include "precision_landing/px4_landing_target_core.hpp"

namespace {

bool ensureDirectory(const std::string& path) {
  if (path.empty()) {
    return false;
  }
  std::string current;
  for (const char character : path) {
    current.push_back(character);
    if (character != '/' || current.size() == 1U) {
      continue;
    }
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      return false;
    }
  }
  return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

std::string wallTimestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

bool fileIsEmpty(const std::string& path) {
  struct stat status;
  return ::stat(path.c_str(), &status) != 0 || status.st_size == 0;
}

}  // namespace

class Px4LandingTargetNode {
 public:
  Px4LandingTargetNode()
      : private_nh_("~"),
        core_(parameter("fresh_s", 0.10), parameter("short_hold_s", 0.20),
              parameter("local_pose_timeout_s", 0.20), cameraOffset()) {
    private_nh_.param("enable_target_publish", enable_target_publish_, false);
    private_nh_.param("final_accuracy_window_s", final_accuracy_window_s_, 1.0);
    private_nh_.param<std::string>("log_directory", log_directory_,
                                   "/home/wxh/nb_code/autofly_ws/log/px4_native");

    center_sub_ = nh_.subscribe("/precision_landing/board_center_camera", 5,
                                &Px4LandingTargetNode::centerCallback, this);
    local_pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 5,
                                    &Px4LandingTargetNode::localPoseCallback,
                                    this);
    state_sub_ = nh_.subscribe("/mavros/state", 5,
                               &Px4LandingTargetNode::stateCallback, this);
    extended_state_sub_ = nh_.subscribe(
        "/mavros/extended_state", 5,
        &Px4LandingTargetNode::extendedStateCallback, this);
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/precision_landing/px4_native/status", 5);
    inspection_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/precision_landing/px4_native/target_local_enu", 5);
    if (enable_target_publish_) {
      landing_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
          "/mavros/landing_target/pose", 5);
    }
    ensureDirectory(log_directory_);
    ROS_WARN_STREAM("PX4 native precision landing is target-only; mode, arming, "
                    "and LAND remain manual. target_publish="
                    << (enable_target_publish_ ? "enabled" : "disabled"));
  }

  void tick() {
    const ros::Time now = ros::Time::now();
    refreshSafetyChecks(now);

    precision_landing::NativeTargetInput input;
    input.now_s = now.toSec();
    input.vision_stamp_s = center_stamp_.toSec();
    input.local_pose_age_s = local_pose_stamp_.isZero()
                                 ? std::numeric_limits<double>::infinity()
                                 : (now - local_pose_stamp_).toSec();
    input.vision_valid = !center_stamp_.isZero();
    input.fcu_connected = fcu_state_.connected;
    input.enable_target_publish = enable_target_publish_;
    input.mavros_local_ned = mavros_local_ned_;
    input.controller_conflict = controller_conflict_;
    input.camera_center = center_camera_;
    input.vehicle_position_enu = vehicle_position_enu_;
    input.local_from_body = local_from_body_;

    const precision_landing::NativeTargetOutput output = core_.step(input);
    if (output.target_valid) {
      publishPose(inspection_target_pub_, output.target_enu, now);
    }
    if (output.should_publish && landing_target_pub_) {
      publishPose(landing_target_pub_, output.target_enu, now);
    }
    publishStatus(output);
    recordAccuracySample(output);
    writeDetailRow(now, output);
  }

 private:
  double parameter(const std::string& name, double default_value) {
    double value = default_value;
    private_nh_.param(name, value, default_value);
    return value;
  }

  Eigen::Vector3d cameraOffset() {
    return Eigen::Vector3d(parameter("camera_offset_forward_m", 0.0),
                           parameter("camera_offset_left_m", 0.0),
                           parameter("camera_offset_up_m", 0.0));
  }

  void centerCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    center_camera_ = Eigen::Vector3d(message->pose.position.x,
                                     message->pose.position.y,
                                     message->pose.position.z);
    center_stamp_ = message->header.stamp;
  }

  void localPoseCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    const geometry_msgs::Point& p = message->pose.position;
    const geometry_msgs::Quaternion& q = message->pose.orientation;
    vehicle_position_enu_ = Eigen::Vector3d(p.x, p.y, p.z);
    local_from_body_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    local_pose_stamp_ = message->header.stamp.isZero() ? ros::Time::now()
                                                       : message->header.stamp;
  }

  void stateCallback(const mavros_msgs::StateConstPtr& message) {
    fcu_state_ = *message;
  }

  void extendedStateCallback(
      const mavros_msgs::ExtendedStateConstPtr& message) {
    const uint8_t previous = landed_state_;
    landed_state_ = message->landed_state;
    if (landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR &&
        !session_active_) {
      startFlightSession();
    }
    if (session_active_ &&
        landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
        previous != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
      finishFlightSession(ros::Time::now().toSec());
    }
  }

  void refreshSafetyChecks(const ros::Time& now) {
    if (!last_safety_check_.isZero() &&
        (now - last_safety_check_).toSec() < 1.0) {
      return;
    }
    last_safety_check_ = now;
    std::string mav_frame;
    mavros_local_ned_ =
        nh_.getParam("/mavros/landing_target/mav_frame", mav_frame) &&
        mav_frame == "LOCAL_NED";

    ros::V_string nodes;
    controller_conflict_ = false;
    if (ros::master::getNodes(nodes)) {
      for (const std::string& node : nodes) {
        if (node == "/precision_landing") {
          controller_conflict_ = true;
          break;
        }
      }
    }
  }

  void publishPose(ros::Publisher& publisher, const Eigen::Vector3d& target,
                   const ros::Time& stamp) {
    geometry_msgs::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.pose.position.x = target.x();
    message.pose.position.y = target.y();
    message.pose.position.z = target.z();
    message.pose.orientation.w = 1.0;
    publisher.publish(message);
  }

  void publishStatus(
      const precision_landing::NativeTargetOutput& output) {
    std_msgs::String message;
    std::ostringstream stream;
    stream << precision_landing::nativeTargetStateName(output.state)
           << " reason=" << output.reason << " age_s=" << output.target_age_s
           << " publish=" << (output.should_publish ? 1 : 0)
           << " frame_local_ned=" << (mavros_local_ned_ ? 1 : 0)
           << " conflict=" << (controller_conflict_ ? 1 : 0);
    message.data = stream.str();
    status_pub_.publish(message);
  }

  void recordAccuracySample(
      const precision_landing::NativeTargetOutput& output) {
    if (!session_active_ || center_stamp_.isZero() ||
        center_stamp_ == last_accuracy_stamp_ || !output.observation_fresh) {
      return;
    }
    last_accuracy_stamp_ = center_stamp_;
    accuracy_samples_.push_back(
        {center_stamp_.toSec(), output.body_frd.x(), output.body_frd.y(),
         output.body_frd.z()});
  }

  void startFlightSession() {
    session_active_ = true;
    accuracy_samples_.clear();
    last_accuracy_stamp_ = ros::Time();
    ensureDirectory(log_directory_);
    detail_path_ = log_directory_ + "/flight_" + wallTimestamp() + ".csv";
    detail_stream_.open(detail_path_, std::ios::out | std::ios::trunc);
    if (!detail_stream_) {
      ROS_ERROR_STREAM("Cannot open PX4 native precision-landing log: "
                       << detail_path_);
      return;
    }
    detail_stream_
        << "stamp_s,state,reason,target_age_s,camera_x,camera_y,camera_z,"
           "forward_m,right_m,down_m,vehicle_e,vehicle_n,vehicle_u,target_e,"
           "target_n,target_u,mode,landed_state,published\n";
  }

  void writeDetailRow(
      const ros::Time& stamp,
      const precision_landing::NativeTargetOutput& output) {
    if (!session_active_ || !detail_stream_) {
      return;
    }
    detail_stream_ << std::setprecision(12) << stamp.toSec() << ','
                   << precision_landing::nativeTargetStateName(output.state)
                   << ',' << output.reason << ',' << output.target_age_s << ','
                   << center_camera_.x() << ',' << center_camera_.y() << ','
                   << center_camera_.z() << ',' << output.body_frd.x() << ','
                   << output.body_frd.y() << ',' << output.body_frd.z() << ','
                   << vehicle_position_enu_.x() << ','
                   << vehicle_position_enu_.y() << ','
                   << vehicle_position_enu_.z() << ',' << output.target_enu.x()
                   << ',' << output.target_enu.y() << ',' << output.target_enu.z()
                   << ',' << fcu_state_.mode << ','
                   << static_cast<unsigned int>(landed_state_) << ','
                   << (output.should_publish ? 1 : 0) << '\n';
    detail_stream_.flush();
  }

  void finishFlightSession(double touchdown_s) {
    const precision_landing::LandingAccuracyResult result =
        precision_landing::selectFinalAccuracy(
            accuracy_samples_, touchdown_s, final_accuracy_window_s_);
    if (detail_stream_) {
      detail_stream_.close();
    }
    const std::string summary_path = log_directory_ + "/summary.csv";
    const bool write_header = fileIsEmpty(summary_path);
    std::ofstream summary(summary_path, std::ios::out | std::ios::app);
    if (summary) {
      if (write_header) {
        summary << "wall_time,method,touchdown_stamp_s,valid,forward_m,right_m,"
                   "down_m,horizontal_error_m,sample_stamp_s,in_window_count,"
                   "detail_file\n";
      }
      summary << wallTimestamp() << ",px4_native," << std::setprecision(12)
              << touchdown_s << ',' << (result.valid ? 1 : 0) << ',';
      if (result.valid) {
        summary << result.sample.forward_m << ',' << result.sample.right_m << ','
                << result.sample.down_m << ',' << result.horizontal_error_m
                << ',' << result.sample.stamp_s;
      } else {
        summary << "nan,nan,nan,nan,nan";
      }
      summary << ',' << result.in_window_count << ',' << detail_path_ << '\n';
    } else {
      ROS_ERROR_STREAM("Cannot append PX4 native landing summary: "
                       << summary_path);
    }
    session_active_ = false;
    accuracy_samples_.clear();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber center_sub_;
  ros::Subscriber local_pose_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber extended_state_sub_;
  ros::Publisher landing_target_pub_;
  ros::Publisher inspection_target_pub_;
  ros::Publisher status_pub_;
  precision_landing::Px4LandingTargetCore core_;
  mavros_msgs::State fcu_state_;
  Eigen::Vector3d center_camera_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d vehicle_position_enu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond local_from_body_ = Eigen::Quaterniond::Identity();
  ros::Time center_stamp_;
  ros::Time local_pose_stamp_;
  ros::Time last_safety_check_;
  ros::Time last_accuracy_stamp_;
  bool enable_target_publish_ = false;
  bool mavros_local_ned_ = false;
  bool controller_conflict_ = false;
  bool session_active_ = false;
  uint8_t landed_state_ = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
  double final_accuracy_window_s_ = 1.0;
  std::string log_directory_;
  std::string detail_path_;
  std::ofstream detail_stream_;
  std::vector<precision_landing::LandingAccuracySample> accuracy_samples_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "px4_landing_target");
  Px4LandingTargetNode node;
  ros::Rate rate(10.0);
  while (ros::ok()) {
    ros::spinOnce();
    node.tick();
    rate.sleep();
  }
  return 0;
}
