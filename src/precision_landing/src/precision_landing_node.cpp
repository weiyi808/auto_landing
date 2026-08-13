#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "precision_landing/controller_core.hpp"

class PrecisionLandingNode {
 public:
  PrecisionLandingNode()
      : private_nh_("~"), controller_(precision_landing::Limits{}) {
    private_nh_.param("enable_flight", enable_flight_, false);
    center_sub_ = nh_.subscribe("/precision_landing/board_center_camera", 5,
                                &PrecisionLandingNode::centerCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 5,
                               &PrecisionLandingNode::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 5,
                              &PrecisionLandingNode::odomCallback, this);
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("/precision_landing/status", 5);
    ready_pub_ = nh_.advertise<std_msgs::Bool>(
        "/precision_landing/ready_to_land", 1);
    last_tick_ = ros::Time::now();
  }

  void tick() {
    const ros::Time now = ros::Time::now();
    precision_landing::Input input;
    input.enable_flight = enable_flight_;
    input.fcu_connected = fcu_state_.connected;
    input.fcu_offboard = fcu_state_.mode == "OFFBOARD";
    input.tag_valid = !center_stamp_.isZero();
    input.transform_valid = true;
    input.tag_age_s = (now - center_stamp_).toSec();
    input.vehicle_age_s = (now - odom_stamp_).toSec();
    input.dt_s = std::max(0.001, (now - last_tick_).toSec());
    input.tag_position_body_frd = center_frd_;
    last_tick_ = now;

    const precision_landing::Output output = controller_.step(input);
    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = now;
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    setpoint.type_mask = 0b0000111111000111;
    setpoint.velocity.x = output.velocity_body_frd.x();
    setpoint.velocity.y = output.velocity_body_frd.y();
    setpoint.velocity.z = output.velocity_body_frd.z();
    setpoint_pub_.publish(setpoint);

    std_msgs::String status;
    status.data = output.reason;
    status_pub_.publish(status);
    std_msgs::Bool ready;
    ready.data = output.ready_to_land;
    ready_pub_.publish(ready);
  }

 private:
  void centerCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    const auto& p = message->pose.position;
    center_frd_ = Eigen::Vector3d(-p.y, p.x, p.z);
    center_stamp_ = message->header.stamp;
  }
  void stateCallback(const mavros_msgs::StateConstPtr& message) {
    fcu_state_ = *message;
  }
  void odomCallback(const nav_msgs::OdometryConstPtr&) {
    odom_stamp_ = ros::Time::now();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber center_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher setpoint_pub_;
  ros::Publisher status_pub_;
  ros::Publisher ready_pub_;
  precision_landing::ControllerCore controller_;
  mavros_msgs::State fcu_state_;
  ros::Time center_stamp_;
  ros::Time odom_stamp_;
  ros::Time last_tick_;
  Eigen::Vector3d center_frd_ = Eigen::Vector3d::Zero();
  bool enable_flight_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "precision_landing");
  PrecisionLandingNode node;
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spinOnce();
    node.tick();
    rate.sleep();
  }
}
