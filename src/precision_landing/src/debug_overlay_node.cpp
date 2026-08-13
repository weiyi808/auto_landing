#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include "precision_landing/board_geometry.hpp"
#include "precision_landing/board_pose_estimator.hpp"

class PrecisionLandingVision {
 public:
  PrecisionLandingVision() : private_nh_("~") {
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    detector_parameters_ = cv::aruco::DetectorParameters::create();
    detector_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    private_nh_.param("guide_kp", guide_kp_, 0.8);
    private_nh_.param("guide_max_speed", guide_max_speed_, 0.25);
    private_nh_.param("board_scale", board_scale_, 1.0);
    private_nh_.param("max_reprojection_error_px", max_reprojection_error_px_, 3.0);

    std::string log_path;
    private_nh_.param<std::string>(
        "log_path", log_path, "/tmp/precision_landing_error.csv");
    log_.open(log_path, std::ios::app);
    if (log_ && log_.tellp() == 0) {
      log_ << "timestamp,tag_count,reprojection_rms_px,forward_m,right_m,down_m,"
              "error_xy_m,guide_vx,guide_vy,actual_vx,actual_vy,actual_vz,state\n";
    }

    camera_info_sub_ = nh_.subscribe(
        "/camera/color/camera_info", 1,
        &PrecisionLandingVision::cameraInfoCallback, this);
    image_sub_ = nh_.subscribe(
        "/camera/color/image_raw", 1,
        &PrecisionLandingVision::imageCallback, this);
    status_sub_ = nh_.subscribe(
        "/precision_landing/status", 2,
        &PrecisionLandingVision::statusCallback, this);
    command_sub_ = nh_.subscribe(
        "/mavros/setpoint_raw/local", 2,
        &PrecisionLandingVision::commandCallback, this);
    debug_pub_ = nh_.advertise<sensor_msgs::Image>(
        "/precision_landing/debug/image_raw", 1);
    center_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/precision_landing/board_center_camera", 2);
  }

 private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& message) {
    camera_matrix_ = cv::Mat(3, 3, CV_64F,
                             const_cast<double*>(message->K.data())).clone();
    distortion_ = cv::Mat(message->D).clone();
    have_camera_info_ = true;
  }

  void statusCallback(const std_msgs::StringConstPtr& message) {
    status_ = message->data;
  }

  void commandCallback(const mavros_msgs::PositionTargetConstPtr& message) {
    command_ = *message;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& message) {
    if (!have_camera_info_ || message->data.empty() ||
        (message->encoding != "rgb8" && message->encoding != "bgr8")) {
      return;
    }

    cv::Mat raw(message->height, message->width, CV_8UC3,
                const_cast<unsigned char*>(message->data.data()), message->step);
    cv::Mat view;
    if (message->encoding == "rgb8") {
      cv::cvtColor(raw, view, cv::COLOR_RGB2BGR);
    } else {
      view = raw.clone();
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(view, dictionary_, corners, ids,
                             detector_parameters_, rejected);

    for (std::size_t index = 0; index < ids.size(); ++index) {
      if (precision_landing::blackMarkerSizeMeters(ids[index], 1.0) <= 0.0) {
        continue;
      }
      for (int corner = 0; corner < 4; ++corner) {
        cv::line(view, corners[index][corner],
                 corners[index][(corner + 1) % 4], cv::Scalar(0, 255, 0), 2);
      }
      cv::putText(view, "ID " + std::to_string(ids[index]), corners[index][0],
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }

    const precision_landing::BoardPoseEstimate estimate =
        precision_landing::estimateBoardPose(
            ids, corners, camera_matrix_, distortion_, board_scale_,
            max_reprojection_error_px_);
    if (estimate.valid) {
      ROS_INFO_THROTTLE(2.0,
                        "Joint PnP accepted: detected=%zu valid=%d rms=%.3f px",
                        ids.size(), estimate.tag_count,
                        estimate.reprojection_rms_px);
      publishCenterAndOverlay(message, estimate, view);
    } else {
      ROS_WARN_THROTTLE(2.0,
                        "Joint PnP rejected: %s (detected=%zu valid=%d rms=%.3f px)",
                        estimate.reason.c_str(), ids.size(), estimate.tag_count,
                        estimate.reprojection_rms_px);
      drawText(view, "NO BOARD POSE: " + estimate.reason, 25,
               cv::Scalar(0, 0, 255));
      drawText(view, "valid tags " + std::to_string(estimate.tag_count), 53,
               cv::Scalar(0, 165, 255));
    }
    publishDebugImage(message, view);
  }

  void publishCenterAndOverlay(
      const sensor_msgs::ImageConstPtr& image,
      const precision_landing::BoardPoseEstimate& estimate,
      cv::Mat& view) {
    geometry_msgs::PoseStamped pose;
    pose.header = image->header;
    pose.pose.position.x = estimate.tvec[0];
    pose.pose.position.y = estimate.tvec[1];
    pose.pose.position.z = estimate.tvec[2];
    pose.pose.orientation.w = 1.0;
    center_pub_.publish(pose);

    const std::vector<cv::Point3d> board_origin{{0.0, 0.0, 0.0}};
    std::vector<cv::Point2d> projected_origin;
    cv::projectPoints(board_origin, estimate.rvec, estimate.tvec,
                      camera_matrix_, distortion_, projected_origin);
    const cv::Point target(cvRound(projected_origin[0].x),
                           cvRound(projected_origin[0].y));
    cv::circle(view, target, 17, cv::Scalar(0, 0, 255), -1);
    cv::circle(view, target, 23, cv::Scalar(255, 255, 255), 2);
    cv::putText(view, "ID0 TARGET", target + cv::Point(25, -20),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);

    const double forward = -estimate.tvec[1];
    const double right = estimate.tvec[0];
    const double down = estimate.tvec[2];
    const double lateral_error = std::hypot(forward, right);
    double guide_x = guide_kp_ * forward;
    double guide_y = guide_kp_ * right;
    double guide_speed = std::hypot(guide_x, guide_y);
    if (guide_max_speed_ > 0.0 && guide_speed > guide_max_speed_) {
      guide_x *= guide_max_speed_ / guide_speed;
      guide_y *= guide_max_speed_ / guide_speed;
      guide_speed = guide_max_speed_;
    }

    const cv::Point arrow_origin(view.cols / 2, view.rows / 2);
    const double arrow_scale = guide_max_speed_ > 0.0
                                   ? 140.0 / guide_max_speed_
                                   : 0.0;
    const cv::Point arrow_tip(
        arrow_origin.x + cvRound(guide_y * arrow_scale),
        arrow_origin.y - cvRound(guide_x * arrow_scale));
    cv::arrowedLine(view, arrow_origin, arrow_tip, cv::Scalar(255, 0, 255),
                    6, cv::LINE_AA, 0, 0.25);

    char line1[320];
    char line2[320];
    std::snprintf(
        line1, sizeof(line1),
        "ID0 FRD forward %+.3f right %+.3f down %.3f m | error %.3f | tags %d | RMS %.2f px",
        forward, right, down, lateral_error, estimate.tag_count,
        estimate.reprojection_rms_px);
    std::snprintf(
        line2, sizeof(line2),
        "GUIDE vx %+.2f vy %+.2f speed %.2f | ACTUAL vx %+.2f vy %+.2f vz %+.2f | %s",
        guide_x, guide_y, guide_speed, command_.velocity.x,
        command_.velocity.y, command_.velocity.z, status_.c_str());
    drawText(view, line1, 25, cv::Scalar(0, 0, 255));
    drawText(view, line2, 53, cv::Scalar(255, 0, 255));

    if (log_) {
      log_ << std::fixed << std::setprecision(6) << image->header.stamp.toSec()
           << ',' << estimate.tag_count << ',' << estimate.reprojection_rms_px
           << ',' << forward << ',' << right << ',' << down << ','
           << lateral_error << ',' << guide_x << ',' << guide_y << ','
           << command_.velocity.x << ',' << command_.velocity.y << ','
           << command_.velocity.z << ',' << status_ << '\n';
      log_.flush();
    }
  }

  void publishDebugImage(const sensor_msgs::ImageConstPtr& source,
                         const cv::Mat& view) {
    sensor_msgs::Image output;
    output.header = source->header;
    output.height = view.rows;
    output.width = view.cols;
    output.encoding = "bgr8";
    output.step = view.cols * 3;
    output.data.assign(view.data, view.data + view.total() * view.elemSize());
    debug_pub_.publish(output);
  }

  static void drawText(cv::Mat& image, const std::string& text, int y,
                       const cv::Scalar& color) {
    cv::putText(image, text, cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(255, 255, 255), 4);
    cv::putText(image, text, cv::Point(15, y), cv::FONT_HERSHEY_SIMPLEX,
                0.55, color, 2);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber command_sub_;
  ros::Publisher debug_pub_;
  ros::Publisher center_pub_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  bool have_camera_info_ = false;
  double guide_kp_ = 0.8;
  double guide_max_speed_ = 0.25;
  double board_scale_ = 1.0;
  double max_reprojection_error_px_ = 3.0;
  std::string status_ = "WAIT";
  mavros_msgs::PositionTarget command_;
  std::ofstream log_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "precision_landing_vision");
  PrecisionLandingVision node;
  ros::spin();
}
