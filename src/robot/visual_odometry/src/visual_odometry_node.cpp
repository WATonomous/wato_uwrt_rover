// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Frame-to-frame visual odometry on the navcam: ORB features in the previous image get 3D
// positions from its depth image, are matched into the current image, and PnP + RANSAC gives
// the motion. Perseverance runs the equivalent on an FPGA, which is what lets it do VO
// continuously while driving.

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rover_core/params.hpp"
#include "rover_msgs/msg/visual_odometry.hpp"
#include "visual_odometry/motion.hpp"

using sensor_msgs::msg::Image;

namespace
{

constexpr double kMaxRotation = 0.5;  // rad between frames; more is a bad match

}  // namespace

class VisualOdometryNode : public rclcpp::Node
{
public:
  VisualOdometryNode()
  : Node("visual_odometry")
  , base_from_optical_(rover_core::loadRoverParams(rover_core::roverConfigPath()).navcamOptical())
  , max_depth_(declare_parameter("max_depth", 8.0))
  , min_depth_(declare_parameter("min_depth", 0.3))
  , ratio_(declare_parameter("match_ratio", 0.8))
  , min_inliers_(declare_parameter("min_inliers", 20))
  , max_step_(declare_parameter("max_step", 0.5))
  , orb_(cv::ORB::create(
      declare_parameter("features", 600),
      /*scaleFactor=*/1.2f,
      /*nlevels=*/6,
      /*edgeThreshold=*/15,
      /*firstLevel=*/0,
      /*WTA_K=*/2,
      cv::ORB::HARRIS_SCORE,
      /*patchSize=*/15,
      declare_parameter("fast_threshold", 8)))
  , matcher_(cv::NORM_HAMMING)
  {
    pub_ = create_publisher<rover_msgs::msg::VisualOdometry>("/visual_odometry", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/navcam/camera_info", 10, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        k_ = cv::Matx33d(m->k[0], m->k[1], m->k[2], m->k[3], m->k[4], m->k[5], m->k[6], m->k[7], m->k[8]);
      });
    image_sub_.subscribe(this, "/navcam/image");
    depth_sub_.subscribe(this, "/navcam/depth_image");
    sync_ = std::make_shared<Sync>(Policy(10), image_sub_, depth_sub_);
    sync_->registerCallback(&VisualOdometryNode::onFrame, this);
  }

private:
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  using Sync = message_filters::Synchronizer<Policy>;

  struct Frame
  {
    rclcpp::Time stamp;
    cv::Mat depth;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
  };

  void onFrame(const Image::ConstSharedPtr & image, const Image::ConstSharedPtr & depth)
  {
    if (!k_) return;
    Frame frame;
    frame.stamp = image->header.stamp;
    frame.depth = cv_bridge::toCvCopy(depth, "32FC1")->image;
    cv::Mat gray;
    cv::cvtColor(cv_bridge::toCvShare(image, "rgb8")->image, gray, cv::COLOR_RGB2GRAY);
    orb_->detectAndCompute(gray, cv::noArray(), frame.keypoints, frame.descriptors);

    if (previous_ && !previous_->descriptors.empty() && !frame.descriptors.empty()) {
      publishMotion(*previous_, frame);
    }
    previous_ = std::move(frame);
  }

  void publishMotion(const Frame & prev, const Frame & cur)
  {
    std::vector<std::vector<cv::DMatch>> knn;
    matcher_.knnMatch(prev.descriptors, cur.descriptors, knn, 2);

    const cv::Matx33d & k = *k_;
    std::vector<cv::Point3f> points;
    std::vector<cv::Point2f> pixels;
    for (const auto & m : knn) {
      if (m.size() < 2 || m[0].distance > ratio_ * m[1].distance) continue;
      const cv::Point2f & p = prev.keypoints[m[0].queryIdx].pt;
      const float d = prev.depth.at<float>(static_cast<int>(std::lround(p.y)), static_cast<int>(std::lround(p.x)));
      if (!std::isfinite(d) || d < min_depth_ || d > max_depth_) continue;
      points.emplace_back((p.x - k(0, 2)) * d / k(0, 0), (p.y - k(1, 2)) * d / k(1, 1), d);
      pixels.push_back(cur.keypoints[m[0].trainIdx].pt);
    }

    rover_msgs::msg::VisualOdometry msg;
    msg.header.stamp = cur.stamp;
    msg.header.frame_id = "base_link";
    msg.previous_stamp = prev.stamp;
    msg.matches = static_cast<uint32_t>(points.size());
    msg.delta.orientation.w = 1.0;
    const auto motion = visual_odometry::estimateMotion(points, pixels, k, base_from_optical_, min_inliers_);
    if (motion) {
      const Eigen::Vector3d t = motion->body_motion.translation();
      const Eigen::Quaterniond q(motion->body_motion.linear());
      msg.inliers = static_cast<uint32_t>(motion->inliers);
      msg.valid = t.norm() < max_step_ && Eigen::AngleAxisd(q).angle() < kMaxRotation;
      msg.delta.position.x = t.x();
      msg.delta.position.y = t.y();
      msg.delta.position.z = t.z();
      msg.delta.orientation.x = q.x();
      msg.delta.orientation.y = q.y();
      msg.delta.orientation.z = q.z();
      msg.delta.orientation.w = q.w();
    }
    pub_->publish(msg);
  }

  const Eigen::Isometry3d base_from_optical_;
  const double max_depth_;
  const double min_depth_;
  const double ratio_;
  const int min_inliers_;
  const double max_step_;
  cv::Ptr<cv::ORB> orb_;
  cv::BFMatcher matcher_;
  std::optional<cv::Matx33d> k_;
  std::optional<Frame> previous_;

  rclcpp::Publisher<rover_msgs::msg::VisualOdometry>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<Sync> sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
