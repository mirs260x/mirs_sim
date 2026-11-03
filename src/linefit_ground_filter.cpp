#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
#include <ground_segmentation/ground_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * @brief base_linkの蓄積点群をLinefitで分類し、地面点と非地面点を配信する。
 * 点の単位はm。判定時だけodomの回転で重力方向に合わせ、出力座標とstampは入力を保つ。
 * TF欠測・base_link以外の入力は配信せず、非有限点は除く。速度指令は出力しない。
 */
class LinefitGroundFilter : public rclcpp::Node
{
public:
  /** @brief 起動時のパラメータを読み、点群の購読と地面・障害物の配信を開始する。 */
  LinefitGroundFilter() : Node("linefit_ground_filter"), buffer_(get_clock()), listener_(buffer_)
  {
    GroundSegmentationParams p;
    p.n_bins = declare_parameter("n_bins", 80);
    p.n_segments = declare_parameter("n_segments", 180);
    p.n_threads = declare_parameter("n_threads", 4);
    const double r_min = declare_parameter("r_min", 0.2);
    const double r_max = declare_parameter("r_max", 10.0);
    const double fit_error = declare_parameter("max_fit_error", 0.05);
    p.max_dist_to_line = declare_parameter("max_dist_to_line", 0.05);
    p.sensor_height = declare_parameter("sensor_height", 0.242);
    p.min_slope = declare_parameter("min_slope", 0.0);
    p.max_slope = declare_parameter("max_slope", 0.21256);
    p.long_threshold = declare_parameter("long_threshold", 1.0);
    p.max_long_height = declare_parameter("max_long_height", 0.1);
    p.max_start_height = declare_parameter("max_start_height", 0.2);
    p.line_search_angle = declare_parameter("line_search_angle", 0.1);
    if (p.n_bins <= 0 || p.n_segments <= 0 || p.n_threads <= 0 ||
        p.n_threads > p.n_segments || !std::isfinite(r_min) || !std::isfinite(r_max) ||
        r_min < 0 || r_max <= r_min) {
      throw std::invalid_argument("Invalid Linefit bins, threads or range");
    }
    for (double value : {fit_error, p.max_dist_to_line, p.sensor_height, p.min_slope,
        p.max_slope, p.long_threshold, p.max_long_height, p.max_start_height,
        p.line_search_angle}) {
      if (!std::isfinite(value) || value < 0) {
        throw std::invalid_argument("Linefit thresholds must be finite and nonnegative");
      }
    }
    p.r_min_square = r_min * r_min;
    p.r_max_square = r_max * r_max;
    p.max_error_square = fit_error * fit_error;
    segmenter_ = std::make_unique<GroundSegmentation>(p);
    ground_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ground_segmentation/ground_points", 10);
    obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ground_segmentation/obstacle_points", 10);
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_assembled", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {classify(*msg);});
  }

private:
  void classify(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (msg.header.frame_id != "base_link") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Expected base_link input");
      return;
    }
    Eigen::Quaternionf rotation;
    try {
      const auto tf = buffer_.lookupTransform("odom", "base_link", rclcpp::Time(msg.header.stamp));
      const auto & q = tf.transform.rotation;
      rotation = Eigen::Quaternionf(q.w, q.x, q.y, q.z);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Linefit TF: %s", e.what());
      return;
    }
    PointCloud source, cloud, aligned, ground, obstacles;
    pcl::fromROSMsg(msg, source);
    for (const auto & point : source) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        cloud.push_back(point);
      }
    }
    // 平行移動は除き、base_link原点の地面からの高さを維持する。
    pcl::transformPointCloud(cloud, aligned, Eigen::Vector3f::Zero(), rotation);
    std::vector<int> labels;
    segmenter_->segment(aligned, &labels);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      (labels[i] == 1 ? ground : obstacles).push_back(cloud[i]);
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(ground, output);
    output.header = msg.header;
    ground_->publish(output);
    pcl::toROSMsg(obstacles, output);
    output.header = msg.header;
    obstacles_->publish(output);
  }
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<GroundSegmentation> segmenter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_, obstacles_;
};

/** @brief ROS 2を初期化し、Linefitの分類処理を終了要求まで実行する。@return 正常終了時0。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinefitGroundFilter>());
  rclcpp::shutdown();
  return 0;
}
