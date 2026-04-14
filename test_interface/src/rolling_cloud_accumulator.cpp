#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class RollingCloudAccumulator {
 public:
  RollingCloudAccumulator() : nh_(), pnh_("~") {
    pnh_.param("input_topic", input_topic_, std::string("cloud"));
    pnh_.param("output_topic", output_topic_, std::string("cloud_accum"));
    pnh_.param("max_frames", max_frames_, 20);
    pnh_.param("max_age_sec", max_age_sec_, 8.0);
    pnh_.param("voxel_leaf_size", voxel_leaf_size_, 0.15);
    pnh_.param("input_voxel_leaf_size", input_voxel_leaf_size_, 0.10);
    pnh_.param("max_points_before_filter", max_points_before_filter_, 250000);
    pnh_.param("gradient_z_min", gradient_z_min_, 0.0);
    pnh_.param("gradient_z_max", gradient_z_max_, 3.0);

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    sub_ = nh_.subscribe(input_topic_, 1, &RollingCloudAccumulator::cloudCallback, this);
  }

 private:
  struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  };

  static uint8_t lerpChannel(uint8_t a, uint8_t b, double t) {
    const double clamped_t = std::max(0.0, std::min(1.0, t));
    return static_cast<uint8_t>(std::round((1.0 - clamped_t) * a + clamped_t * b));
  }

  static Rgb lerpColor(const Rgb& a, const Rgb& b, double t) {
    return Rgb{lerpChannel(a.r, b.r, t), lerpChannel(a.g, b.g, t), lerpChannel(a.b, b.b, t)};
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeByHeight(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) const {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
    colored->reserve(input->size());

    const double z_span = std::max(1e-3, gradient_z_max_ - gradient_z_min_);
    const Rgb low{72, 96, 118};
    const Rgb mid{134, 165, 160};
    const Rgb high{214, 198, 154};

    for (const auto& pt : input->points) {
      const double normalized = std::max(0.0, std::min(1.0, (pt.z - gradient_z_min_) / z_span));
      const Rgb rgb = normalized < 0.5
                          ? lerpColor(low, mid, normalized / 0.5)
                          : lerpColor(mid, high, (normalized - 0.5) / 0.5);

      pcl::PointXYZRGB out;
      out.x = pt.x;
      out.y = pt.y;
      out.z = pt.z;
      out.r = rgb.r;
      out.g = rgb.g;
      out.b = rgb.b;
      colored->push_back(out);
    }

    colored->width = colored->size();
    colored->height = 1;
    colored->is_dense = input->is_dense;
    return colored;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxelFilter(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& input, double leaf_size) const {
    if (leaf_size <= 0.0 || input->empty()) {
      return input;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*filtered);
    return filtered;
  }

  void trimHistory(const ros::Time& stamp) {
    while (static_cast<int>(clouds_.size()) > max_frames_) {
      clouds_.pop_front();
    }

    if (max_age_sec_ <= 0.0) {
      return;
    }

    while (!clouds_.empty()) {
      const double age = (stamp - clouds_.front().first).toSec();
      if (age <= max_age_sec_) {
        break;
      }
      clouds_.pop_front();
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(msg, *input);
    input = voxelFilter(input, input_voxel_leaf_size_);

    clouds_.emplace_back(msg.header.stamp, input);
    trimHistory(msg.header.stamp);

    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto& stamped_cloud : clouds_) {
      *merged += *(stamped_cloud.second);
    }

    if (max_points_before_filter_ > 0 &&
        static_cast<int>(merged->size()) > max_points_before_filter_) {
      const double adaptive_leaf = std::max(voxel_leaf_size_, input_voxel_leaf_size_);
      merged = voxelFilter(merged, adaptive_leaf);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered = voxelFilter(merged, voxel_leaf_size_);

    sensor_msgs::PointCloud2 output;
    const auto colored = colorizeByHeight(filtered);
    pcl::toROSMsg(*colored, output);
    output.header = msg.header;
    output.header.frame_id = "world";
    pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  std::string input_topic_;
  std::string output_topic_;
  int max_frames_;
  double max_age_sec_;
  double voxel_leaf_size_;
  double input_voxel_leaf_size_;
  int max_points_before_filter_;
  double gradient_z_min_;
  double gradient_z_max_;
  std::deque<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZI>::Ptr>> clouds_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rolling_cloud_accumulator");
  RollingCloudAccumulator node;
  ros::spin();
  return 0;
}
