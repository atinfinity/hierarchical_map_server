#ifndef TILE_MAP_SERVER__TILE_MAP_SERVER_NODE_HPP_
#define TILE_MAP_SERVER__TILE_MAP_SERVER_NODE_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tile_map_server/tile_cache.hpp"
#include "tile_map_server/tileset.hpp"

namespace tile_map_server
{

/// Sliding-window map server that stitches a tile window centered on the robot's
/// current position into a single OccupancyGrid and publishes it on /map.
class TileMapServer : public nav2_util::LifecycleNode
{
public:
  explicit TileMapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TileMapServer() override;

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void onTimer();
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool lookupRobotPose(double & x, double & y);

  /// Request the worker thread to rebuild the window (keeps only the latest request)
  void requestWindow(const TileIndex & center);
  void stopWorker();
  void workerLoop();

  // Parameters
  std::string tileset_path_;
  int window_size_{3};
  double hysteresis_m_{5.0};
  double update_period_s_{0.5};
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
  std::string topic_name_{"map"};
  std::vector<double> initial_window_center_{0.0, 0.0};
  int tile_cache_size_{25};

  TilesetInfo tileset_;
  std::unique_ptr<TileCache> cache_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // State shared with the worker thread
  std::thread worker_;
  std::mutex job_mutex_;
  std::condition_variable job_cv_;
  std::optional<TileIndex> pending_;
  bool running_{false};

  std::mutex center_mutex_;
  std::optional<TileIndex> current_center_;  // center tile of the published window
};

}  // namespace tile_map_server

#endif  // TILE_MAP_SERVER__TILE_MAP_SERVER_NODE_HPP_
