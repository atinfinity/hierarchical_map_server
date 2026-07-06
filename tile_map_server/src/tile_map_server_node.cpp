#include "tile_map_server/tile_map_server_node.hpp"

#include <chrono>
#include <utility>

#include <tf2/exceptions.h>

#include "tile_map_server/stitcher.hpp"

namespace tile_map_server
{

TileMapServer::TileMapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("tile_map_server", "", options)
{
  declare_parameter("tileset_path", std::string(""));
  declare_parameter("window_size", 3);
  declare_parameter("hysteresis_m", 5.0);
  declare_parameter("update_period_s", 0.5);
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_frame", std::string("base_link"));
  declare_parameter("topic_name", std::string("map"));
  declare_parameter("initial_window_center", std::vector<double>{0.0, 0.0});
  declare_parameter("tile_cache_size", 25);
}

TileMapServer::~TileMapServer()
{
  stopWorker();
}

nav2_util::CallbackReturn TileMapServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tileset_path_ = get_parameter("tileset_path").as_string();
  window_size_ = static_cast<int>(get_parameter("window_size").as_int());
  hysteresis_m_ = get_parameter("hysteresis_m").as_double();
  update_period_s_ = get_parameter("update_period_s").as_double();
  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_frame").as_string();
  topic_name_ = get_parameter("topic_name").as_string();
  initial_window_center_ = get_parameter("initial_window_center").as_double_array();
  tile_cache_size_ = static_cast<int>(get_parameter("tile_cache_size").as_int());

  if (window_size_ < 1 || window_size_ % 2 == 0) {
    RCLCPP_ERROR(get_logger(), "window_size must be a positive odd number (got %d)", window_size_);
    return nav2_util::CallbackReturn::FAILURE;
  }
  if (initial_window_center_.size() != 2) {
    RCLCPP_ERROR(get_logger(), "initial_window_center must be [x, y]");
    return nav2_util::CallbackReturn::FAILURE;
  }

  try {
    tileset_ = loadTilesetInfo(tileset_path_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load tileset: %s", e.what());
    return nav2_util::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(
    get_logger(), "Tileset: %.3f m/cell, tile %d cells (%.1f m), window %dx%d (%.1f m)",
    tileset_.resolution, tileset_.tile_size_cells, tileset_.tile_size_m(),
    window_size_, window_size_, window_size_ * tileset_.tile_size_m());

  // キャッシュ容量は最低でも窓全体+一列分を確保する
  const int min_cache = window_size_ * (window_size_ + 1);
  cache_ = std::make_unique<TileCache>(
    tileset_, static_cast<std::size_t>(std::max(tile_cache_size_, min_cache)),
    [this](const std::string & msg) {
      RCLCPP_WARN(get_logger(), "Tile load error: %s", msg.c_str());
    });

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1),
    std::bind(&TileMapServer::onInitialPose, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TileMapServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  map_pub_->on_activate();

  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    running_ = true;
    pending_.reset();
  }
  worker_ = std::thread(&TileMapServer::workerLoop, this);

  // tf確立前でも初期窓を配信する(AMCLが地図を得るまでtfは出ないため)
  requestWindow(tileIndexOf(tileset_, initial_window_center_[0], initial_window_center_[1]));

  timer_ = create_wall_timer(
    std::chrono::duration<double>(update_period_s_),
    std::bind(&TileMapServer::onTimer, this));

  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TileMapServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  stopWorker();
  map_pub_->on_deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TileMapServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  map_pub_.reset();
  initial_pose_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  cache_.reset();
  {
    std::lock_guard<std::mutex> lock(center_mutex_);
    current_center_.reset();
  }
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TileMapServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool TileMapServer::lookupRobotPose(double & x, double & y)
{
  try {
    const auto tf =
      tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Cannot look up robot pose (%s -> %s): %s",
      global_frame_.c_str(), robot_frame_.c_str(), e.what());
    return false;
  }
}

void TileMapServer::onTimer()
{
  double x = 0.0, y = 0.0;
  if (!lookupRobotPose(x, y)) {
    return;
  }

  std::optional<TileIndex> center;
  {
    std::lock_guard<std::mutex> lock(center_mutex_);
    center = current_center_;
  }
  if (!center || shouldRecenter(tileset_, *center, x, y, hysteresis_m_)) {
    requestWindow(tileIndexOf(tileset_, x, y));
  }
}

void TileMapServer::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto idx =
    tileIndexOf(tileset_, msg->pose.pose.position.x, msg->pose.pose.position.y);

  std::optional<TileIndex> center;
  {
    std::lock_guard<std::mutex> lock(center_mutex_);
    center = current_center_;
  }
  if (!center || *center != idx) {
    RCLCPP_INFO(
      get_logger(), "Initial pose received (%.1f, %.1f) -> recentering to tile (%d, %d)",
      msg->pose.pose.position.x, msg->pose.pose.position.y, idx.x, idx.y);
    requestWindow(idx);
  }
}

void TileMapServer::requestWindow(const TileIndex & center)
{
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (!running_) {
      return;
    }
    pending_ = center;  // latest-wins: 未処理の古い要求は上書き
  }
  job_cv_.notify_one();
}

void TileMapServer::stopWorker()
{
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    running_ = false;
    pending_.reset();
  }
  job_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void TileMapServer::workerLoop()
{
  while (true) {
    TileIndex target;
    {
      std::unique_lock<std::mutex> lock(job_mutex_);
      job_cv_.wait(lock, [this] {return !running_ || pending_.has_value();});
      if (!running_) {
        return;
      }
      target = *pending_;
      pending_.reset();
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(
      stitchWindow(tileset_, *cache_, target, window_size_));
    grid->header.frame_id = global_frame_;
    grid->header.stamp = now();
    grid->info.map_load_time = grid->header.stamp;
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();

    map_pub_->publish(std::move(grid));
    {
      std::lock_guard<std::mutex> lock(center_mutex_);
      current_center_ = target;
    }
    RCLCPP_INFO(
      get_logger(),
      "Published window centered on tile (%d, %d), stitched in %ld ms (cache: %zu tiles)",
      target.x, target.y, elapsed_ms, cache_->size());
  }
}

}  // namespace tile_map_server
