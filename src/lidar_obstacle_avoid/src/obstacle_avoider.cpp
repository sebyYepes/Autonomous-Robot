/*
 * MIT License
 * (c) 2025 Sebastian Yepes – Obstacle-avoidance node
 *
 * LiDAR  ➜  /scan             (input)
 * Node   ➜  serial /dev/ttyACM0 9600 baud (output)
 */

#include <memory>
#include <string>
#include <fcntl.h>      // open
#include <termios.h>    // POSIX serial
#include <unistd.h>     // write, close

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarAvoider : public rclcpp::Node
{
public:
  LidarAvoider()
  : Node("lidar_avoider"), detect_dist_(0.40)   // metres
  {
    /* ---------- Open serial port to Arduino ---------- */
    const char * port = "/dev/ttyACM0";         // adjust if needed
    fd_ = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Cannot open %s", port);
      throw std::runtime_error("Serial open failed");
    }
    termios tty{};
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 N 1
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    tcsetattr(fd_, TCSANOW, &tty);
    RCLCPP_INFO(get_logger(), "Serial port %s opened", port);

    /* ---------- Subscribe to /scan ---------- */
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LidarAvoider::scan_cb, this, std::placeholders::_1));
  }

  ~LidarAvoider() override
  {
    if (fd_ >= 0) close(fd_);
  }

private:
  /* ---------- Callback ---------- */
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto & ranges = msg->ranges;
    const size_t n = ranges.size();
    /* Front sector ≈ middle third */
    const size_t start = n / 3;
    const size_t end   = 2 * n / 3;

    float min_range = msg->range_max;
    for (size_t i = start; i < end; ++i)
      if (ranges[i] > msg->range_min && ranges[i] < min_range)
        min_range = ranges[i];

    const char * cmd;
    if (min_range < detect_dist_) {
      cmd = "STOP\n";
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000,
                           "Obstacle %.2f m – STOP", min_range);
    } else {
      cmd = "FORWARD\n";
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 3000,
                           "Clear (%.2f m)", min_range);
    }
    write(fd_, cmd, std::strlen(cmd));  // send to Arduino
  }

  /* ---------- Members ---------- */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  int   fd_{-1};
  const float detect_dist_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarAvoider>());
  rclcpp::shutdown();
  return 0;
}

