#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <sstream>

using namespace std::chrono_literals;

class SerialMotorBridge : public rclcpp::Node {
public:
  SerialMotorBridge() : Node("serial_motor_bridge") {
    declare_parameter<std::string>("port", "/dev/arduino_uno");
    declare_parameter<double>("wheel_separation", 0.16);
    declare_parameter<double>("max_linear_speed", 0.5);

    port_ = get_parameter("port").as_string();
    wheel_separation_ = get_parameter("wheel_separation").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();

    fd_ = open_serial(port_.c_str());
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "シリアルポートを開けません: %s", port_.c_str());
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&SerialMotorBridge::cmdVelCallback, this, std::placeholders::_1));
    encoder_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("encoder_ticks", 10);
    read_timer_ = create_wall_timer(10ms, std::bind(&SerialMotorBridge::readSerial, this));
  }

  ~SerialMotorBridge() override { if (fd_ >= 0) close(fd_); }

private:
  int open_serial(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return fd;
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_lflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }
    return fd;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double v = msg->linear.x, w = msg->angular.z;
    double v_left  = v - (w * wheel_separation_ / 2.0);
    double v_right = v + (w * wheel_separation_ / 2.0);

    int pwm_left  = std::clamp(static_cast<int>((v_left  / max_linear_speed_) * 255.0), -255, 255);
    int pwm_right = std::clamp(static_cast<int>((v_right / max_linear_speed_) * 255.0), -255, 255);
    sendCommand(pwm_left, pwm_right);
  }

  void sendCommand(int left, int right) {
    if (fd_ < 0) return;
    std::ostringstream oss;
    oss << "M," << left << "," << right << "\n";
    std::string s = oss.str();
    if (write(fd_, s.c_str(), s.size()) < 0) {
      RCLCPP_WARN(get_logger(), "シリアル書き込み失敗: %s", strerror(errno));
    }
  }

  void readSerial() {
    if (fd_ < 0) return;
    char buf[256];
    ssize_t n = read(fd_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      line_buffer_ += buf;
      size_t pos;
      while ((pos = line_buffer_.find('\n')) != std::string::npos) {
        parseLine(line_buffer_.substr(0, pos));
        line_buffer_.erase(0, pos + 1);
      }
    }
  }

  void parseLine(const std::string &line) {
    if (line.empty() || line[0] != 'E') return;
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> fields;
    while (std::getline(ss, token, ',')) fields.push_back(token);
    if (fields.size() < 3) return;
    try {
      std_msgs::msg::Int32MultiArray msg;
      msg.data = {std::stoi(fields[1]), std::stoi(fields[2])};
      encoder_pub_->publish(msg);
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "パース失敗: %s", line.c_str());
    }
  }

  int fd_{-1};
  std::string port_, line_buffer_;
  double wheel_separation_, max_linear_speed_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialMotorBridge>());
  rclcpp::shutdown();
  return 0;
}
