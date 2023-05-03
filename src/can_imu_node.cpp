#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pwd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class CanImu : public rclcpp::Node {
public:
  CanImu() : Node("minimal_publisher") {
    this->declare_parameter("canDevice", "can0");

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    timer_ =
        this->create_wall_timer(1ms, std::bind(&CanImu::timer_callback, this));
  }

  int socketSetup() {
    const char *interface =
        this->get_parameter("canDevice").as_string().c_str();
    // CAN connection variables
    int rc;
    struct sockaddr_can addr;
    struct ifreq ifr;
    memset(&ifr, 0x0, sizeof(ifr));
    memset(&addr, 0x0, sizeof(addr));

    //  Open CAN_RAW socket
    skt_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (skt_ == -1) {
      std::perror("Socket");
      return -1; // errSocket
    }
    // Set a receive filter so we only receive select CAN IDs
    struct can_filter rfilter[3];

    rfilter[0].can_id = 0x178;
    rfilter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

    rfilter[1].can_id = 0x174;
    rfilter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

    rfilter[2].can_id = 0x17C;
    rfilter[2].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

    rc = setsockopt(skt_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter,
                    sizeof(rfilter));
    if (rc == -1) {
      std::perror("Setsockopt filter");
      return -2; // errsetup
    }

    std::strncpy(ifr.ifr_name, interface, IFNAMSIZ);

    while (::ioctl(skt_, SIOCGIFINDEX, &ifr) == -1) {
      char buffer[50];
      sprintf(buffer, "%s device, ioctl", interface);
      std::perror(buffer); // if vcan0 device is not added
      usleep(100000);
    }
    usleep(20000);

    // Bind the socket to the network interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    rc = ::bind(skt_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (rc == -1) {
      std::perror("bind");
      return -2; // errsetup
    }
    // Log that the service is up and running
    std::cout << "CANBUS interface Started: " << interface << std::endl;

    return 0;
  }

  void closeSocket() { ::close(skt_); }

private:
  void timer_callback() {
    struct can_frame frame;
    auto nbytes = ::read(skt_, &frame, CAN_MTU);

    switch (nbytes) {
    case CAN_MTU:
      switch (frame.can_id) {
      case 0x174: { // imu_1
        counter_imu_ = 1;
        imu_msg_.linear_acceleration.y =
            (-gravity_ *
             (8.4 / 65535.0 * (*(std::uint16_t *)(frame.data + 4)) - 4.2));
        imu_msg_.angular_velocity.z =
            ((320 / 65535.0 * (*(std::uint16_t *)(frame.data + 0)) - 160) *
             M_PI / 180);
      }

      break;
      case 0x178: { // imu_2
        counter_imu_++;
        imu_msg_.linear_acceleration.x =
            -gravity_ *
            (8.4 / 65535.0 * (*(std::uint16_t *)(frame.data + 4)) - 4.2);
        imu_msg_.angular_velocity.x =
            ((320 / 65535.0 * (*(std::uint16_t *)(frame.data + 0)) - 160) *
             M_PI / 180);
      } break;
      case 0x17C: { // imu_3
        imu_msg_.linear_acceleration.z =
            gravity_ *
            (8.4 / 65535.0 * (*(std::uint16_t *)(frame.data + 4)) - 4.2);
        if (counter_imu_ == 2) {
          imu_msg_.header.stamp = this->now();
          publisher_->publish(imu_msg_);
        }
      } break;
      }
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  int skt_;
  int counter_imu_ = 0;
  double gravity_ = 9.80665;
  sensor_msgs::msg::Imu imu_msg_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CanImu>();
  int rc = node->socketSetup();
  if (rc == -1) {
    node->closeSocket();
    RCLCPP_INFO(node->get_logger(), "Couldn't Open the CAN network interface");
    return 0;
  } else if (rc == -2) {
    RCLCPP_INFO(node->get_logger(), "Error in canbus I/O interface");
    return errno;
  }

  usleep(50000);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
