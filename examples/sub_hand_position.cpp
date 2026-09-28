#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <cstdint>
int can_init();

static const std::vector<uint32_t> FRAME_IDS = {
    0x05738001, 0x05740001, 0x05748001,
    0x05750001, 0x05758001, 0x05760001};
static const std::vector<uint32_t> FRAME_IDS2 = {
    0x05738002, 0x05740002, 0x05748002,
    0x05750002, 0x05758002, 0x05760002};
int can_init()
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0)
    {
        std::perror("socket");
        return -1;
    }

    struct ifreq ifr{};
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
    {
        std::perror("ioctl");
        close(sock);
        return -1;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        std::perror("bind");
        close(sock);
        return -1;
    }
    return sock;
}

bool send_left_hand_positions(int sock, const std::vector<uint16_t> &pos)
{
    if (pos.size() != 6)
    {
        std::fprintf(stderr, "pos.size() != 6\n");
        return false;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        struct can_frame frame{};
        frame.can_id = (FRAME_IDS[i] & CAN_EFF_MASK) | CAN_EFF_FLAG;
        frame.can_dlc = 2;
        frame.data[0] = pos[i] & 0xFF;
        frame.data[1] = (pos[i] >> 8) & 0xFF;

        if (write(sock, &frame, sizeof(frame)) != sizeof(frame))
        {
            std::perror("write");
            return false;
        }
        std::printf("Sent 0x%08X  [%02X %02X]\n",
                    frame.can_id, frame.data[0], frame.data[1]);
    }
    return true;
}

bool send_right_hand_positions(int sock, const std::vector<uint16_t> &pos)
{
    if (pos.size() != 6)
    {
        std::fprintf(stderr, "pos.size() != 6\n");
        return false;
    }

    for (size_t i = 0; i < 6; ++i)
    {
        struct can_frame frame{};
        frame.can_id = (FRAME_IDS2[i] & CAN_EFF_MASK) | CAN_EFF_FLAG;
        frame.can_dlc = 2;
        frame.data[0] = pos[i] & 0xFF;
        frame.data[1] = (pos[i] >> 8) & 0xFF;

        if (write(sock, &frame, sizeof(frame)) != sizeof(frame))
        {
            std::perror("write");
            return false;
        }
        std::printf("Sent 0x%08X  [%02X %02X]\n",
                    frame.can_id, frame.data[0], frame.data[1]);
    }
    return true;
}
class HandPositionSub : public rclcpp::Node
{
public:
    HandPositionSub()
        : Node("hand_position_sub")
    {
        sub_ = create_subscription<std_msgs::msg::Float32>(
            "rightHandPosition", // 话题名
            10,                  // QoS
            std::bind(&HandPositionSub::callback, this, std::placeholders::_1));
        // sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "rightHandPosition", // 话题名
        //     10,                  // QoS
        //     std::bind(&HandPositionSub::callback, this, std::placeholders::_1));
        sub_1 = create_subscription<std_msgs::msg::Float32>(
            "leftHandPosition", // 话题名
            10,                 // QoS
            std::bind(&HandPositionSub::callback1, this, std::placeholders::_1));
        sock = can_init();
        RCLCPP_INFO(this->get_logger(), "Waiting for /handPosition ...");
    }

private:
    void callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received handPosition = %.3f", msg->data);
        std::vector<uint16_t> TARGET_POS;
        TARGET_POS.reserve(6);
        for (int i = 0; i < 4; ++i)
        {
            TARGET_POS.push_back(static_cast<uint16_t>(msg->data));
        }

        TARGET_POS.push_back(static_cast<uint16_t>(msg->data) / 2.0 + 600);
        TARGET_POS.push_back(0.0);
        send_right_hand_positions(sock, TARGET_POS);
    }
    // void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    // {
    //     std::vector<uint16_t> TARGET_POS;
    //     TARGET_POS.reserve(6);
    //     if (msg->data.size() < 6)
    //     {
    //         RCLCPP_WARN(this->get_logger(), "got %zu floats, need 6", msg->data.size());
    //         return;
    //     }
    //     RCLCPP_INFO(this->get_logger(),
    //                 "rightHandPosition: [%f, %f, %f, %f, %f, %f]",
    //                 msg->data[0], msg->data[1], msg->data[2],
    //                 msg->data[3], msg->data[4], msg->data[5]);
    //     TARGET_POS.reserve(6);
    //     for (int i = 0; i < 6; ++i)
    //     {
    //         TARGET_POS.push_back(static_cast<uint16_t>(msg->data[i]));
    //     }
    //     send_right_hand_positions(sock, TARGET_POS);
    // }
    // void callback1(const std_msgs::msg::Float32::SharedPtr msg)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received handPosition = %.3f", msg->data);
    //     std::vector<uint16_t> TARGET_POS;
    //     TARGET_POS.reserve(6);
    //     for (int i = 0; i < 4; ++i)
    //     {
    //         TARGET_POS.push_back(static_cast<uint16_t>(msg->data));
    //     }
    //     TARGET_POS.push_back(static_cast<uint16_t>(msg->data) / 2.0 + 600);
    //     TARGET_POS.push_back(0.0);
    //     send_left_hand_positions(sock, TARGET_POS);
    // }

      void callback1(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received handPosition = %.3f", msg->data);
        std::vector<uint16_t> TARGET_POS;
        TARGET_POS.reserve(6);
        //pinch
        TARGET_POS.push_back(1000.0);
        TARGET_POS.push_back(1000.0);
        
        TARGET_POS.push_back(static_cast<uint16_t>(msg->data) *0.556 + 444.0);
        TARGET_POS.push_back(static_cast<uint16_t>(msg->data) *0.556 + 444.0);
        TARGET_POS.push_back(static_cast<uint16_t>(msg->data) *0.385 + 615.0);
        TARGET_POS.push_back(145.0);
        //  TARGET_POS.push_back(1000.0);
        // TARGET_POS.push_back(1000.0);
        
        // TARGET_POS.push_back(1000.0);
        // TARGET_POS.push_back(static_cast<uint16_t>(msg->data) *0.385 + 615.0);
        // TARGET_POS.push_back(static_cast<uint16_t>(msg->data) *0.333 + 667.0);
        // TARGET_POS.push_back(342.0);
        send_left_hand_positions(sock, TARGET_POS);
    }
    int sock;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandPositionSub>());
    rclcpp::shutdown();
    return 0;
}