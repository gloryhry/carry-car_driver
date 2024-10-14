#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <mutex>
#include <iostream>

// #define Debug

// 定义串口对象
std::shared_ptr<serial::Serial> ser;
// 定义全局变量来保存cmd_vel消息
geometry_msgs::msg::Twist cmd_vel_msg;
// 定义ROS2的时间戳变量
rclcpp::Time last_cmd_vel_time;

std::shared_ptr<rclcpp::Node> node;

// 定义全局锁
std::mutex serial_mutex;

// 车的两轮宽度
double wheel_width;
// 车的两轮直径
double wheel_diameter;

// 初始化一个长度为35的uchar数组，并将所有值置为0
unsigned char data[35] = {0};

void init_data()
{
    data[0] = 0xA5;
    data[1] = 0x5A;
    // 控制命令
    data[2] = 0b01000001;
    data[3] = 0x1C;
    // 摄像头控制
    data[4] = 0b10110000;
    // 视频控制
    data[5] = 0x00;
    // 控制字 32bit 32-31机械臂电流等级 29-28底盘电流等级 
    data[6] = 0b10110100;
    data[7] = 0x00;
    data[8] = 0x00;
    data[9] = 0x00;
    data[10] = 0x00;
    data[11] = 0x00;
    data[12] = 0x00;
    data[13] = 0x00;
    int speed = 0;
    // 速度高字节
    data[14] = speed >> 8 << 4 & 0xF0 + speed >> 8 & 0x0F;
    // 速度左低字节
    data[15] = speed & 0xFF;
    // 速度右低字节
    data[16] = speed & 0xFF;
    
    // 后面都为0
    for(int index=17;index<32;index++)
    {
        data[index] = 0x00;
    }
    // CRC
    data[32] = data[33] = 0x00;
    data[34] = 0x55;
}

// CRC16多项式
const uint16_t POLY = 0x8005;

// 预计算的CRC表，使用标准的CRC16-CCITT多项式
uint16_t crc16_table[256];

// 初始化CRC表
void init_crc16_table() {
    for (uint16_t i = 0; i < 256; ++i) {
        uint16_t crc = i << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc = crc << 1;
            }
        }
        crc16_table[i] = crc;
    }
}

// 计算CRC16值
uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    for (size_t i = 0; i < length; ++i) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ crc16_table[index];
    }
    return crc;
}

// 小车使用以下CRC16算法
// CRC-16-XMODEM implementation
uint16_t crc16_xmodem(const uint8_t* data, size_t length) {
    uint16_t crc = 0x0000; // 初始值
    uint16_t polynomial = 0x1021; // 多项式

    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8; // 将当前字节移到高位，与CRC寄存器进行异或

        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) { // 判断最高位是否为1
                crc = (crc << 1) ^ polynomial; // 左移1位后与多项式异或
            } else {
                crc <<= 1; // 左移1位
            }
        }
    }

    return crc;
}

void calculateWheelSpeeds(double wheel_width, double wheel_diameter, double speed, double rotate, double &left_wheel_speed, double &right_wheel_speed) {
    // 计算左轮和右轮的线速度
    double V_L = speed - (wheel_width * rotate) / 2.0;
    double V_R = speed + (wheel_width * rotate) / 2.0;

    // 将线速度转换为轮速
    left_wheel_speed = V_L;
    right_wheel_speed = V_R; 
    // left_wheel_speed = V_L / (wheel_diameter / 2.0);
    // right_wheel_speed = V_R / (wheel_diameter / 2.0);
}

// 定义函数，往RS232串口中写数据
void write_to_serial()
{   
    rclcpp::Time current_time = node->now();

    double left_velocity, right_velocity;
    double speed, rotate;
    
    std::unique_lock<std::mutex> lock(serial_mutex);
    speed = cmd_vel_msg.linear.x;
    rotate = cmd_vel_msg.angular.z;
    lock.unlock();

    calculateWheelSpeeds(wheel_width, wheel_diameter, speed, rotate, left_velocity, right_velocity);

    if ((current_time - last_cmd_vel_time).seconds() > 0.5) {
        RCLCPP_WARN(node->get_logger(), "cmd_vel not updating");
        // 将left_velocity和right_velocity 改为0
        left_velocity = 0;
        right_velocity = 0;
    }

    int left_v, right_v;
    left_v = left_velocity / 0.19 * 100;
    right_v = right_velocity / 0.19 * 100;

    // 确保速度在-1000到1000的范围内
    left_v = std::max(-1000, std::min(left_v, 1000));
    right_v = std::max(-1000, std::min(right_v, 1000));

    // 速度高字节
    data[14] = left_v >> 8 << 4 & 0xF0 | right_v >> 8 & 0x0F;
    // 速度左低字节
    data[15] = left_v & 0xFF;
    // 速度右低字节
    data[16] = right_v & 0xFF;

    // 计算CRC
    uint16_t crc = crc16_xmodem(data, 32);
    data[32] = crc & 0xff;
    data[33] = crc >> 8;

#ifdef Debug
    std::string output;
    for(auto temp_data: data)
    {
        std::ostringstream oss;
        // 设置流为16进制格式
        oss << std::hex << (int)temp_data;
        output = output + " 0x" + oss.str();
    }
    RCLCPP_INFO(node->get_logger(), output);
#endif

    // 写入串口
    ser->write(data, sizeof(data));
}

// 定义回调函数，当接收到cmd_vel消息时调用
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(serial_mutex);
    last_cmd_vel_time = node->now();
    cmd_vel_msg = *msg;
    lock.unlock();
#ifdef Debug
    RCLCPP_INFO(node->get_logger(), "Received cmd_vel: linear.x = %.2f, angular.z = %.2f", msg->linear.x, msg->angular.z);
#endif
}

int main(int argc, char *argv[])
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    // 初始化CRC表
    init_crc16_table();
    // 初始化发送数据
    init_data();
    
    // 创建节点实例
    node.reset(new rclcpp::Node("rs232_driver"));

    // 从参数服务器获取串口名称、波特率和超时时间
    std::string port_name;
    int baud_rate;
    int timeout;

    node->declare_parameter("port_name", "/dev/ttyUSB0");
    node->declare_parameter("baud_rate", 115200);
    node->declare_parameter("timeout", 1000);

    node->get_parameter("port_name", port_name);
    node->get_parameter("baud_rate", baud_rate);
    node->get_parameter("timeout", timeout);
    
    node->declare_parameter("wheel_width", 0.4);
    node->declare_parameter("wheel_diameter", 0.2);

    node->get_parameter("wheel_width", wheel_width);
    node->get_parameter("wheel_diameter", wheel_diameter);

    // 初始化串口对象
    ser.reset(new serial::Serial(port_name, baud_rate, serial::Timeout::simpleTimeout(timeout)));
    if (!ser->isOpen()) {
        RCLCPP_ERROR(node->get_logger(), "Can not open Serial port: %s", port_name.c_str());
        return -1;
    }

    last_cmd_vel_time = node->now();

    // 创建订阅者，订阅cmd_vel话题
    auto subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, cmd_vel_callback);

    // 按照40Hz的频率循环
    rclcpp::Rate rate(40);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        write_to_serial();
        rate.sleep();
    }

    // 关闭ROS2节点
    rclcpp::shutdown();
    return 0;
}