#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class VelocityFilterNode : public rclcpp::Node
{
public:
    VelocityFilterNode() : Node("velocity_filter_node")
    {
        // 声明并获取参数
        this->declare_parameter("window_size", 5);
        window_size_ = this->get_parameter("window_size").as_int();
        
        // 确保窗口大小为奇数
        if (window_size_ % 2 == 0) {
            window_size_++;
            RCLCPP_WARN(this->get_logger(), "窗口大小调整为奇数: %d", window_size_);
        }

        // 创建订阅者订阅原始速度数据
        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "simulated_motor_velocity", 10,
            std::bind(&VelocityFilterNode::velocity_callback, this, std::placeholders::_1));
        
        // 创建发布者发布滤波后的速度
        filtered_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "filtered_velocity", 10);
        
        RCLCPP_INFO(this->get_logger(), "速度滤波器节点已启动，窗口大小: %d", window_size_);
    }

private:
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 将新数据添加到缓冲区
        velocity_buffer_.push_back(msg->data);
        
        // 保持缓冲区大小不超过窗口大小
        if (velocity_buffer_.size() > static_cast<size_t>(window_size_)) {
            velocity_buffer_.pop_front();
        }
        
        // 当缓冲区满了之后才进行滤波
        if (velocity_buffer_.size() == static_cast<size_t>(window_size_)) {
            // 使用中值滤波 - 对随机噪声有较好的抑制效果
            std::deque<double> sorted_buffer = velocity_buffer_;
            std::sort(sorted_buffer.begin(), sorted_buffer.end());
            
            // 计算中值
            double median = sorted_buffer[window_size_ / 2];
            
            // 发布滤波后的速度
            std_msgs::msg::Float64 filtered_msg;
            filtered_msg.data = median;
            filtered_velocity_pub_->publish(filtered_msg);
            RCLCPP_INFO(this->get_logger(), "速度滤波器节点已启动，窗口大小: %d", window_size_);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filtered_velocity_pub_;
    std::deque<double> velocity_buffer_;
    int window_size_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityFilterNode>());
    rclcpp::shutdown();
    return 0;
}