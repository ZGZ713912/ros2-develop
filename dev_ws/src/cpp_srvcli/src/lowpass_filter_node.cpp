#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class LowpassFilterNode : public rclcpp::Node
{
public:
    LowpassFilterNode() : Node("lowpass_filter_node")
    {
        // 声明并获取参数
        this->declare_parameter("alpha", 0.3);  // 低通滤波系数 (0 < alpha <= 1)
        alpha_ = this->get_parameter("alpha").as_double();
        
        // 确保alpha在有效范围内
        if (alpha_ <= 0.0 || alpha_ > 1.0) {
            alpha_ = 0.3;
            RCLCPP_WARN(this->get_logger(), "滤波系数必须在(0, 1]范围内，已重置为默认值: 0.3");
        }
        
        // 创建订阅者和发布者
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_data",
            10,
            std::bind(&LowpassFilterNode::data_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("filtered_lowpass", 10);
        
        // 初始化滤波器状态
        is_initialized_ = false;
        filtered_value_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "低通滤波器已启动，滤波系数: %.2f", alpha_);
    }

private:
    void data_callback(const std_msgs::msg::Float64 & msg)
    {
        // 初始化滤波器（使用第一个数据点）
        if (!is_initialized_) {
            filtered_value_ = msg.data;
            is_initialized_ = true;
            return;
        }
        
        // 一阶低通滤波算法: y(n) = α*x(n) + (1-α)*y(n-1)
        filtered_value_ = alpha_ * msg.data + (1 - alpha_) * filtered_value_;
        
        // 发布滤波后的数据
        auto filtered_msg = std_msgs::msg::Float64();
        filtered_msg.data = filtered_value_;
        publisher_->publish(filtered_msg);
    }
    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    
    // 低通滤波参数
    double alpha_;
    double filtered_value_;
    bool is_initialized_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowpassFilterNode>());
    rclcpp::shutdown();
    return 0;
}
    