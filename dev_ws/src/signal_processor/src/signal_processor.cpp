#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class SignalProcessor : public rclcpp::Node
{
public:
    SignalProcessor() : Node("signal_processor")
    {
        // 创建订阅者，订阅正弦信号和方波信号
        sine_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "sine_signal", 10,
            std::bind(&SignalProcessor::sine_callback, this, std::placeholders::_1));
            
        square_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "square_signal", 10,
            std::bind(&SignalProcessor::square_callback, this, std::placeholders::_1));
            
        // 创建发布者，发布处理后的信号
        processed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("processed_signal", 10);
        
        // 初始化信号值
        last_sine_ = 0.0;
        last_square_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Signal processor started. Processing signals...");
    }

private:
    void sine_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_sine_ = msg->data;
        process_signals();
    }
    
    void square_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        last_square_ = msg->data;
        process_signals();
    }
    
    void process_signals()
    {
        // 处理逻辑：当正弦信号与方波信号同号时，输出原正弦信号，否则输出为零
        double processed;
        if ((last_sine_ >= 0 && last_square_ >= 0) || (last_sine_ <= 0 && last_square_ <= 0)) {
            processed = last_sine_;
        } else {
            processed = 0.0;
        }
        
        // 发布处理后的信号
        auto processed_msg = std_msgs::msg::Float64();
        processed_msg.data = processed;
        processed_publisher_->publish(processed_msg);
        
        // 可以取消注释下面一行来查看处理结果
        // RCLCPP_INFO(this->get_logger(), "Processed: %.2f (sine: %.2f, square: %.2f)", 
        //             processed, last_sine_, last_square_);
    }
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sine_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr square_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processed_publisher_;
    
    double last_sine_;    // 存储最近接收到的正弦信号
    double last_square_;  // 存储最近接收到的方波信号
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalProcessor>());
    rclcpp::shutdown();
    return 0;
}
