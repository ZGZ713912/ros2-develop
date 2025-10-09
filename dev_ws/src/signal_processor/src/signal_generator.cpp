#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class SignalGenerator : public rclcpp::Node
{
public:
    SignalGenerator() : Node("signal_generator")
    {
        // 创建发布者，发布正弦信号和方波信号
        sine_publisher_ = this->create_publisher<std_msgs::msg::Float64>("sine_signal", 10);
        square_publisher_ = this->create_publisher<std_msgs::msg::Float64>("square_signal", 10);
        
        // 创建定时器，以500Hz的频率发布信号
        double publish_frequency = 500.0; // 发布频率500Hz
        timer_ = this->create_wall_timer(
            std::chrono::microseconds((int)(1e6 / publish_frequency)),
            std::bind(&SignalGenerator::generate_signals, this));
            
        RCLCPP_INFO(this->get_logger(), "Signal generator started. Publishing sine (10Hz) and square (1Hz) at 500Hz");
    }

private:
    void generate_signals()
    {
        // 获取当前时间(秒)
        auto now = this->get_clock()->now();
        double time = now.seconds();
        
        // 生成10Hz正弦信号 (sin(2π*10*t))
        double sine = std::sin(2 * M_PI * 10 * time);
        
        // 生成1Hz方波信号 (±1)
        double square = (std::fmod(time, 1.0) < 0.5) ? 1.0 : -1.0;
        
        // 发布信号
        auto sine_msg = std_msgs::msg::Float64();
        sine_msg.data = sine;
        sine_publisher_->publish(sine_msg);
        
        auto square_msg = std_msgs::msg::Float64();
        square_msg.data = square;
        square_publisher_->publish(square_msg);
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sine_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr square_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalGenerator>());
    rclcpp::shutdown();
    return 0;
}
