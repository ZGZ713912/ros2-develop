#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <random>

class DataGeneratorNode : public rclcpp::Node
{
public:
    DataGeneratorNode() : Node("data_generator_node")
    {
        // 声明并获取参数
        this->declare_parameter("frequency", 2.0);       // 信号频率，默认2Hz
        this->declare_parameter("amplitude", 1.0);       // 信号幅度，默认1.0
        this->declare_parameter("noise_amplitude", 0.15); // 噪声幅度，默认0.15
        this->declare_parameter("publish_rate", 100.0);   // 发布频率，默认100Hz
        
        frequency_ = this->get_parameter("frequency").as_double();
        amplitude_ = this->get_parameter("amplitude").as_double();
        noise_amplitude_ = this->get_parameter("noise_amplitude").as_double();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        
        // 创建发布者
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("raw_data", 10);
        
        // 创建定时器，按指定频率发布数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / publish_rate_)),
            std::bind(&DataGeneratorNode::publish_data, this));
        
        // 初始化随机数生成器
        std::random_device rd;
        rng_ = std::mt19937(rd());
        dist_ = std::normal_distribution<double>(0.0, noise_amplitude_);
        
        // 记录开始时间
        start_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "数据发生器已启动: 频率=%.2fHz, 噪声幅度=%.2f", 
                   frequency_, noise_amplitude_);
    }

private:
    void publish_data()
    {
        // 计算当前时间（秒）
        double t = (this->get_clock()->now() - start_time_).seconds();
        
        // 生成正弦信号
        double signal = amplitude_ * std::sin(2 * M_PI * frequency_ * t);
        
        // 添加高斯噪声
        double noise = dist_(rng_);
        double noisy_signal = signal + noise;
        
        // 发布带噪声的信号
        auto msg = std_msgs::msg::Float64();
        msg.data = noisy_signal;
        publisher_->publish(msg);
    }
    
    // 发布者和定时器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 参数
    double frequency_;
    double amplitude_;
    double noise_amplitude_;
    double publish_rate_;
    
    // 随机数生成
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;
    
    // 时间跟踪
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
    