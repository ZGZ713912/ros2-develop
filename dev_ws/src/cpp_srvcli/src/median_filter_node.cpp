#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>
#include <algorithm>

class MedianFilterNode : public rclcpp::Node
{
public:
    MedianFilterNode() : Node("median_filter_node")
    {
        // 声明并获取参数
        this->declare_parameter("window_size", 5);  // 中值滤波窗口大小
        window_size_ = this->get_parameter("window_size").as_int();
        
        // 确保窗口大小为奇数
        if (window_size_ % 2 == 0) {
            window_size_++;
            RCLCPP_WARN(this->get_logger(), "窗口大小必须为奇数，已自动调整为: %d", window_size_);
        }
        
        // 创建订阅者和发布者
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "raw_data",
            10,
            std::bind(&MedianFilterNode::data_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("filtered_median", 10);
        
        RCLCPP_INFO(this->get_logger(), "中值滤波器已启动，窗口大小: %d", window_size_);
    }

private:
    void data_callback(const std_msgs::msg::Float64 & msg)
    {
        // 将新数据添加到缓冲区
        data_buffer_.push_back(msg.data);
        
        // 保持缓冲区大小不超过窗口大小
        if (data_buffer_.size() > static_cast<size_t>(window_size_)) {
            data_buffer_.pop_front();
        }
        
        // 当缓冲区满了之后才进行滤波
        if (data_buffer_.size() == static_cast<size_t>(window_size_)) {
            // 复制缓冲区并排序以找到中值
            std::deque<double> sorted_buffer = data_buffer_;
            std::sort(sorted_buffer.begin(), sorted_buffer.end());
            
            // 计算中值
            double median = sorted_buffer[window_size_ / 2];
            
            // 发布滤波后的数据
            auto filtered_msg = std_msgs::msg::Float64();
            filtered_msg.data = median;
            publisher_->publish(filtered_msg);
        }
    }
    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    
    // 中值滤波参数和缓冲区
    int window_size_;
    std::deque<double> data_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MedianFilterNode>());
    rclcpp::shutdown();
    return 0;
}
    