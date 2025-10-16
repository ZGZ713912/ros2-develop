#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller_node")
    {
        // 声明并获取PID参数和目标速度
        this->declare_parameter("target_velocity", 300.0);
        this->declare_parameter("kp", 0.5);
        this->declare_parameter("ki", 0.1);
        this->declare_parameter("kd", 0.05);
        this->declare_parameter("output_limit", 100.0);
        this->declare_parameter("integral_limit", 50.0);
        
        // 获取参数值
        target_velocity_ = this->get_parameter("target_velocity").as_double();
        kp_ = this->get_parameter("kp").as_double();
        ki_ = this->get_parameter("ki").as_double();
        kd_ = this->get_parameter("kd").as_double();
        output_limit_ = this->get_parameter("output_limit").as_double();
        integral_limit_ = this->get_parameter("integral_limit").as_double();
        
        // 初始化变量
        error_ = 0.0;
        prev_error_ = 0.0;
        integral_ = 0.0;
        derivative_ = 0.0;
        last_time_ = this->get_clock()->now();
        
        // 创建订阅者订阅滤波后的速度
        filtered_velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "filtered_velocity", 10,
            std::bind(&PIDControllerNode::velocity_callback, this, std::placeholders::_1));
        
        // 创建发布者发布控制扭矩
        control_torque_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "motor_simulator_control_input", 10);
            
        // 创建发布者发布目标速度（用于可视化）
        target_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "target_velocity", 10);
        
        RCLCPP_INFO(this->get_logger(), "PID控制器节点已启动");
        RCLCPP_INFO(this->get_logger(), "目标速度: %.2f, Kp: %.2f, Ki: %.2f, Kd: %.2f",
                   target_velocity_, kp_, ki_, kd_);
    }

private:
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 计算时间差
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        
        if (dt <= 0.0) {
            return; // 避免除以零
        }
        
        // 当前速度
        double current_velocity = msg->data;
        
        // 计算误差
        error_ = target_velocity_ - current_velocity;
        
        // 计算积分项（带积分限幅）
        integral_ += error_ * dt;
        if (integral_ > integral_limit_) {
            integral_ = integral_limit_;
        } else if (integral_ < -integral_limit_) {
            integral_ = -integral_limit_;
        }
        
        // 计算微分项
        derivative_ = (error_ - prev_error_) / dt;
        prev_error_ = error_;
        
        // 计算PID输出
        double output = kp_ * error_ + ki_ * integral_ + kd_ * derivative_;
        
        // 输出限幅
        if (output > output_limit_) {
            output = output_limit_;
        } else if (output < -output_limit_) {
            output = -output_limit_;
        }
        
        // 发布控制扭矩
        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = 0.1*output;
        control_torque_pub_->publish(torque_msg);
        
        // 发布目标速度（用于可视化）
        std_msgs::msg::Float64 target_msg;
        target_msg.data = target_velocity_;
        target_velocity_pub_->publish(target_msg);
    }

    // PID参数
    double kp_, ki_, kd_;
    double target_velocity_;
    double output_limit_;
    double integral_limit_;
    
    // PID变量
    double error_;
    double prev_error_;
    double integral_;
    double derivative_;
    rclcpp::Time last_time_;
    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr filtered_velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}
    