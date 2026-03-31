// 包含必要的头文件
#include "rclcpp/rclcpp.hpp"              // ROS2 C++客户端库
#include "std_msgs/msg/string.hpp"        // 标准字符串消息类型
#include "rm_interfaces/msg/armors.hpp"   // 装甲板消息类型
#include <math.h>                         // 数学函数库
#include <vector>                         // 向量容器
#include <chrono>                         // 时间处理库
#include <iomanip>                        // 输入输出格式控制
#define _USE_MATH_DEFINES                 // 启用数学常量定义
#include <cmath>                          // C++数学库

// 定义时间变量，用于计算时间差
std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();

// 包含TF2库，用于处理四元数和欧拉角转换
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/**
 * @brief 将四元数方向转换为偏航角(Yaw)
 * @param orientation 四元数方向
 * @return 偏航角(弧度)
 */
double orientationToYaw(const geometry_msgs::msg::Quaternion& orientation) {
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double yaw, pitch, roll;
    m.getEulerYPR(yaw, pitch, roll);  // 获取欧拉角
    return yaw;
}

/**
 * @brief 更新最小值和最大值
 * @param a 当前值
 * @param min 最小值引用
 * @param max 最大值引用
 */
void ex(double& a,double& min,double& max)
{
    if(a<min)
    {
        min = a;
    }
    if(a>max)
    {
        max = a;
    }
}

/**
 * @brief 装甲板数据分析节点类
 * 用于收集装甲板数据并计算噪声协方差参数
 */
class TopicSubscribe01 : public rclcpp::Node
{
public:
    // 构造函数，初始化节点
    TopicSubscribe01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点开始构建.", name.c_str());

        // 声明参数
        this->declare_parameter("sample_count", 10000);
        this->declare_parameter("min_sample_interval", 0.1);
        
        // 获取参数
        sample_count_ = this->get_parameter("sample_count").as_int();
        min_sample_interval_ = this->get_parameter("min_sample_interval").as_double();

        // 创建订阅者，订阅装甲板消息
        command_subscribe_ = this->create_subscription<rm_interfaces::msg::Armors>(
            "armor_detector/armors", rclcpp::SensorDataQoS(),std::bind(&TopicSubscribe01::command_callback,this,std::placeholders::_1));
     
    }

private:
    // 声明订阅者
    rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr command_subscribe_;
    
    // 配置参数
    int sample_count_;              // 需要收集的样本数量
    double min_sample_interval_;    // 最小采样间隔(秒)
    
    // 存储接收到的装甲板数据
    std::vector<rm_interfaces::msg::Armors> datas;
    
    // 在线计算统计值，节省内存
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_yaw = 0.0;
    
    double sum_sq_x = 0.0;  // 平方和用于计算方差
    double sum_sq_y = 0.0;
    double sum_sq_z = 0.0;
    double sum_sq_yaw = 0.0;
    
    int valid_samples = 0;  // 有效样本数
    
    // 观测噪声协方差参数
    double R_x = 0.0;
    double R_y = 0.0;
    double R_z = 0.0;
    double R_yaw = 0.0;

    // 过程噪声协方差参数
    double s2qx=0.0;
    double s2qy=0.0;
    double s2qz=0.0;
    double s2qyaw=0.0;

    // 过程噪声协方差的最小值和最大值限制
    double s2qx_min = 0.1;
    double s2qx_max = 100.0;
    double s2qy_min = 0.1;
    double s2qy_max = 100.0;
    double s2qz_min = 0.1;
    double s2qz_max = 100.0;
    double s2qyaw_min = 0.1;
    double s2qyaw_max = 100.0;
    
    // 上一帧的位置和偏航角数据
    double last_x_ = 0.0, last_y_ = 0.0, last_z_ = 0.0, last_yaw_ = 0.0;

    /**
     * @brief 装甲板消息回调函数
     * 处理接收到的装甲板消息，计算相关参数
     * @param msg 接收到的装甲板消息
     */
    void command_callback(const rm_interfaces::msg::Armors::SharedPtr msg)
    {
        // 输出当前已收集的数据数量
        std::cout<<"已经收集了"<<datas.size()<<"个数据"<<std::endl;
        
        // 将接收到的消息存入数据列表
        datas.push_back(*msg);
        
        // 计算距离上次处理的时间差
        auto current_time = std::chrono::steady_clock::now();
        auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time_).count()/1000.0;

        // 如果消息不为空且时间差大于最小采样间隔
        if (!msg->armors.empty() && delta_time > min_sample_interval_) {
            // 更新上次处理时间
            last_time_ = current_time;
            
            // 获取当前帧的位置和偏航角数据
            double current_x = msg->armors[0].pose.position.x;
            double current_y = msg->armors[0].pose.position.y;
            double current_z = msg->armors[0].pose.position.z;
            double current_yaw = orientationToYaw(msg->armors[0].pose.orientation);
            
            // 累积统计数据（在线算法计算均值和方差）
            sum_x += current_x;
            sum_y += current_y;
            sum_z += current_z;
            sum_yaw += current_yaw;
            
            sum_sq_x += current_x * current_x;
            sum_sq_y += current_y * current_y;
            sum_sq_z += current_z * current_z;
            sum_sq_yaw += current_yaw * current_yaw;
            
            valid_samples++;

            // 如果不是第一条数据
            if (valid_samples > 1) {
                // 如果时间差大于0
                if (delta_time > 0) {
                    // 计算各方向的速度
                    double v_x = (current_x - last_x_) / delta_time;
                    double v_y = (current_y - last_y_) / delta_time;
                    double v_z = (current_z - last_z_) / delta_time;
                    double v_yaw = (current_yaw - last_yaw_) / delta_time;

                    // 根据速度计算过程噪声协方差参数
                    // 速度越大，噪声协方差越小
                    s2qx = exp(-(abs(v_x) + 0.5*abs(v_yaw))) * (s2qx_max - s2qx_min) + s2qx_min;
                    ex(s2qx,s2qx_min,s2qx_max);  // 确保在合理范围内
                    s2qy = exp(-(abs(v_y) + 0.5*abs(v_yaw))) * (s2qy_max - s2qy_min) + s2qy_min;
                    ex(s2qy,s2qy_min,s2qy_max);
                    s2qz = exp(-(abs(v_z) + 0.5*abs(v_yaw))) * (s2qz_max - s2qz_min) + s2qz_min;
                    ex(s2qz,s2qz_min,s2qz_max);
                    s2qyaw = exp(-(abs(v_x) + 0.5*abs(v_z))) * (s2qyaw_max - s2qyaw_min) + s2qyaw_min;
                    ex(s2qyaw,s2qyaw_min,s2qyaw_max);
                }
            }

            // 更新上一帧数据
            last_x_ = current_x;
            last_y_ = current_y;
            last_z_ = current_z;
            last_yaw_ = current_yaw;
        }

        // 当收集到足够数据时进行统计分析
        if(valid_samples >= sample_count_)
        {
            // 计算均值
            double mean_x = sum_x / valid_samples;
            double mean_y = sum_y / valid_samples;
            double mean_z = sum_z / valid_samples;
            double mean_yaw = sum_yaw / valid_samples;
            
            // 使用在线算法计算方差: Var(X) = E[X^2] - E[X]^2
            R_x = sum_sq_x / valid_samples - mean_x * mean_x;
            R_y = sum_sq_y / valid_samples - mean_y * mean_y;
            R_z = sum_sq_z / valid_samples - mean_z * mean_z;
            R_yaw = sum_sq_yaw / valid_samples - mean_yaw * mean_yaw;
            
            // 确保方差非负（处理浮点误差）
            R_x = std::max(0.0, R_x);
            R_y = std::max(0.0, R_y);
            R_z = std::max(0.0, R_z);
            R_yaw = std::max(0.0, R_yaw);

            // 输出结果
            std::cout << std::fixed << std::setprecision(10);
    
        }
    }
};

/**
 * @brief 主函数
 * 初始化ROS2节点并运行
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicSubscribe01>("calculation_subscribe");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}