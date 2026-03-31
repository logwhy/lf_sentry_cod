// 引入必要的头文件
#include "rclcpp/rclcpp.hpp"                             // ROS2 C++ 核心库
#include "message_filters/subscriber.h"                  // message_filters 订阅者
#include "message_filters/synchronizer.h"                // 消息同步器
#include "message_filters/sync_policies/approximate_time.h" // 近似时间同步策略
#include "std_msgs/msg/string.hpp"                       // 示例消息类型（替换为实际类型）

#include <image_transport/image_transport.hpp> //订阅的图像节点

// 简化时间单位字面量（例如 100ms）
using namespace std::chrono_literals;

// 定义节点类，继承自 rclcpp::Node
class DrawImgNode : public rclcpp::Node {
public:
    DrawImgNode() : Node("DrawImgNode_subscriber") {
    /* ----------------------------------------- */
    /* 步骤 1: 创建三个订阅者，分别订阅三个话题 */
    /* ----------------------------------------- */
    // 参数说明：
    // - this: 绑定当前节点
    // - "topic1": 话题名称
    sub1_.subscribe(this, "topic1");
    sub2_.subscribe(this, "topic2");
    sub3_.subscribe(this, "topic3");

    /* ----------------------------------------- */
    /* 步骤 2: 定义同步策略（处理三个消息类型） */
    /* ----------------------------------------- */
    // ApproximateTime 策略允许消息时间戳存在一定误差
    // 模板参数为三个订阅话题的消息类型（此处均为 std_msgs::msg::String）
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      std_msgs::msg::String,
      std_msgs::msg::String,
      std_msgs::msg::String
    >;

    /* ----------------------------------------- */
    /* 步骤 3: 初始化同步器并配置参数 */
    /* ----------------------------------------- */
    // 参数说明：
    // - SyncPolicy(10): 队列大小为10（缓存未匹配消息的数量）
    // - sub1_, sub2_, sub3_: 绑定的订阅者
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), 
      sub1_, 
      sub2_,
      sub3_
    );
    
    // 设置最大允许时间差为 200ms（消息时间戳差异超过此值则不触发回调）
    sync_->setMaxIntervalDuration(rclcpp::Duration(200ms)); 

    // 注册同步回调函数，this 表示将当前对象指针传递给回调
    sync_->registerCallback(&DrawImgNode::sync_callback, this);
  }

private:
  /* ----------------------------------------- */
  /* 步骤 4: 定义同步回调函数 */
  /* ----------------------------------------- */
  // 参数说明：
  // - msg1, msg2, msg3: 三个话题的消息指针（按订阅者顺序排列）
  void sync_callback(
    const std_msgs::msg::String::ConstSharedPtr &msg1,
    const std_msgs::msg::String::ConstSharedPtr &msg2,
    const std_msgs::msg::String::ConstSharedPtr &msg3
  ) {
    // 打印同步后的消息内容
    RCLCPP_INFO(this->get_logger(), "--- Synchronized Messages ---");
    RCLCPP_INFO(this->get_logger(), "Topic1: %s", msg1->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic2: %s", msg2->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Topic3: %s", msg3->data.c_str());
  }

  // 成员变量声明
  message_filters::Subscriber<image_transport> sub1_; // 话题1订阅者
  message_filters::Subscriber<std_msgs::msg::String> sub2_; // 话题2订阅者
  message_filters::Subscriber<std_msgs::msg::String> sub3_; // 话题3订阅者
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_; // 同步器指针
};

/* ----------------------------------------- */
/* 主函数 */
/* ----------------------------------------- */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);                              // 初始化 ROS2 上下文
  auto node = std::make_shared<DrawImgNode>();  // 创建节点对象
  rclcpp::spin(node);                                    // 进入事件循环
  rclcpp::shutdown();                                    // 关闭 ROS2 上下文
  return 0;
}