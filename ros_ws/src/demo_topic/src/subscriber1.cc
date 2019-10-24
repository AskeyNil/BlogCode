#include "demo_msgs/Team.h"
// 导入 ros 头文件
#include <ros/ros.h>

void subCallback(const demo_msgs::Team::ConstPtr &msg) {
    // 一般写业务逻辑，此处打印接收到的数据
    ROS_INFO_STREAM(msg->name);
}

int main(int argc, char **argv) {
    // 创建 ros 的节点名称
    char node_name[] = "demo_subscriber";
    // 初始化 ros 节点
    ros::init(argc, argv, node_name);
    // 创建一个节点对象
    ros::NodeHandle node;
    // 创建一个 topic 的名称
    char topic_name[] = "demo_topic";
    // 创建一个 Subscriber 对象
    const ros::Subscriber &sub = node.subscribe(topic_name, 1000, subCallback);
    // 开启 ros 的运行时循环
    ros::spin();
    return 0;
}