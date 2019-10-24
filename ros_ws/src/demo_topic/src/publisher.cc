#include "std_msgs/String.h"
// 导入 ros 头文件
#include <ros/ros.h>

int main(int argc, char **argv) {
    // 创建 ros 的节点名称
    char node_name[] = "demo_publisher";
    // 初始化 ros 节点
    ros::init(argc, argv, node_name);
    // 创建一个节点对象
    ros::NodeHandle node;

    // 创建一个 topic 的名称
    char topic_name[] = "demo_topic";
    // 获取一个 publisher 的对象
    const ros::Publisher &pub =
        node.advertise<std_msgs::String>(topic_name, 1000);
    // 创建一个要发送的消息对象
    std_msgs::String str;
    str.data = "I send a message";
    // 使用 publisher 发送该消息
    while (ros::ok()) {
        pub.publish(str);
    }

    // 进入一个简单的事件循环
    ros::spin();
    return 0;
}
