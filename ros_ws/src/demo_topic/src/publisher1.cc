#include "demo_msgs/Team.h"
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
        node.advertise<demo_msgs::Team>(topic_name, 1000);
    // 创建一个要发送的消息对象
    demo_msgs::Team team;
    // 填充要发送的数据
    team.name = "I'm a team";
    team.leader.name = "AskeyNil";
    team.leader.age = 18;
    team.location.angular.x = 1;
    team.location.linear.z = 2;
    // 使用 publisher 发送该消息
    while (ros::ok()) {
        pub.publish(team);
    }
    // 开启 ros 的运行时循环
    ros::spin();
    return 0;
}