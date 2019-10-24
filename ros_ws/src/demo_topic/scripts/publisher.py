#!/usr/bin/env python
# coding: utf-8

# 导入 rospy 依赖
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    # 节点名称
    node_name = "demo_publisher"
    # 初始化 ros 节点
    rospy.init_node(node_name)
    # 创建一个 topic 名称
    topic_name = "demo_topic"
    # 创建一个发布者
    pub = rospy.Publisher(topic_name, String, queue_size=1000)
    # 创建一个要发布的消息
    string = String()
    # 填充要发布的消息
    string.data = "I send a message"
    # 使用 publisher 发送该消息
    while not rospy.is_shutdown():
        pub.publish(string)
    # 开启 ros 运行时循环
    rospy.spin()
