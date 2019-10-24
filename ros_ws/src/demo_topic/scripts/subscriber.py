#!/usr/bin/env python
# coding: utf-8

# 导入 rospy 依赖
import rospy
from std_msgs.msg import String


def subCallback(msg=String()):
    # 一般写业务逻辑，此处打印接收到的数据
    print (msg.data)


if __name__ == "__main__":
    # 节点名称
    node_name = "demo_subscriber"
    # 初始化 ros 节点
    rospy.init_node(node_name)
    # 创建一个 topic 名称
    topic_name = "demo_topic"
    # 创建一个 Subscriber 对象
    sub = rospy.Subscriber(topic_name, String, subCallback)
    # 开启 ros 运行时循环
    rospy.spin()
