#!/usr/bin/env python
# coding: utf-8

# 导入 rospy 依赖
import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse


def serviceCallback(request=AddTwoIntsRequest()):
    response = AddTwoIntsResponse()
    response.sum = request.a + request.b
    # 返回一个对应类型的 response 代表成功响应
    # 返回空值，代表拒绝响应
    # 在 Python 中可以更加灵活的使用返回数据，此例返回 a+b 也可以达到效果
    return response


if __name__ == "__main__":
    # 节点名称
    node_name = "py_server"
    # 初始化 ros 节点
    rospy.init_node(node_name)
    # 创建 Service 名称
    service_name = "demo_service"
    # 创建 server
    server = rospy.Service(service_name, AddTwoInts, serviceCallback)
    # 开启 ros 运行时循环
    rospy.spin()
