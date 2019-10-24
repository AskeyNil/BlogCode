#!/usr/bin/env python
# coding: utf-8

# 导入 rospy 依赖
import rospy
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse


if __name__ == "__main__":
    # 节点名称
    node_name = "py_client"
    # 初始化 ros 节点
    rospy.init_node(node_name)
    # 创建 Service 名称
    service_name = "demo_service"
    # 创建 Server
    client = rospy.ServiceProxy(service_name, AddTwoInts)
    # 创建 Request
    request = AddTwoIntsRequest()
    request.a, request.b = 1, 2
    try:
        response = client.call(request)
        print(response.sum)
    except rospy.ServiceException as error:     # 服务器拒绝响应的错误
        print("服务器拒绝响应")

    # 开启 ros 运行时循环
    rospy.spin()
