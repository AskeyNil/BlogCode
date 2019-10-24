// 导入 ros 头文件
#include "roscpp_tutorials/TwoInts.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    // 创建 ros 的节点名称
    char node_name[] = "cpp_client";
    // 初始化 ros 节点
    ros::init(argc, argv, node_name);
    // 创建一个节点对象
    ros::NodeHandle node;

    // 创建 service 名称
    char service_name[] = "demo_service";
    // 创建 Client
    ros::ServiceClient client =
        node.serviceClient<roscpp_tutorials::TwoInts>(service_name);
    // 创建一个Request 和 Response
    roscpp_tutorials::TwoInts::Request request;
    roscpp_tutorials::TwoInts::Response response;
    request.a = 1, request.b = 2;
    if (client.call(request, response)) { // 判断是否响应
        // 获取到响应的数据
        std::cout << response.sum << std::endl;
    } else {
        std::cout << "服务器 拒绝" << std::endl;
    }
    // 进入一个简单的事件循环
    ros::spin();
    return 0;
}