// 导入 ros 头文件
#include "roscpp_tutorials/TwoInts.h"
#include <ros/ros.h>

bool serviceCallback(roscpp_tutorials::TwoInts::Request &request,
                     roscpp_tutorials::TwoInts::Response &response) {
    //  返回值 bool：true代表成功响应，false代表拒绝响应
    //  可根据业务实际情况返回相应数据，本例就不做false处理了
    response.sum = request.a + request.b;
    return true;
}

int main(int argc, char **argv) {
    // 创建 ros 的节点名称
    char node_name[] = "cpp_server";
    // 初始化 ros 节点
    ros::init(argc, argv, node_name);
    // 创建一个节点对象
    ros::NodeHandle node;
    // 创建 service 名称
    char service_name[] = "demo_service";
    // 创建一个 server
    const ros::ServiceServer &server =
        node.advertiseService(service_name, serviceCallback);
    // 进入一个简单的事件循环
    ros::spin();
    return 0;
}