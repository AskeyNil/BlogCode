#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[]) {
    Mat src = imread("./img/blox.jpg"), gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    // 创建 orb 检测器
    auto orb = ORB::create();

    // 使用 orb 查找特征点
    vector<KeyPoint> keyPoints;
    orb->detect(gray, keyPoints);

    // 使用 orb 计算描述符
    Mat desc;
    orb->compute(gray, keyPoints, desc);

    // 查找特征点和计算描述符也可以归为一步
    // orb->detectAndCompute(src, Mat(), keyPoints, desc);

    // 绘制特征点
    drawKeypoints(src, keyPoints, src);

    imshow("src", src);
    waitKey();
    destroyAllWindows();

    return 0;
}
