#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void findMarkers(const Mat &distance_binary, const Mat &binary, Mat &markers,
                 int &compCount, bool isSystem) {

    if (isSystem) {
        // 6-8 步 使用 OpenCV 提供的函数替代
        compCount = connectedComponents(distance_binary, markers);
        // 9. 在标记图中将二值图黑色的区域对应的位置设置标记为轮廓数量加 1
        // 注: connectedComponents 函数的返回值就是轮廓数加1
        markers.setTo(compCount, 255 - binary);
    } else {
        // 6. 查找距离二值图的轮廓
        vector<vector<Point>> contours;
        findContours(distance_binary, contours, RETR_EXTERNAL,
                     CHAIN_APPROX_SIMPLE);
        compCount = contours.size();

        // 7. 给二值图轮廓中的每个点进行颜色标记，从 1 开始标记
        for (int i = 0; i < compCount; i++) {
            drawContours(distance_binary, contours, i, Scalar(i + 1), -1);
        }

        // 8. 将标记的图转化为固定类型的标记图
        distance_binary.convertTo(markers, CV_32S);

        // 9. 在标记图中将二值图黑色的区域对应的位置设置标记为轮廓数量加 1
        markers.setTo(++compCount, 255 - binary);
    }
}

int main() {

    // 1. 获取需要分割的图片
    Mat src = imread("./img/coins.jpg");
    imshow("src", src);

    // 2. 转化为灰度图
    Mat gray;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    imshow("gray", gray);

    // 3. 转化为二值图
    Mat binary;
    threshold(gray, binary, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    imshow("binary", binary);

    // 4. 转化为距离图
    Mat distance;
    distanceTransform(binary, distance, DIST_L2, 3);
    // 将距离图标准化到 0, 1 之间
    normalize(distance, distance, 0, 1, NORM_MINMAX);
    imshow("distance", distance);

    // 5.分离距离图,转化为二值
    Mat distance_binary;
    threshold(distance, distance_binary, 0.8, 255, THRESH_BINARY);
    // 5.1 将distance_binary 转化到 CV_8U
    Mat distance_binary2;
    distance_binary.convertTo(distance_binary2, CV_8U);
    imshow("distance_binary", distance_binary2);

    // 6-9 步
    Mat markers;
    int compCount;
    findMarkers(distance_binary2, binary, markers, compCount, true);

    // 10. 使用分水岭算法注水
    watershed(src, markers);

    // 11. 给注水后的标记图上色
    vector<Vec3b> colorTab;
    RNG rng;
    for (int i = 0; i < compCount; i++) {
        int g = rng.uniform(0, 255);
        int b = rng.uniform(0, 255);
        int r = rng.uniform(0, 255);
        colorTab.emplace_back(g, b, r);
    }

    for (int i = 0; i < markers.rows; i++)
        for (int j = 0; j < markers.cols; j++) {
            int index = markers.at<int>(i, j);
            if (index <= 0 || index > compCount)
                src.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
            else
                src.at<Vec3b>(i, j) = colorTab[index - 1];
        }
    imshow("water_later", src);

    waitKey();
    destroyAllWindows();
}
