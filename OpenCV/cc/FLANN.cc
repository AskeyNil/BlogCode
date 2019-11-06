#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[]) {
    Mat img1 = imread("./img/box.png"), gray1;
    Mat img2 = imread("./img/box_in_scene.png"), gray2;
    cvtColor(img1, gray1, COLOR_BGR2GRAY);
    cvtColor(img2, gray2, COLOR_BGR2GRAY);

    // 创建 orb 检测器
    auto orb = ORB::create();

    // 使用 orb 查找特征点和计算描述符
    vector<KeyPoint> keyPoints1, keyPoints2;
    Mat desc1, desc2;
    orb->detectAndCompute(img1, Mat(), keyPoints1, desc1);
    orb->detectAndCompute(img2, Mat(), keyPoints2, desc2);

    // 创建 indexPar 和 searchPar
    // 当前使用 ORB 检测器，indexPar 使用LshIndexParams 参数默认给6,12，1
    auto indexPar = makePtr<flann::LshIndexParams>(6, 12, 1);
    // 检索参数，数值越大越准确，但是也越耗时
    auto searchPar = makePtr<flann::SearchParams>(100);
    // 使用 indexpar 和 searchPar 创建 flannMatcher
    FlannBasedMatcher flannMatcher(indexPar, searchPar);
    vector<vector<DMatch>> matches;
    vector<vector<char>> matchesMask;
    // 本例子使用 knnMatch， k 设置为 2
    flannMatcher.knnMatch(desc1, desc2, matches, 2);

    for (int i = 0; i < matches.size(); i++) {
        DMatch first = matches[i][0], last = matches[i][1];
        // 第一个小于第二个的百分之80，舍去该值。这个比例根据不同的图片要进行一些微调。
        if (first.distance < 0.8 * last.distance) {
            matchesMask.push_back({1, 0});
        } else {
            matchesMask.push_back({0, 0});
        }
    }

    Mat dst;
    // 使用 Mask 掩码来输出图像
    drawMatches(img1, keyPoints1, img2, keyPoints2, matches, dst,
                Scalar::all(-1), Scalar::all(-1), matchesMask,
                DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    imshow("dst", dst);
    waitKey();
    destroyAllWindows();

    // auto flanna =

    return 0;
}
