#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

// 哈里斯
void harris(const Mat &src, Mat &dst) {
    Mat gray, points;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    cornerHarris(gray, points, 2, 3, 0.04);
    double points_max;
    src.copyTo(dst);
    minMaxLoc(points, nullptr, &points_max);
    for (int i = 0; i < dst.size().height; i++) {
        for (int j = 0; j < dst.size().width; j++) {
            if (points.at<float>(i, j) > 0.01 * points_max) {
                dst.at<Vec3b>(i, j) = {0, 0, 255};
            }
        }
    }
}

// 史托马斯
void shiTomasi(const Mat &src, Mat &dst) {
    Mat gray;
    vector<Point> points;
    src.copyTo(dst);
    cvtColor(src, gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(gray, points, 25, 0.01, 10);
    for (int i = 0; i < points.size(); i++) {
        circle(dst, points[i], 3, {0, 0, 255}, -1);
    }
}

int main(int argc, char const *argv[]) {
    // 哈里斯角点检测
    String filename = "./img/blox.jpg";
    Mat src = imread(filename), dst;
    imshow("src", src);

    // harris(src, dst);
    shiTomasi(src, dst);
    imshow("dst", dst);
    waitKey();
    destroyAllWindows();

    return 0;
}
