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

    // 创建 BFMatcher，开启crossCheck获取更精确的结果
    auto matcher = BFMatcher::create(NORM_HAMMING, true);
    // BFMatcher::create 函数需要两个参数，第一个是normType，第二个为crossCheck
    // 1. normType
    // 指定距离测量的规则，默认值是NORM_L2，该值一般与NORM_L1用在SIFT和SURF检测中。
    // 如果我们使用ORB检测，我们需要将该值设置为 NORM_HAMMING2。
    // 2. crossCheck
    // 交叉检查，默认为 false，设置该值为 true 来获取更精确的结果。

    // 对比两张图的特征，进行匹配
    vector<DMatch> matches;
    matcher->match(desc1, desc2, matches);

    // 根据评分排序，从小到大，因为越小越精确
    sort(matches.begin(), matches.end());

    // 移除较差的匹配结果，只保留前百分之15的结果
    double percent = 0.15;
    int num = matches.size() * percent;
    // 至少保留十个结果
    num = max(num, 10);

    // 对 matches 进行裁剪
    matches.assign(matches.begin(), matches.begin() + num);

    // 绘制图形
    Mat dst;
    drawMatches(img1, keyPoints1, img2, keyPoints2, matches, dst);
    // 绘制图形的时候，可以将最后一个参数，flags
    // 设置为 DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS 从而只显示匹配成功的点。

    imshow("src", dst);
    waitKey();
    destroyAllWindows();
    return 0;
}
