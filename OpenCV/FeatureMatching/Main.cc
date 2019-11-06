//
// Created by askeynil on 2019/11/5.
//

#include "FeatureMatching.h"

int main() {
  Mat img1 = imread("/home/askeynil/Pictures/test/Camera/box.png"), gray1;
  Mat img2 = imread("/home/askeynil/Pictures/test/Camera/box_in_scene.png"), gray2;
  cvtColor(img1, gray1, COLOR_BGR2GRAY);
  cvtColor(img2, gray2, COLOR_BGR2GRAY);
  auto matcher = FeatureMatching(gray1, gray2);

  Mat dst;
  vector<KeyPoint> keyPoints1, keyPoints2;

//    使用 match 匹配
//    vector<DMatch> matches;
//    matcher.match(keyPoints1, keyPoints2, matches);
//    drawMatches(img1, keyPoints1, img2, keyPoints2, matches, dst);

//    使用 knnMatch 匹配
//    vector<vector<DMatch>> matches;
//    vector<vector<char>> matchesMask;
//    matcher.knnMatch(keyPoints1, keyPoints2, matches, matchesMask, FeatureMatching::BF);
//    drawMatches(img1, keyPoints1,
//                img2, keyPoints2,
//                matches, dst,
//                Scalar::all(-1), Scalar::all(-1),
//                matchesMask,
//                DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  vector<Point> points;
  matcher.findCorner(points);

  cout << points.size() << endl;
  polylines(img2, {points},
            true,
            {0, 0, 255},
            3,
            LINE_AA);

  imshow("img2", img2);
  waitKey();
  destroyAllWindows();
}