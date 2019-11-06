//
// Created by askeynil on 2019/11/5.
//

#ifndef MAIN_FEATUREMATCHING_H
#define MAIN_FEATUREMATCHING_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class FeatureMatching {

private:

  const Ptr<ORB> orb;

public:
  enum {
    BF,
    FLANN
  };

  vector<Mat> featureImgs;
  Mat checkImg;

  FeatureMatching(Mat &featureImg, Mat &checkImg, int nfeatures = 500);

  FeatureMatching(vector<Mat> &featureImgs, Mat &checkImg, int nfeatures = 500);

  /**
   * 获取特征点和描述符（多张图）
   * > 使用该方法返回所有的特征匹配
   * @param featureKeyPoints  特征图片的特征点
   * @param checkPoints       检查图片的特征点
   * @param featureDesc       特征图片的描述符
   * @param checkDesc         检查图片的描述符
   */
  void obtainPointsDesc(vector<vector<KeyPoint>> &featureKeyPoints,
                        vector<KeyPoint> &checkPoints,
                        vector<Mat> &featureDesc,
                        Mat &checkDesc);

  /**
   * 获取特征点和描述符（单张图）
   * > 使用该方法只会返回 featureImgs 中第一个图片与检查图片的匹配
   * @param featureKeyPoints  特征图片的特征点
   * @param checkPoints       检查图片的特征点
   * @param featureDesc       特征图片的描述符
   * @param checkDesc         检查图片的描述符
   * @param index             特征图片的索引
   */
  void obtainPointsDesc(vector<KeyPoint> &featureKeyPoints,
                        vector<KeyPoint> &checkPoints,
                        Mat &featureDesc,
                        Mat &checkDesc,
                        int index = 0);

  /**
   * 特征匹配（多张图）
   * @param featureKeyPoints  特征图片的特征点
   * @param checkKeyPoints    检查图片的特征点
   * @param allMatches        对应图片的所有的匹配点
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           保留比例，默认百分之15
   * @param leastNumber       最少保留特征数， 默认为10
   */
  void match(vector<vector<KeyPoint>> &featureKeyPoints,
             vector<KeyPoint> &checkKeyPoints,
             vector<vector<DMatch>> &allMatches,
             int type = BF,
             double percent = 0.15,
             int leastNumber = 10);

  /**
   * 特征匹配（单张图）
   * @param featureKeyPoints  特征图片的特征点
   * @param checkKeyPoints    检查图片的特征点
   * @param allMatches        对应图片的所有的匹配点
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           保留比例，默认百分之15
   * @param leastNumber       最少保留特征数， 默认为10
   * @param index             特征图片索引，默认为0
   */
  void match(vector<KeyPoint> &featureKeyPoints,
             vector<KeyPoint> &checkKeyPoints,
             vector<DMatch> &matches,
             int type = BF,
             double percent = 0.15,
             int leastNumber = 10,
             int index = 0);

  /**
   * 使用 knnMatch 特征匹配（多张）
   * @param featureKeyPoints  特征图片的特征点
   * @param checkKeyPoints    检查图片的特征点
   * @param allMatches        对应图片的所有的匹配点
   * @param allMatchesMask    对应图片的掩模
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           匹配系数，越小点越少。默认 0.8
   * @param k                 knn Match中k的系数 默认为2
   */
  void knnMatch(vector<vector<KeyPoint>> &featureKeyPoints,
                vector<KeyPoint> &checkKeyPoints,
                vector<vector<vector<DMatch>>> &allMatches,
                vector<vector<vector<char>>> &allMatchesMask,
                int type = BF,
                double percent = 0.8,
                int k = 2);

  /**
   * 使用 knnMatch 特征匹配（多张）
   * @param featureKeyPoints  特征图片的特征点
   * @param checkKeyPoints    检查图片的特征点
   * @param matches           图片的所有的匹配点
   * @param matchesMask       图片的掩模
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           匹配系数，越小点越少。默认 0.8
   * @param k                 knn Match中k的系数 默认为2
   * @param index             特征图片的索引
   */
  void knnMatch(vector<KeyPoint> &featureKeyPoints,
                vector<KeyPoint> &checkKeyPoints,
                vector<vector<DMatch>> &matches,
                vector<vector<char>> &matchesMask,
                int type = BF,
                double percent = 0.8,
                int k = 2,
                int index = 0);

  /**
   * 特征匹配（多张图）
   * @param allPoints         匹配出来的对应坐标点
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           匹配系数，越小点越少。默认 0.8
   * @param k                 knn Match中k的系数 默认为2
   * @return                  是否匹配成功
   */
  bool findCorner(vector<vector<Point>> &allPoints,
                  int type = BF,
                  double percent = 0.8,
                  int k = 2);

  /**
   * 特征匹配（多张图）
   * @param allPoints         匹配出来的对应坐标点
   * @param featureKeyPoints  特征图片的特征点
   * @param checkKeyPoints    检查图片的特征点
   * @param allMatches        对应图片的所有的匹配点
   * @param allMatchesMask    对应图片的掩模
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           匹配系数，越小点越少。默认 0.8
   * @param k                 knn Match中k的系数 默认为2
   * @return                  是否匹配成功
   */
  bool findCorner(vector<vector<Point>> &allPoints,
                  vector<vector<KeyPoint>> &featureKeyPoints,
                  vector<KeyPoint> &checkKeyPoints,
                  vector<vector<DMatch>> &allMatches,
                  vector<vector<char>> &allMatchesMask,
                  int type = BF,
                  double percent = 0.8,
                  int k = 2);

  /**
   * 特征匹配（单张图）
   * @param points            匹配出来的对应坐标点
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           匹配系数，越小点越少。默认 0.8
   * @param k                 knn Match中k的系数 默认为2
   * @param index             特征图片的索引
   * @return                  是否匹配成功
   */
  bool findCorner(vector<Point> &points,
                  int type = BF,
                  double percent = 0.8,
                  int k = 2,
                  int index = 0);

  /**
   * 特征匹配（多张图）
   * @param allPoints         匹配出来的对应坐标点
   * @param featureKeyPoints  特征图片的特征点
   * @param checkKeyPoints    检查图片的特征点
   * @param matches           对应图片的所有的匹配点
   * @param matchesMask       对应图片的掩模
   * @param type              匹配算法，当前只有 BF 和 FLANN 两种。 默认 BF
   * @param percent           匹配系数，越小点越少。默认 0.8
   * @param k                 knn Match中k的系数 默认为2
   * @param index             特征图片的索引
   * @return                  是否匹配成功
   */
  bool findCorner(vector<Point> &points,
                  vector<KeyPoint> &featureKeyPoints,
                  vector<KeyPoint> &checkKeyPoints,
                  vector<DMatch> &matches,
                  vector<char> &matchesMask,
                  int type = BF,
                  double percent = 0.8,
                  int k = 2,
                  int index = 0);

};

#endif //MAIN_FEATUREMATCHING_H
