//
// Created by askeynil on 2019/11/5.
//

#include "FeatureMatching.h"

FeatureMatching::FeatureMatching(Mat &featureImg,
                                 Mat &checkImg,
                                 int nfeatures) :
    featureImgs({featureImg}), checkImg(checkImg), orb(ORB::create(nfeatures)) {}

FeatureMatching::FeatureMatching(vector<Mat> &featureImgs,
                                 Mat &checkImg,
                                 int nfeatures) :
    featureImgs(featureImgs), checkImg(checkImg), orb(ORB::create(nfeatures)) {}

void FeatureMatching::obtainPointsDesc(vector<vector<KeyPoint>> &featureKeyPoints,
                                       vector<KeyPoint> &checkPoints,
                                       vector<Mat> &featureDesc,
                                       Mat &checkDesc) {
  featureKeyPoints.clear();
  featureDesc.clear();
  for (const Mat &img: featureImgs) {
    vector<KeyPoint> keyPoints;
    Mat desc;
    orb->detectAndCompute(img, Mat(), keyPoints, desc);
  }
  orb->detectAndCompute(checkImg, Mat(), checkPoints, checkDesc);
}

void FeatureMatching::obtainPointsDesc(vector<KeyPoint> &featureKeyPoints,
                                       vector<KeyPoint> &checkPoints,
                                       Mat &featureDesc,
                                       Mat &checkDesc,
                                       int index) {
  if (index >= featureImgs.size())
    throw domain_error("无效索引");
  orb->detectAndCompute(featureImgs[index], Mat(), featureKeyPoints, featureDesc);
  orb->detectAndCompute(checkImg, Mat(), checkPoints, checkDesc);
}

void _match(const vector<Mat> &featureDesc,
            const Mat &checkDesc,
            vector<vector<DMatch>> &allMatches,
            int type,
            double percent,
            int leastNumber) {
  allMatches.clear();

  DescriptorMatcher *matcher = nullptr;

  if (type == FeatureMatching::BF) {
    // 创建 BFMatcher
    matcher = new BFMatcher(NORM_HAMMING, true);
  } else if (type == FeatureMatching::FLANN) {
    auto indexPar = makePtr<flann::LshIndexParams>(6, 12, 1);
    // 检索参数，数值越大越准确，但是也越耗时
    auto searchPar = makePtr<flann::SearchParams>(100);
    // 使用 indexPar 和 searchPar 创建 flannMatcher
    matcher = new FlannBasedMatcher(indexPar, searchPar);
  } else {
    throw domain_error("无效类型");
  }

  for (const auto &i : featureDesc) {
    vector<DMatch> matches;
    // 使用 match 处理数据
    matcher->match(i, checkDesc, matches);
    // 排序
    sort(matches.begin(), matches.end());
    // 对数据进行裁剪
    int number = (int) (matches.size() * percent);
    number = max(number, leastNumber);
    // 裁剪 matches
    matches.assign(matches.begin(), matches.begin() + number);
    allMatches.push_back(matches);
  }

  delete matcher;
}

void FeatureMatching::match(vector<vector<KeyPoint>> &featureKeyPoints,
                            vector<KeyPoint> &checkKeyPoints,
                            vector<vector<DMatch>> &allMatches,
                            int type,
                            double percent,
                            int leastNumber) {
  vector<Mat> featureDesc;
  Mat checkDesc;
  obtainPointsDesc(featureKeyPoints, checkKeyPoints, featureDesc, checkDesc);

  _match(featureDesc,
         checkDesc,
         allMatches,
         type,
         percent,
         leastNumber);
}

void FeatureMatching::match(vector<KeyPoint> &featureKeyPoints,
                            vector<KeyPoint> &checkKeyPoints,
                            vector<DMatch> &matches,
                            int type,
                            double percent,
                            int leastNumber,
                            int index) {
  Mat featureDesc;
  Mat checkDesc;
  obtainPointsDesc(featureKeyPoints, checkKeyPoints, featureDesc, checkDesc, index);

  vector<vector<DMatch>> allMatches;
  _match({featureDesc},
         checkDesc,
         allMatches,
         type,
         percent,
         leastNumber);
  matches = allMatches[0];
}

bool _knnMatch(const vector<Mat> &featureDesc,
               const Mat &checkDesc,
               vector<vector<vector<DMatch>>> &allMatches,
               vector<vector<vector<char>>> &allMatchesMask,
               int type,
               double percent,
               int k) {

  allMatches.clear();

  DescriptorMatcher *matcher = nullptr;

  if (type == FeatureMatching::BF) {
    // 创建 BFMatcher
    matcher = new BFMatcher(NORM_HAMMING);
  } else if (type == FeatureMatching::FLANN) {
    auto indexPar = makePtr<flann::LshIndexParams>(6, 12, 1);
    // 检索参数，数值越大越准确，但是也越耗时
    auto searchPar = makePtr<flann::SearchParams>(100);
    // 使用 indexPar 和 searchPar 创建 flannMatcher
    matcher = new FlannBasedMatcher(indexPar, searchPar);
//        matcher->knnMatch()
  } else {
    throw domain_error("无效类型");
  }

  for (const auto &i : featureDesc) {

    vector<vector<DMatch>> matches;
    vector<vector<char>> matchesMask;
    // 使用 knnMatch 处理数据
    matcher->knnMatch(i, checkDesc, matches, k);

    for (auto &matche : matches) {
      DMatch first = matche[0], last = matche[1];

      if (first.distance < percent * last.distance) {
        matchesMask.push_back({1, 0});
      } else {
        matchesMask.push_back({0, 0});
      }
    }
    allMatches.push_back(matches);
    allMatchesMask.push_back(matchesMask);
  }
  delete matcher;
}

void FeatureMatching::knnMatch(vector<vector<KeyPoint>> &featureKeyPoints,
                               vector<KeyPoint> &checkKeyPoints,
                               vector<vector<vector<DMatch>>> &allMatches,
                               vector<vector<vector<char>>> &allMatchesMask,
                               int type,
                               double percent,
                               int k) {
  vector<Mat> featureDesc;
  Mat checkDesc;
  obtainPointsDesc(featureKeyPoints, checkKeyPoints, featureDesc, checkDesc);

  _knnMatch(featureDesc,
            checkDesc,
            allMatches,
            allMatchesMask,
            type,
            percent,
            k);

}

void FeatureMatching::knnMatch(vector<KeyPoint> &featureKeyPoints,
                               vector<KeyPoint> &checkKeyPoints,
                               vector<vector<DMatch>> &matches,
                               vector<vector<char>> &matchesMask,
                               int type,
                               double percent,
                               int k,
                               int index) {
  Mat featureDesc;
  Mat checkDesc;
  obtainPointsDesc(featureKeyPoints, checkKeyPoints, featureDesc, checkDesc, index);

  vector<vector<vector<DMatch>>> allMatches;
  vector<vector<vector<char>>> allMatchesMask;

  _knnMatch({featureDesc},
            checkDesc,
            allMatches,
            allMatchesMask,
            type,
            percent,
            k);

  matches = allMatches[0];
  matchesMask = allMatchesMask[0];

}

bool _findCorner(const vector<vector<KeyPoint>> &featureKeyPoints,
                 const vector<Mat> &featureImgs,
                 const vector<KeyPoint> &checkPoints,
                 const vector<Mat> &featureDesc,
                 const Mat &checkDesc,
                 vector<vector<DMatch>> &allMatches,
                 vector<vector<char>> &allMatchesMask,
                 int type,
                 double percent,
                 int k,
                 vector<vector<Point>> &allPoints) {

  allMatches.clear();

  DescriptorMatcher *matcher = nullptr;

  if (type == FeatureMatching::BF) {
    // 创建 BFMatcher
    matcher = new BFMatcher(NORM_HAMMING);
  } else if (type == FeatureMatching::FLANN) {
    auto indexPar = makePtr<flann::LshIndexParams>(6, 12, 1);
    // 检索参数，数值越大越准确，但是也越耗时
    auto searchPar = makePtr<flann::SearchParams>(100);
    // 使用 indexPar 和 searchPar 创建 flannMatcher
    matcher = new FlannBasedMatcher(indexPar, searchPar);
  } else {
    throw domain_error("无效类型");
  }

  for (size_t i = 0; i < featureKeyPoints.size(); ++i) {

    vector<vector<DMatch >> matches;
    vector<vector<char>> matchesMask;
    vector<DMatch> goodMatches;
    // 使用 knnMatch 处理数据
    matcher->knnMatch(featureDesc[i], checkDesc, matches, k);

    for (auto &matche : matches) {
      DMatch first = matche[0], last = matche[1];
      if (first.distance < percent * last.distance) {
        goodMatches.push_back(first);
      }
    }

    if (goodMatches.size() > 10) {
      vector<Point2f> srcPoints, dstPoints;

      for (auto good : goodMatches) {
        srcPoints.push_back(featureKeyPoints[i][good.queryIdx].pt);
        dstPoints.push_back(checkPoints[good.trainIdx].pt);
      }
      vector<char> mask;
      Mat M = cv::findHomography(srcPoints, dstPoints, mask, RANSAC);
      matchesMask.push_back(mask);
      Mat img1 = featureImgs[i];
      vector<Point2f> points{
          Point2f(0, 0),
          Point2f(0, img1.size().height - 1),
          Point2f(img1.size().width - 1, img1.size().height - 1),
          Point2f(img1.size().width - 1, 0),
      };
      vector<Point> dst;
      perspectiveTransform(points, points, M);
      for (const auto &p : points) {
        dst.emplace_back(p.x, p.y);
      }
      allPoints.push_back(dst);
      allMatches.push_back(goodMatches);
      allMatchesMask.push_back(mask);

    } else {
      delete matcher;
      return false;
    }
  }

  delete matcher;
}

bool FeatureMatching::findCorner(vector<vector<Point>> &allPoints,
                                 int type,
                                 double percent,
                                 int k) {
  vector<vector<KeyPoint>> featureKeyPoints;
  vector<KeyPoint> checkKeyPoints;
  vector<vector<DMatch>> allMatches;
  vector<vector<char>> allMatchesMask;
  return findCorner(allPoints,
                    featureKeyPoints,
                    checkKeyPoints,
                    allMatches,
                    allMatchesMask,
                    type, percent, k);
}

bool FeatureMatching::findCorner(vector<vector<Point>> &allPoints,
                                 vector<vector<KeyPoint>> &featureKeyPoints,
                                 vector<KeyPoint> &checkKeyPoints,
                                 vector<vector<DMatch>> &allMatches,
                                 vector<vector<char>> &allMatchesMask,
                                 int type,
                                 double percent,
                                 int k) {
  vector<Mat> featureDesc;
  Mat checkDesc;
  obtainPointsDesc(featureKeyPoints, checkKeyPoints, featureDesc, checkDesc);
  return _findCorner(featureKeyPoints,
                     featureImgs,
                     checkKeyPoints,
                     featureDesc,
                     checkDesc,
                     allMatches,
                     allMatchesMask,
                     type,
                     percent,
                     k,
                     allPoints);

}

bool FeatureMatching::findCorner(vector<Point> &points,
                                 int type,
                                 double percent,
                                 int k,
                                 int index) {
  vector<KeyPoint> featureKeyPoints, checkKeyPoints;
  vector<DMatch> matches;
  vector<char> matchesMask;
  return findCorner(points,
                    featureKeyPoints,
                    checkKeyPoints,
                    matches,
                    matchesMask,
                    type, percent, k, index);
}

bool FeatureMatching::findCorner(vector<Point> &points,
                                 vector<KeyPoint> &featureKeyPoints,
                                 vector<KeyPoint> &checkKeyPoints,
                                 vector<DMatch> &matches,
                                 vector<char> &matchesMask,
                                 int type,
                                 double percent,
                                 int k,
                                 int index) {
  Mat featureDesc;
  Mat checkDesc;
  obtainPointsDesc(featureKeyPoints, checkKeyPoints, featureDesc, checkDesc, index);

  vector<vector<DMatch>> allMatches;
  vector<vector<char>> allMatchesMask;
  vector<vector<Point>> allPoints;
  bool result = _findCorner({featureKeyPoints},
                            {featureImgs[index]},
                            checkKeyPoints,
                            {featureDesc},
                            checkDesc,
                            allMatches,
                            allMatchesMask,
                            type,
                            percent,
                            k,
                            allPoints);
  matches = allMatches[0];
  matchesMask = allMatchesMask[0];
  points = allPoints[0];
  return result;
}









