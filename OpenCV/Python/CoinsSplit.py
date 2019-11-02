import cv2 as cv
import numpy as np


def findMarkers(distance_binary, binary, isSystem):
    if isSystem:
        # 6-8 步 使用 OpenCV 提供的函数替代
        compCount, markers = cv.connectedComponents(distance_binary)
        # 9. 在标记图中将二值图黑色的区域对应的位置设置标记为轮廓数量加 1
        # 注: connectedComponents 函数的返回值就是轮廓数加1
        markers[binary == 0] = compCount
    else:
        # 6. 查找距离二值图的轮廓
        _, contours, _ = cv.findContours(
            distance_binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # 7. 给二值图轮廓中的每个点进行颜色标记，从 1 开始标记
        compCount = len(contours)
        for index in range(compCount):
            cv.drawContours(distance_binary, contours, index, index+1, -1)

        # 8. 将标记的图转化为固定类型的标记图
        markers = np.int32(distance_binary)

        # 9. 在标记图中将二值图黑色的区域对应的位置设置标记为轮廓数量加 1
        compCount += 1
        markers[binary == 0] = compCount
    return compCount, markers


    # 1. 获取需要分割的图片
src = cv.imread("./img/coins.jpg")
cv.imshow("src", src)

# 2. 转化为灰度图
gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
cv.imshow("gray", gray)

# 3. 转化为二值图
_, binary = cv.threshold(gray, 0, 255,
                         cv.THRESH_BINARY_INV | cv.THRESH_OTSU)
cv.imshow("binary", binary)

# 4. 转化为距离图
distance = cv.distanceTransform(binary, cv.DIST_L2, 3)
# 将距离图标准化到 0,1 之间
cv.normalize(distance, distance, 0, 1.0, cv.NORM_MINMAX)
cv.imshow("distance", distance)

# 5. 分离距离图, 转化为二值图
_, distance_binary = cv.threshold(distance, 0.8, 255, cv.THRESH_BINARY)
distance_binary = distance_binary.astype(np.uint8)
cv.imshow("distance_binary", distance_binary)

compCount, markers = findMarkers(distance_binary, binary, False)

# 10. 使用分水岭算法注水
cv.watershed(src, markers)

# 11. 给注水之后的标记图上色
for index in range(1, compCount+1):
    src[markers == index] = np.random.randint(0, 256, size=(1, 3))
cv.imshow("water_later", src)

while (True):
    s = cv.waitKey(100)
    if s == ord('q'):
        break
cv.destroyAllWindows()
