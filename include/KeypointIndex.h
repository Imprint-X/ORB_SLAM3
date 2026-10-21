// ORB-SLAM3 (GPLv3 or later): keypoint storage differs for stereo fisheye frames.
#ifndef ORB_SLAM3_KEYPOINT_INDEX_H
#define ORB_SLAM3_KEYPOINT_INDEX_H
#include <opencv2/core.hpp>
#include <vector>

namespace ORB_SLAM3 {
inline const cv::KeyPoint& KeypointAt(size_t index, int leftCount,
    const std::vector<cv::KeyPoint>& left, const std::vector<cv::KeyPoint>& right,
    const std::vector<cv::KeyPoint>& undistorted) {
    if (leftCount < 0) return undistorted.at(index);
    return index < static_cast<size_t>(leftCount) ? left.at(index)
        : right.at(index - static_cast<size_t>(leftCount));
}
}
#endif
