// Small numerical/configuration checks; no hardware accuracy claim.
#include "System.h"
#include "Settings.h"
#include "KeypointIndex.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    try {
        // Fisheye relocalization must address right keys using the concatenated
        // descriptor index, even when there are no undistorted right keys.
        std::vector<cv::KeyPoint> left(1), right(1), undistorted;
        left[0].angle = 10; right[0].angle = 40;
        if (ORB_SLAM3::KeypointAt(1, 1, left, right, undistorted).angle != 40)
            throw std::runtime_error("fisheye right-key index mismatch");
        bool rejected = false;
        try { ORB_SLAM3::KeypointAt(2, 1, left, right, undistorted); }
        catch (const std::out_of_range&) { rejected = true; }
        if (!rejected) throw std::runtime_error("out-of-range key index accepted");
        ORB_SLAM3::Settings settings(argv[1], ORB_SLAM3::System::STEREO);
        cv::FileStorage fs(argv[1], cv::FileStorage::READ);
        for (int camera = 1; camera <= 2; ++camera) {
            auto* model = camera == 1 ? settings.camera1() : settings.camera2();
            for (int k = 1; k <= 4; ++k) {
                const float expected = fs["Camera" + std::to_string(camera) + ".k" + std::to_string(k)];
                if (std::abs(model->getParameter(k+3)-expected) > 1e-7)
                    throw std::runtime_error("camera distortion read from the wrong configuration field");
            }
            Eigen::Vector3f ray(.1f, -.2f, 1.f);
            auto pixel = model->project(ray);
            auto restored = model->unprojectEig(cv::Point2f(pixel.x(), pixel.y()));
            if ((restored / restored.z() - ray).norm() > 1e-4f)
                throw std::runtime_error("KB projection/unprojection mismatch");
        }
        auto transform = settings.Tlr();
        if ((transform.matrix() * transform.inverse().matrix() - Eigen::Matrix4f::Identity()).norm() > 1e-5f)
            throw std::runtime_error("Sophus/Eigen inverse mismatch");
        Eigen::Matrix3f identity = Eigen::Matrix3f::Identity(), restored;
        cv::Mat cvIdentity;
        cv::eigen2cv(identity, cvIdentity);
        cv::cv2eigen(cvIdentity, restored);
        if ((identity-restored).norm() > 1e-7f) throw std::runtime_error("OpenCV/Eigen conversion mismatch");
        std::cout << "PASS: distinct stereo calibration, KB projection, Sophus inverse, OpenCV/Eigen conversion\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
    return 0;
}
