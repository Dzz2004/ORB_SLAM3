/**
 * Minimal headless TartanAir baseline runner for ORB-SLAM3.
 *
 * Usage:
 *   run_tartanair VOCAB SETTINGS MODE MANIFEST IMU_CSV OUTPUT_DIR SEED
 *
 * MODE is rgb, rgb_imu, or rgbd_imu.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <ImuTypes.h>
#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace {

struct FrameRow {
    double timestamp = 0.0;
    std::string rgb_path;
    std::string depth_path;
    int gt_index = -1;
    int frame_id = -1;
};

struct ImuRow {
    double timestamp = 0.0;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
};

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }
    return fields;
}

std::vector<FrameRow> load_manifest(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open manifest: " + path);
    }
    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("Empty manifest: " + path);
    }
    std::vector<FrameRow> rows;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        const auto fields = split_csv(line);
        if (fields.size() != 5) {
            throw std::runtime_error("Invalid manifest row: " + line);
        }
        FrameRow row;
        row.timestamp = std::stod(fields[0]);
        row.rgb_path = fields[1];
        row.depth_path = fields[2];
        row.gt_index = std::stoi(fields[3]);
        row.frame_id = std::stoi(fields[4]);
        rows.push_back(row);
    }
    if (rows.empty()) {
        throw std::runtime_error("Manifest has no data rows: " + path);
    }
    for (std::size_t i = 1; i < rows.size(); ++i) {
        if (rows[i].timestamp <= rows[i - 1].timestamp) {
            throw std::runtime_error("Manifest timestamps must be strictly increasing");
        }
    }
    return rows;
}

std::vector<ImuRow> load_imu(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open IMU CSV: " + path);
    }
    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("Empty IMU CSV: " + path);
    }
    std::vector<ImuRow> rows;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }
        const auto fields = split_csv(line);
        if (fields.size() != 7) {
            throw std::runtime_error("Invalid IMU row: " + line);
        }
        ImuRow row;
        row.timestamp = std::stod(fields[0]);
        row.gx = std::stof(fields[1]);
        row.gy = std::stof(fields[2]);
        row.gz = std::stof(fields[3]);
        row.ax = std::stof(fields[4]);
        row.ay = std::stof(fields[5]);
        row.az = std::stof(fields[6]);
        rows.push_back(row);
    }
    for (std::size_t i = 1; i < rows.size(); ++i) {
        if (rows[i].timestamp <= rows[i - 1].timestamp) {
            throw std::runtime_error("IMU timestamps must be strictly increasing");
        }
    }
    return rows;
}

cv::Mat decode_depth(const std::string& path) {
    const cv::Mat encoded = cv::imread(path, cv::IMREAD_UNCHANGED);
    if (encoded.empty()) {
        throw std::runtime_error("Cannot read depth image: " + path);
    }
    if (encoded.type() == CV_32FC1) {
        return encoded.clone();
    }
    if (encoded.type() != CV_8UC4) {
        throw std::runtime_error(
            "Depth must be CV_32FC1 or lossless 4-channel float32 PNG: " + path);
    }
    cv::Mat decoded(encoded.rows, encoded.cols, CV_32FC1,
                    const_cast<unsigned char*>(encoded.data));
    return decoded.clone();
}

double valid_depth_ratio(const cv::Mat& depth) {
    cv::Mat finite_mask;
    cv::Mat lower_mask;
    cv::Mat upper_mask;
    cv::compare(depth, depth, finite_mask, cv::CMP_EQ);
    cv::compare(depth, 0.0, lower_mask, cv::CMP_GT);
    cv::compare(depth, 1000.0, upper_mask, cv::CMP_LT);
    cv::Mat valid_mask = finite_mask & lower_mask & upper_mask;
    return static_cast<double>(cv::countNonZero(valid_mask)) /
           static_cast<double>(depth.total());
}

std::string join_path(const std::string& directory, const std::string& filename) {
    if (!directory.empty() && directory.back() == '/') {
        return directory + filename;
    }
    return directory + "/" + filename;
}

ORB_SLAM3::System::eSensor parse_mode(const std::string& mode) {
    if (mode == "rgb") {
        return ORB_SLAM3::System::MONOCULAR;
    }
    if (mode == "rgb_imu") {
        return ORB_SLAM3::System::IMU_MONOCULAR;
    }
    if (mode == "rgbd_imu") {
        return ORB_SLAM3::System::IMU_RGBD;
    }
    throw std::runtime_error("Unknown mode: " + mode);
}

Sophus::SE3f load_camera_to_body(const std::string& settings_path) {
    cv::FileStorage settings(settings_path, cv::FileStorage::READ);
    if (!settings.isOpened()) {
        throw std::runtime_error("Cannot open settings: " + settings_path);
    }
    cv::Mat cv_tbc;
    settings["IMU.T_b_c1"] >> cv_tbc;
    if (cv_tbc.rows != 4 || cv_tbc.cols != 4 || cv_tbc.type() != CV_32F) {
        throw std::runtime_error("IMU.T_b_c1 must be a 4x4 float matrix");
    }
    Eigen::Matrix3f rotation;
    Eigen::Vector3f translation;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            rotation(row, column) = cv_tbc.at<float>(row, column);
        }
        translation(row) = cv_tbc.at<float>(row, 3);
    }
    return Sophus::SE3f(rotation, translation);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 8) {
            std::cerr
                << "Usage: run_tartanair VOCAB SETTINGS MODE MANIFEST IMU_CSV "
                   "OUTPUT_DIR SEED\n";
            return 2;
        }

        const std::string vocab = argv[1];
        const std::string settings = argv[2];
        const std::string mode = argv[3];
        const std::vector<FrameRow> frames = load_manifest(argv[4]);
        const std::vector<ImuRow> imu = load_imu(argv[5]);
        const std::string output = argv[6];
        const int seed = std::stoi(argv[7]);
        const auto sensor = parse_mode(mode);
        const Sophus::SE3f camera_to_body = load_camera_to_body(settings);

        ORB_SLAM3::System slam(vocab, settings, sensor, false, 0, mode);
        // System construction calls SeedRandOnce(0) from TwoViewReconstruction.
        // Re-seed afterward so the CLI seed controls the actual run.
        DUtils::Random::SeedRand(seed);
        cv::setRNGSeed(seed);
        std::ofstream tracking(join_path(output, "tracking.csv"));
        tracking
            << "timestamp,valid,state,quality_rgb,quality_depth,source,latency_ms,map_id\n";
        tracking << std::setprecision(17);

        std::ofstream trajectory(join_path(output, "trajectory_est.tum"));
        trajectory << std::setprecision(17);
        std::ofstream camera_trajectory(
            join_path(output, "trajectory_camera.tum"));
        camera_trajectory << std::setprecision(17);

        std::size_t imu_cursor = 0;
        std::size_t valid_poses = 0;
        for (std::size_t index = 0; index < frames.size(); ++index) {
            const FrameRow& frame = frames[index];
            cv::Mat rgb = cv::imread(frame.rgb_path, cv::IMREAD_COLOR);
            if (rgb.empty()) {
                throw std::runtime_error("Cannot read RGB image: " + frame.rgb_path);
            }

            cv::Mat depth;
            double depth_ratio = -1.0;
            if (sensor == ORB_SLAM3::System::IMU_RGBD) {
                depth = decode_depth(frame.depth_path);
                depth_ratio = valid_depth_ratio(depth);
            }

            std::vector<ORB_SLAM3::IMU::Point> measurements;
            if (index > 0 &&
                (sensor == ORB_SLAM3::System::IMU_MONOCULAR ||
                 sensor == ORB_SLAM3::System::IMU_RGBD)) {
                while (imu_cursor < imu.size() &&
                       imu[imu_cursor].timestamp <= frame.timestamp) {
                    const ImuRow& sample = imu[imu_cursor];
                    measurements.emplace_back(
                        sample.ax, sample.ay, sample.az,
                        sample.gx, sample.gy, sample.gz,
                        sample.timestamp);
                    ++imu_cursor;
                }
            }

            const auto start = std::chrono::steady_clock::now();
            Sophus::SE3f pose;
            if (sensor == ORB_SLAM3::System::IMU_RGBD) {
                pose = slam.TrackRGBD(rgb, depth, frame.timestamp, measurements);
            } else {
                pose = slam.TrackMonocular(rgb, frame.timestamp, measurements);
            }
            const auto end = std::chrono::steady_clock::now();
            const double latency_ms =
                std::chrono::duration<double, std::milli>(end - start).count();
            const int state = slam.GetTrackingState();
            const int map_id = slam.GetCurrentMapId();

            const bool finite = pose.matrix().allFinite();
            const bool valid =
                finite && !pose.matrix().isZero(1e-8) && state == 2;
            if (valid) {
                const Sophus::SE3f camera_to_world = pose.inverse();
                const Sophus::SE3f body_to_world =
                    camera_to_world * camera_to_body.inverse();
                const Eigen::Vector3f camera_translation =
                    camera_to_world.translation();
                const Eigen::Quaternionf camera_rotation(
                    camera_to_world.rotationMatrix());
                const Eigen::Vector3f translation = body_to_world.translation();
                const Eigen::Quaternionf rotation(body_to_world.rotationMatrix());
                camera_trajectory
                    << frame.timestamp << ' '
                    << camera_translation.x() << ' '
                    << camera_translation.y() << ' '
                    << camera_translation.z() << ' '
                    << camera_rotation.x() << ' '
                    << camera_rotation.y() << ' '
                    << camera_rotation.z() << ' '
                    << camera_rotation.w() << '\n';
                trajectory
                    << frame.timestamp << ' '
                    << translation.x() << ' ' << translation.y() << ' '
                    << translation.z() << ' '
                    << rotation.x() << ' ' << rotation.y() << ' '
                    << rotation.z() << ' ' << rotation.w() << '\n';
                ++valid_poses;
            }

            const int tracked_points =
                static_cast<int>(slam.GetTrackedKeyPointsUn().size());
            tracking << frame.timestamp << ',' << (valid ? 1 : 0) << ','
                     << state << ',' << tracked_points << ',';
            if (depth_ratio >= 0.0) {
                tracking << depth_ratio;
            } else {
                tracking << "NA";
            }
            tracking << ",slam," << latency_ms << ',' << map_id << '\n';

            if (index + 1 < frames.size()) {
                const double target_seconds =
                    frames[index + 1].timestamp - frame.timestamp;
                const double elapsed_seconds =
                    std::chrono::duration<double>(end - start).count();
                if (elapsed_seconds < target_seconds) {
                    std::this_thread::sleep_for(
                        std::chrono::duration<double>(
                            target_seconds - elapsed_seconds));
                }
            }
        }

        tracking.close();
        trajectory.close();
        camera_trajectory.close();
        slam.Shutdown();
        slam.SaveTrajectoryEuRoC(
            join_path(output, "trajectory_orb_raw.tum"));
        slam.SaveKeyFrameTrajectoryTUM(
            join_path(output, "keyframe_trajectory.tum"));

        std::cout << "frames=" << frames.size()
                  << " valid_poses=" << valid_poses
                  << " mode=" << mode
                  << " seed=" << seed << '\n';
        return valid_poses > 0 ? 0 : 3;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
