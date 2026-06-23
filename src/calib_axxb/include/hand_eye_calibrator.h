/*
 * @Author: ylh
 * @Date: 2024-05-22 20:20:24
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2025-08-31 16:11:15
 */

#ifndef HAND_EYE_CALIBRATOR_H
#define HAND_EYE_CALIBRATOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

struct TimedPose {
    bool valid = false;
    double timestamp = 0.0;
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

struct RelativeMotion {
    Eigen::Vector3d tA, tB;
    Eigen::Quaterniond qA, qB;
};

class HandEyeCalibrator {
public:
    HandEyeCalibrator();
    using Ptr = std::shared_ptr<HandEyeCalibrator>;
    int initParams(std::string& cfgPath);
    int calibProcess();

private:
    // Input and derived pose data.
    std::vector<TimedPose> _APoses;
    std::vector<TimedPose> _BPoses;
    std::vector<TimedPose> _BInterpolatedToA;
    std::vector<RelativeMotion> _relativeMotions;

    // Calibration configuration.
    Eigen::Matrix4d _T_A_B_init;
    Eigen::Quaterniond _initQuat;
    Eigen::Vector3d _initTrans;
    int _skip;
    std::string _saveResultFilePath;
    std::string _APosesPath, _BPosesPath;
    bool _onlyOptimizeRotation;
    std::string _calibrationMode;
    int _maxIterations;
    double _residualThreshold;
    double _timeToleranceSeconds;
    double _BTimeOffsetSeconds;
    double _maxInterpolationGapSeconds;
    double _minMotionTranslation;
    double _minMotionRotationDeg;

    Eigen::Quaterniond _estQuat;
    Eigen::Vector3d _estTrans;

    TimedPose interpolatePose(const TimedPose& a, const TimedPose& b, double alpha);
    bool slerpSafe(const Eigen::Quaterniond& qa, const Eigen::Quaterniond& qb, double t, Eigen::Quaterniond& out);
    Eigen::Matrix3d quatToMat(const Eigen::Quaterniond& q);

    // Read pose files with format: timestamp(s) x y z qx qy qz qw.
    bool loadAPosesFromFile(const std::string& filePath);
    bool loadBPosesFromFile(const std::string& filePath);

    // Interpolate B poses onto A timestamps.
    void timeAlignAndBuildPairs();

    // Build relative transform pairs for AX=XB.
    void buildRelativeMotions(int skip = 1);
    bool calibratePlanarTrajectory();

    // Run Ceres optimization.
    bool calibrate(const Eigen::Quaterniond& initQuat = Eigen::Quaterniond::Identity(),
                   const Eigen::Vector3d& initTrans = Eigen::Vector3d::Zero());

    bool saveResultToFile(const std::string& filePath) const;

    void clear();

    void printSummary() const;
};

#endif // HAND_EYE_CALIBRATOR_H
