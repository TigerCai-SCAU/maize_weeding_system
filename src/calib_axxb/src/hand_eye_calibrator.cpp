/*
 * @Author: ylh 
 * @Date: 2024-05-22 20:30:24 
 * @Last Modified by: ylh 2252512364@qq.com
 * @Last Modified time: 2025-08-31 18:18:57
 */

#include "hand_eye_calibrator.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/version.h>
#include <yaml-cpp/yaml.h>

// ---------- 工具函数实现 ----------

HandEyeCalibrator::HandEyeCalibrator()
    : _onlyOptimizeRotation(false),
      _calibrationMode("axxb"),
      _maxIterations(200),
      _residualThreshold(1e-8),
      _timeToleranceSeconds(0.05),  // 默认 0.05s 容差
      _estQuat(Eigen::Quaterniond::Identity()),
      _estTrans(Eigen::Vector3d::Zero()), 
      _T_A_B_init(Eigen::Matrix4d::Identity()), 
      _initQuat(Eigen::Quaterniond::Identity()), 
      _initTrans(Eigen::Vector3d::Zero()),
      _skip(1),
      _BTimeOffsetSeconds(0.0),
      _maxInterpolationGapSeconds(0.2),
      _minMotionTranslation(0.2),
      _minMotionRotationDeg(1.0),
      _saveResultFilePath("calib_result.txt"),
      _APosesPath(""), _BPosesPath(""){}

int HandEyeCalibrator::initParams(std::string& cfgPath) {
    if (!std::filesystem::exists(cfgPath)) {
        std::cerr << "Config file not found: " << cfgPath << "\n";
        return -1;
    }
    YAML::Node cfg = YAML::LoadFile(cfgPath);
    _onlyOptimizeRotation = cfg["hand_eye_calibrator"]["only_optimize_rotation"].as<bool>();
    if (cfg["hand_eye_calibrator"]["calibration_mode"]) {
        _calibrationMode = cfg["hand_eye_calibrator"]["calibration_mode"].as<std::string>();
    }
    _maxIterations = cfg["hand_eye_calibrator"]["max_iterations"].as<int>();
    _residualThreshold = cfg["hand_eye_calibrator"]["residual_threshold"].as<double>();  // sum of squared residuals threshold
    _timeToleranceSeconds = cfg["hand_eye_calibrator"]["time_tolerance_seconds"].as<double>(); // allowed time diff when aligning
    _skip = cfg["hand_eye_calibrator"]["skip"].as<int>();
    _APosesPath = cfg["hand_eye_calibrator"]["A_poses_file"].as<std::string>();
    _BPosesPath = cfg["hand_eye_calibrator"]["B_poses_file"].as<std::string>();
    _saveResultFilePath = cfg["hand_eye_calibrator"]["save_result_file"].as<std::string>();
    if (cfg["hand_eye_calibrator"]["B_time_offset_seconds"]) {
        _BTimeOffsetSeconds = cfg["hand_eye_calibrator"]["B_time_offset_seconds"].as<double>();
    }
    if (cfg["hand_eye_calibrator"]["max_interpolation_gap_seconds"]) {
        _maxInterpolationGapSeconds = cfg["hand_eye_calibrator"]["max_interpolation_gap_seconds"].as<double>();
    }
    if (cfg["hand_eye_calibrator"]["min_motion_translation"]) {
        _minMotionTranslation = cfg["hand_eye_calibrator"]["min_motion_translation"].as<double>();
    }
    if (cfg["hand_eye_calibrator"]["min_motion_rotation_deg"]) {
        _minMotionRotationDeg = cfg["hand_eye_calibrator"]["min_motion_rotation_deg"].as<double>();
    }
    std::vector<double> T_A_B_init = cfg["hand_eye_calibrator"]["T_A_B_init"].as<std::vector<double>>();
    _T_A_B_init << T_A_B_init[0], T_A_B_init[1], T_A_B_init[2], T_A_B_init[3],
        T_A_B_init[4], T_A_B_init[5], T_A_B_init[6], T_A_B_init[7],
        T_A_B_init[8], T_A_B_init[9], T_A_B_init[10], T_A_B_init[11],
        T_A_B_init[12], T_A_B_init[13], T_A_B_init[14], T_A_B_init[15];
    _initQuat = _T_A_B_init.block<3,3>(0,0);
    _initTrans = _T_A_B_init.block<3,1>(0,3);
    return 0;
}

int HandEyeCalibrator::calibProcess() {
    if (!loadAPosesFromFile(_APosesPath)) {
            std::cout << "Failed to load A poses from file: " << _APosesPath << "\n";
            return -1;
        }
    if (!loadBPosesFromFile(_BPosesPath)) {
            std::cout << "Failed to load B poses from file: " << _BPosesPath << "\n";
            return -1;
        }
    timeAlignAndBuildPairs();
    if (_calibrationMode == "planar" || _calibrationMode == "planar_trajectory") {
        auto res = calibratePlanarTrajectory();
        std::cout << "Calibration result: " << res << "\n";
        printSummary();
        saveResultToFile(_saveResultFilePath);
        return res ? 0 : -1;
    }
    buildRelativeMotions(_skip);
    auto res = calibrate(_initQuat, _initTrans);
    std::cout << "Calibration result: " << res << "\n";
    printSummary();
    saveResultToFile(_saveResultFilePath);
    return 0;
}

void HandEyeCalibrator::clear() {
    _APoses.clear();
    _BPoses.clear();
    _BInterpolatedToA.clear();
    _relativeMotions.clear();
    _estQuat = Eigen::Quaterniond::Identity();
    _estTrans.setZero();
}

// 读取 pose 的文件，文件格式示例（每行）：
// timestamp(s) x y z qx qy qz qw
bool HandEyeCalibrator::loadAPosesFromFile(const std::string& filePath) {
    std::ifstream ifs(filePath);
    if (!ifs.is_open()) {
        std::cerr << "Failed open A file: " << filePath << "\n";
        return false;
    }
    _APoses.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        double ts, x,y,z,qx,qy,qz,qw;
        if (!(iss >> ts >> x >> y >> z >> qx >> qy >> qz >> qw)) {
            std::cerr << "Bad line in A file: " << line << "\n";
            continue;
        }
        TimedPose p;
        p.valid = true;
        p.timestamp = ts;
        p.t = Eigen::Vector3d(x,y,z);
        // 输入顺序 qx qy qz qw -> 构造 quaternion w-last
        p.q = Eigen::Quaterniond(qw, qx, qy, qz);
        p.q.normalize();
        _APoses.emplace_back(p);
    }
    std::sort(_APoses.begin(), _APoses.end(), [](auto &a, auto &b){ return a.timestamp < b.timestamp; });
    std::cout << "Loaded A poses: " << _APoses.size() << "\n";
    return true;
}

bool HandEyeCalibrator::loadBPosesFromFile(const std::string& filePath) {
    std::ifstream ifs(filePath);
    if (!ifs.is_open()) {
        std::cerr << "Failed open B file: " << filePath << "\n";
        return false;
    }
    _BPoses.clear();
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        // uint64_t ts; double x,y,z,qx,qy,qz,qw;
        double ts, x,y,z,qx,qy,qz,qw;
        if (!(iss >> ts >> x >> y >> z >> qx >> qy >> qz >> qw)) {
            std::cerr << "Bad line in B file: " << line << "\n";
            continue;
        }
        TimedPose p;
        p.valid = true;
        p.timestamp = ts + _BTimeOffsetSeconds;
        p.t = Eigen::Vector3d(x,y,z);
        p.q = Eigen::Quaterniond(qw, qx, qy, qz);
        p.q.normalize();
        _BPoses.emplace_back(p);
    }
    std::sort(_BPoses.begin(), _BPoses.end(), [](auto &a, auto &b){ return a.timestamp < b.timestamp; });
    std::cout << "Loaded B poses: " << _BPoses.size() << "\n";
    return true;
}

// 安全 slerp (避免反向)
bool HandEyeCalibrator::slerpSafe(const Eigen::Quaterniond& qa, const Eigen::Quaterniond& qb, double t, Eigen::Quaterniond& out) {
    double cosom = qa.w()*qb.w() + qa.x()*qb.x() + qa.y()*qb.y() + qa.z()*qb.z();
    Eigen::Quaterniond qb2 = qb;
    if (cosom < 0.0) { // take shortest path
        qb2.coeffs() *= -1.0;
        cosom = -cosom;
    }
    // if very close, use linear interp
    if (cosom > 0.9995) {
        out.coeffs() = ( (1.0 - t) * qa.coeffs() + t * qb2.coeffs() );
        out.normalize();
        return true;
    } else {
        double omega = std::acos(cosom);
        double so = std::sin(omega);
        double s1 = std::sin((1.0 - t) * omega) / so;
        double s2 = std::sin(t * omega) / so;
        out.coeffs() = s1 * qa.coeffs() + s2 * qb2.coeffs();
        out.normalize();
        return true;
    }
}

TimedPose HandEyeCalibrator::interpolatePose(const TimedPose& a, const TimedPose& b, double alpha) {
    TimedPose out;
    out.timestamp = static_cast<double>( (1.0 - alpha) * a.timestamp + alpha * b.timestamp );
    out.t = (1.0 - alpha) * a.t + alpha * b.t;
    Eigen::Quaterniond qout;
    slerpSafe(a.q, b.q, alpha, qout);
    out.q = qout;
    return out;
}

// 将 B 插值到 A 时间戳（假定 A 为参考）
void HandEyeCalibrator::timeAlignAndBuildPairs() {
    _BInterpolatedToA.clear();
    if (_APoses.empty() || _BPoses.empty()) {
        std::cerr << "Empty A or B poses; cannot align.\n";
        return;
    }
    std::cout << "A time range: " << _APoses.front().timestamp << " -> " << _APoses.back().timestamp << "\n";
    std::cout << "B time range: " << _BPoses.front().timestamp << " -> " << _BPoses.back().timestamp
              << " (offset applied: " << _BTimeOffsetSeconds << " s)\n";

    size_t idx = 0;
    for (const auto& lp : _APoses) {
        while (idx + 1 < _BPoses.size() && _BPoses[idx+1].timestamp < lp.timestamp) ++idx;
        if (idx + 1 >= _BPoses.size()) {
            break;
        }
        const auto& a = _BPoses[idx];
        const auto& b = _BPoses[idx+1];
        double dt = static_cast<double>(b.timestamp - a.timestamp);
        if (dt <= 0.0) continue;
        if (dt > _maxInterpolationGapSeconds) {
            TimedPose invalid;
            invalid.valid = false;
            _BInterpolatedToA.emplace_back(invalid);
            continue;
        }
        double alpha = (static_cast<double>(lp.timestamp - a.timestamp)) / dt;
        if (alpha < 0.0 || alpha > 1.0) {
            TimedPose invalid;
            invalid.valid = false;
            _BInterpolatedToA.emplace_back(invalid);
            continue;
        }
        TimedPose interp = interpolatePose(a, b, alpha);
        _BInterpolatedToA.emplace_back(interp);
    }
    // ensure same size as A poses: if sizes mismatch (could happen if B exhausted), trim A to match
    size_t common = std::min(_APoses.size(), _BInterpolatedToA.size());
    _APoses.resize(common);
    _BInterpolatedToA.resize(common);

    // optional: remove entries where interpolation missing (timestamp==0)
    std::vector<TimedPose> newA, newBInterp;
    for (size_t i=0;i<common;i++) {
        if (!_BInterpolatedToA[i].valid) continue;
        // also check time diff
        double diff = (_APoses[i].timestamp > _BInterpolatedToA[i].timestamp) ?
                        (_APoses[i].timestamp - _BInterpolatedToA[i].timestamp) :
                        (_BInterpolatedToA[i].timestamp - _APoses[i].timestamp);
        if (diff > _timeToleranceSeconds) continue;
        newA.emplace_back(_APoses[i]);
        newBInterp.emplace_back(_BInterpolatedToA[i]);
    }
    _APoses.swap(newA);
    _BInterpolatedToA.swap(newBInterp);
    std::cout << "After alignment, common poses: " << _APoses.size() << "\n";
}

static double rotationAngleDeg(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond normalized = q.normalized();
    double w = std::clamp(std::abs(normalized.w()), 0.0, 1.0);
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    return 2.0 * std::acos(w) * kRadToDeg;
}

// 构建相对变换对，默认用相邻帧 (i -> i+skip)
void HandEyeCalibrator::buildRelativeMotions(int skip) {
    _relativeMotions.clear();
    if (_APoses.size() <= skip) {
        std::cerr << "Not enough poses to build relative motions.\n";
        return;
    }
    for (size_t i = 0; i + skip < _APoses.size(); ++i) {
        const TimedPose& Li = _APoses[i];
        const TimedPose& Lj = _APoses[i+skip];
        const TimedPose& Ri = _BInterpolatedToA[i];
        const TimedPose& Rj = _BInterpolatedToA[i+skip];

        // compute A = Li^{-1} * Lj
        Eigen::Quaterniond qLi = Li.q;
        Eigen::Quaterniond qLj = Lj.q;
        Eigen::Quaterniond qA = qLi.conjugate() * qLj;
        qA.normalize();
        Eigen::Vector3d tA = qLi.conjugate() * (Lj.t - Li.t);

        // compute B = Ri^{-1} * Rj
        Eigen::Quaterniond qRi = Ri.q;
        Eigen::Quaterniond qRj = Rj.q;
        Eigen::Quaterniond qB = qRi.conjugate() * qRj;
        qB.normalize();
        Eigen::Vector3d tB = qRi.conjugate() * (Rj.t - Ri.t);

        double motionTranslation = std::max(tA.norm(), tB.norm());
        double motionRotationDeg = std::max(rotationAngleDeg(qA), rotationAngleDeg(qB));
        if (motionTranslation < _minMotionTranslation && motionRotationDeg < _minMotionRotationDeg) {
            continue;
        }

        RelativeMotion rm;
        rm.qA = qA;
        rm.qB = qB;
        rm.tA = tA;
        rm.tB = tB;
        _relativeMotions.emplace_back(rm);
    }
    std::cout << "Built relative motions: " << _relativeMotions.size() << "\n";
    if (_relativeMotions.size() < 3) {
        std::cerr << "Too few useful relative motions. Reduce skip/thresholds or record richer turning motion.\n";
    }
}

// helper: quaternion -> matrix
Eigen::Matrix3d HandEyeCalibrator::quatToMat(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix();
}

bool HandEyeCalibrator::calibratePlanarTrajectory() {
    if (_APoses.size() < 3 || _BInterpolatedToA.size() < 3 || _APoses.size() != _BInterpolatedToA.size()) {
        std::cerr << "Need at least 3 aligned pose pairs for planar trajectory calibration.\n";
        return false;
    }

    Eigen::Vector2d meanA = Eigen::Vector2d::Zero();
    Eigen::Vector2d meanB = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < _APoses.size(); ++i) {
        meanA += _APoses[i].t.head<2>();
        meanB += _BInterpolatedToA[i].t.head<2>();
    }
    meanA /= static_cast<double>(_APoses.size());
    meanB /= static_cast<double>(_BInterpolatedToA.size());

    double c = 0.0;
    double s = 0.0;
    for (size_t i = 0; i < _APoses.size(); ++i) {
        const Eigen::Vector2d a = _APoses[i].t.head<2>() - meanA;
        const Eigen::Vector2d b = _BInterpolatedToA[i].t.head<2>() - meanB;
        c += b.x() * a.x() + b.y() * a.y();
        s += b.x() * a.y() - b.y() * a.x();
    }

    const double yaw = std::atan2(s, c);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    Eigen::Matrix2d R2;
    R2 << cy, -sy,
          sy,  cy;
    const Eigen::Vector2d t2 = meanA - R2 * meanB;

    _estQuat = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    _estTrans = Eigen::Vector3d(t2.x(), t2.y(), 0.0);

    double sumSq = 0.0;
    double sum = 0.0;
    double maxErr = 0.0;
    for (size_t i = 0; i < _APoses.size(); ++i) {
        const Eigen::Vector2d pred = R2 * _BInterpolatedToA[i].t.head<2>() + t2;
        const double err = (pred - _APoses[i].t.head<2>()).norm();
        sumSq += err * err;
        sum += err;
        maxErr = std::max(maxErr, err);
    }

    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    std::cout << "Planar trajectory calibration B -> A\n";
    std::cout << "matched_pairs: " << _APoses.size() << "\n";
    std::cout << "yaw_deg: " << yaw * kRadToDeg << "\n";
    std::cout << "translation_xy: " << t2.x() << " " << t2.y() << "\n";
    std::cout << "rmse_xy: " << std::sqrt(sumSq / static_cast<double>(_APoses.size())) << " m\n";
    std::cout << "mean_xy: " << sum / static_cast<double>(_APoses.size()) << " m\n";
    std::cout << "max_xy: " << maxErr << " m\n";
    return true;
}

// ---------- Ceres Cost Functor ----------
// The residual is 6D: [angle_axis(q_err), trans_res]
// where q_err = qA * qX * qB^{-1} * qX^{-1}
// and trans_res = R_A * tX + tA - R_X * tB - tX
struct HandEyeResidual {
    HandEyeResidual(const Eigen::Quaterniond& qA_, const Eigen::Vector3d& tA_,
                    const Eigen::Quaterniond& qB_, const Eigen::Vector3d& tB_,
                    bool useTrans)
        : qA(qA_), tA(tA_), qB(qB_), tB(tB_), _useTrans(useTrans) {}

    template <typename T>
    bool operator()(const T* const qX_ptr/*size4*/, const T* const tX_ptr/*size3*/, T* residuals_ptr) const {
        // qA, qB are stored as double; cast to T
        Eigen::Quaternion<T> qA_T(T(qA.w()), T(qA.x()), T(qA.y()), T(qA.z()));
        Eigen::Quaternion<T> qB_T(T(qB.w()), T(qB.x()), T(qB.y()), T(qB.z()));
        Eigen::Quaternion<T> qX_T(qX_ptr[0], qX_ptr[1], qX_ptr[2], qX_ptr[3]); // w,x,y,z
        qX_T.normalize();

        // rotation residual:
        // q_err = qA * qX * qB^{-1} * qX^{-1}
        Eigen::Quaternion<T> qB_inv = qB_T.conjugate();
        Eigen::Quaternion<T> qX_inv = qX_T.conjugate();
        Eigen::Quaternion<T> qtmp = qA_T * qX_T * qB_inv * qX_inv;
        if (qtmp.w() < T(0)) {
            qtmp.coeffs() *= T(-1);
        }

        // convert qtmp to angle-axis (3D). formula: angle = 2*acos(w), axis = v / sin(angle/2)
        T w = qtmp.w();
        T x = qtmp.x();
        T y = qtmp.y();
        T z = qtmp.z();
        // ensure w in [-1,1]
        if (w > T(1)) w = T(1);
        if (w < T(-1)) w = T(-1);
        T angle = T(2) * acos(w);
        T s = sqrt(T(1) - w*w); // sin(angle/2)
        T eps = T(1e-12);
        T ax = T(0), ay = T(0), az = T(0);
        if (s < eps) {
            // small angle approx -> vector part ~ 0.5 * angle * axis => angle*axis ≈ 2*v
            ax = T(2) * x;
            ay = T(2) * y;
            az = T(2) * z;
        } else {
            ax = angle * x / s;
            ay = angle * y / s;
            az = angle * z / s;
        }
        residuals_ptr[0] = ax;
        residuals_ptr[1] = ay;
        residuals_ptr[2] = az;

        if (_useTrans) {
            // translation residual: R_A * tX + tA - R_X * tB - tX
            // R_A rotate tX: use quaternion rotation formula: q * v * q^{-1}
            Eigen::Matrix<T,3,1> tX(tX_ptr[0], tX_ptr[1], tX_ptr[2]);
            Eigen::Matrix<T,3,1> tA_T(T(tA.x()), T(tA.y()), T(tA.z()));
            Eigen::Matrix<T,3,1> tB_T(T(tB.x()), T(tB.y()), T(tB.z()));

            // rotate tX by qA_T
            Eigen::Quaternion<T> qA_local = qA_T;
            Eigen::Quaternion<T> qX_local = qX_T;
            Eigen::Matrix<T,3,1> RA_tX = qA_local * tX;
            Eigen::Matrix<T,3,1> RX_tB = qX_local * tB_T;

            Eigen::Matrix<T,3,1> tRes = RA_tX + tA_T - RX_tB - tX;
            residuals_ptr[3] = tRes(0);
            residuals_ptr[4] = tRes(1);
            residuals_ptr[5] = tRes(2);
        }
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Quaterniond& qA, const Eigen::Vector3d& tA,
                                       const Eigen::Quaterniond& qB, const Eigen::Vector3d& tB,
                                       bool useTrans) {
        if (useTrans) {
            return (new ceres::AutoDiffCostFunction<HandEyeResidual, 6, 4, 3>(
                new HandEyeResidual(qA, tA, qB, tB, useTrans)));
        } else {
            return (new ceres::AutoDiffCostFunction<HandEyeResidual, 3, 4, 3>(
                new HandEyeResidual(qA, tA, qB, tB, useTrans)));
        }
    }

    Eigen::Quaterniond qA, qB;
    Eigen::Vector3d tA, tB;
    bool _useTrans;
};

// ---------- 自定义回调：每次迭代计算成本并判断是否达到残差阈值 ----------
class ResidualCheckCallback : public ceres::IterationCallback {
public:
    ResidualCheckCallback(ceres::Problem* problem, double threshold)
        : _problem(problem), _threshold(threshold) {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override {
        // Evaluate current residuals' squared sum
        ceres::Problem::EvaluateOptions opts;
        double cost = 0.0;
        double total_squared_error = 0.0;
        std::vector<double> residuals;
        std::vector<double> jacobian_row_block; // not used
        if (_problem->Evaluate(opts, &cost, nullptr, nullptr, nullptr)) {
            total_squared_error = cost; // 'cost' is sum of squared residuals
        }
        std::cout << "Iter " << summary.iteration << " cost: " << total_squared_error << "\n";
        if (total_squared_error < _threshold) { // 达到阈值，收敛。注意这里只是其中一个收敛判断条件，若其他满足，也会判断收敛的
            std::cout << "[Callback] residual threshold reached: " << total_squared_error << " < " << _threshold << "\n";
            return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
        }
        return ceres::SOLVER_CONTINUE;
    }
private:
    ceres::Problem* _problem;
    double _threshold;
};

// ---------- 标定主函数 ----------
bool HandEyeCalibrator::calibrate(const Eigen::Quaterniond& initQuat, const Eigen::Vector3d& initTrans) {
    if (_relativeMotions.empty()) {
        std::cerr << "No relative motions; call buildRelativeMotions() first.\n";
        return false;
    }

    double qx[4];
    qx[0] = initQuat.w();
    qx[1] = initQuat.x();
    qx[2] = initQuat.y();
    qx[3] = initQuat.z();

    double tx[3];
    tx[0] = initTrans.x();
    tx[1] = initTrans.y();
    tx[2] = initTrans.z();

    ceres::Problem problem;
    problem.AddParameterBlock(qx, 4);
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
    problem.SetManifold(qx, new ceres::QuaternionManifold());
#else
    problem.SetParameterization(qx, new ceres::QuaternionParameterization());
#endif

    problem.AddParameterBlock(tx, 3);
    if (_onlyOptimizeRotation) {
        problem.SetParameterBlockConstant(tx);
    }

    for (const auto& rm : _relativeMotions) {
        ceres::CostFunction* cf = HandEyeResidual::Create(rm.qA, rm.tA, rm.qB, rm.tB, !_onlyOptimizeRotation);
        problem.AddResidualBlock(cf, nullptr, qx, tx);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = _maxIterations;
    options.linear_solver_type = ceres::DENSE_QR;
    options.num_threads = 4;
    options.function_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;

    ResidualCheckCallback callback(&problem, _residualThreshold);
    options.callbacks.emplace_back(&callback);
    options.update_state_every_iteration = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "Ceres summary:\n" << summary.BriefReport() << "\n";

    _estQuat = Eigen::Quaterniond(qx[0], qx[1], qx[2], qx[3]);
    _estQuat.normalize();
    _estTrans = Eigen::Vector3d(tx[0], tx[1], tx[2]);
    return summary.termination_type == ceres::CONVERGENCE ||
           summary.termination_type == ceres::NO_CONVERGENCE ||
           summary.termination_type == ceres::USER_SUCCESS;
}
bool HandEyeCalibrator::saveResultToFile(const std::string& filePath) const {
    std::ofstream ofs(filePath);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open output file: " << filePath << "\n";
        return false;
    }
    // 保存成4x4矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = _estQuat.toRotationMatrix();
    T.block<3,1>(0,3) = _estTrans;
    ofs << " T_A_B :\n"<< T << "\n " << " T_B_A :\n" << T.inverse() << "\n";
    // // write as: qw qx qy qz tx ty tz
    // ofs << _estQuat.w() << " " << _estQuat.x() << " " << _estQuat.y() << " " << _estQuat.z()
    //     << " " << _estTrans.x() << " " << _estTrans.y() << " " << _estTrans.z() << "\n";
    ofs.close();
    return true;
}

void HandEyeCalibrator::printSummary() const {
    // 保存成4x4矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = _estQuat.toRotationMatrix();
    T.block<3,1>(0,3) = _estTrans;
    std::cout << " T_A_B :\n"<< T << "\n " << " T_B_A :\n" << T.inverse() << "\n";
    // std::cout << "Estimated quaternion (w,x,y,z): " << _estQuat.w() << " " << _estQuat.x() << " " << _estQuat.y() << " " << _estQuat.z() << "\n";
    // std::cout << "Estimated translation: " << _estTrans.transpose() << "\n";
}
