#include "common.h"

#include "sglog/sglog.h"
namespace jarvis {
namespace planning_lib {

double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += 2.0 * M_PI;
    }
    return a - M_PI;
}

Eigen::MatrixXd ComputeCostFunctionFristMatrix(
    const std::vector<double> &s_order) {
    Eigen::MatrixXd matrix_tmp{
        {1, s_order[0], s_order[1], s_order[2], s_order[3], s_order[4]},
        {s_order[0], s_order[1], s_order[2], s_order[3], s_order[4],
         s_order[5]},
        {s_order[1], s_order[2], s_order[3], s_order[4], s_order[5],
         s_order[6]},
        {s_order[2], s_order[3], s_order[4], s_order[5], s_order[6],
         s_order[7]},
        {s_order[3], s_order[4], s_order[5], s_order[6], s_order[7],
         s_order[8]},
        {s_order[4], s_order[5], s_order[6], s_order[7], s_order[8],
         s_order[9]}};
    return matrix_tmp;
}

Eigen::MatrixXd ComputeCostFunctionSecondMatrix(
    const std::vector<double> &s_order) {
    Eigen::MatrixXd matrix_tmp{
        {0, 0, 0, 0, 0, 0},
        {0, 1, 2 * s_order[0], 3 * s_order[1], 4 * s_order[2], 5 * s_order[3]},
        {0, 2 * s_order[0], 4 * s_order[1], 6 * s_order[2], 8 * s_order[3],
         10 * s_order[4]},
        {0, 3 * s_order[1], 6 * s_order[2], 9 * s_order[3], 12 * s_order[4],
         15 * s_order[5]},
        {0, 4 * s_order[2], 8 * s_order[3], 12 * s_order[4], 16 * s_order[5],
         20 * s_order[6]},
        {0, 5 * s_order[3], 10 * s_order[4], 15 * s_order[5], 20 * s_order[6],
         25 * s_order[7]}};

    return matrix_tmp;
}

Eigen::MatrixXd ComputeCostFunctionThirdMatrix(
    const std::vector<double> &s_order) {
    Eigen::MatrixXd matrix_tmp{
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 4, 12 * s_order[0], 24 * s_order[1], 40 * s_order[2]},
        {0, 0, 12 * s_order[0], 36 * s_order[1], 72 * s_order[2],
         120 * s_order[3]},
        {0, 0, 24 * s_order[1], 72 * s_order[2], 144 * s_order[3],
         240 * s_order[4]},
        {0, 0, 40 * s_order[2], 120 * s_order[3], 240 * s_order[4],
         400 * s_order[5]}};
    return matrix_tmp;
}

Eigen::MatrixXd ComputeCostFunctionForthMatrix(
    const std::vector<double> &s_order) {
    Eigen::MatrixXd matrix_tmp{
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 36, 144 * s_order[0], 360 * s_order[1]},
        {0, 0, 0, 144 * s_order[0], 576 * s_order[1], 1440 * s_order[2]},
        {0, 0, 0, 360 * s_order[1], 1440 * s_order[2], 3600 * s_order[3]}};
    return matrix_tmp;
}

// const double kMathEpsilon = 1e-6;
double LinearInterpolateOfTheta(const double a0, const double t0,
                                const double a1, const double t1,
                                const double t) {
    if (std::abs(t1 - t0) <= kMathEpsilon) {
        SG_ERROR(" input time difference is too small");
        return NormalizeAngle(a0);
    }
    double a0_n = NormalizeAngle(a0);
    double a1_n = NormalizeAngle(a1);
    double diff = a1_n - a0_n;
    if (diff > M_PI) {
        diff = diff - 2 * M_PI;
    } else if (diff < -M_PI) {
        diff = diff + 2 * M_PI;
    }
    double r = (t - t0) / (t1 - t0);
    double a = a0_n + diff * r;
    return NormalizeAngle(a);
}

Vec2d PointRotate(const Vec2d &point, const double x, const double y,
                  const double theta) {
    double x_rotate =
        point.x() * std::cos(theta) + point.y() * std::sin(theta) - x;
    double y_rotate =
        -point.x() * std::sin(theta) + point.y() * std::cos(theta) - y;
    return Vec2d(x_rotate, y_rotate);
}
bool IsNumeric(const std::string &str) {
    if (str.empty() == true) {
        return false;
    }
    for (auto &ite : str) {
        if (std::isdigit(ite) == false) {
            return false;
        }
    }
    return true;
}

void LeastSquareFitCurve(const std::vector<Vec2d> &points, const int32_t order,
                         FitCurve &fit_curve) {
    Eigen::MatrixXd Y(points.size(), 1);
    Eigen::MatrixXd X(points.size(), order);

    for (int32_t i = 0; i < points.size(); ++i) {
        Y(i, 0) = points[i].y();
        for (int32_t j = 0; j < order; ++j) {
            if (j == 0) {
                X(i, j) = 1;
            } else {
                X(i, j) = X(i, j - 1) * points[i].x();
            }
        }
        // SG_INFO("x = %lf,y = %lf", points[i].x(), points[i].y());
    }

    Eigen::MatrixXd param_mat =
        (X.transpose() * X).inverse() * X.transpose() * Y;
    for (int32_t i = 0; i < param_mat.size(); ++i) {
        fit_curve.fit_param.emplace_back(param_mat(i, 0));
    }

    // SG_INFO("b = %lf,k = %lf", fit_curve.fit_param[0],
    // fit_curve.fit_param[1]);
    fit_curve.start = points.front();
    fit_curve.end = points.back();
}
void TurnPrint(const TURN turn) {
    if (turn == TURN::LEFT) {
        SG_INFO("turn left");
    } else if (turn == TURN::RIGHT) {
        SG_INFO("turn right");
    } else if (turn == TURN::STRAIGHT) {
        SG_INFO("turn straight");
    } else {
        SG_INFO("turn none");
    }
}

void PnCLaneAddStarightLine(PnCLane &pnc_lanes) {
    if (pnc_lanes.lanes.size() == 0) {
        FitCurve curve;
        curve.fit_param = std::vector<double>(2, 0);
        curve.can_left_change = false;
        curve.can_right_change = false;
        curve.id = "virtual";
        curve.digit_id = 0;
        curve.source = Source::NONE;
        pnc_lanes.lanes.emplace_back(curve);
        pnc_lanes.self_car_lane_cnt = 0;
        // SG_WARN("pnc lane add zero line");
    }
}

double AverageSpeed(const double v0, const double ref_v, const double a,
                    const double l) {
    double t0 = (-v0 + std::sqrt(v0 * v0 + 2 * a * l)) / a;
    double vl = v0 + a * t0;
    if (vl < ref_v) {
        return l / t0;
    }

    double l1 = (ref_v * ref_v - v0 * v0) / (2 * a);
    double t1 = (ref_v - v0) / a;
    double t2 = (l - l1) / ref_v;
    return l / (t1 + t2);
}

}  // namespace planning_lib
}  // namespace jarvis