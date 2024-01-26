#include "runge_kutta.h"

#include <vector>

#include "sglog/sglog.h"

namespace jarvis {
namespace planning_lib {
inline std::vector<double> operator+(const std::vector<double> &vec1,
                                     const std::vector<double> &vec2) {
    if (vec1.size() != vec2.size()) {
        SG_INFO("vec1 and vec2 are not equal length");
        return std::vector<double>();
    }
    std::vector<double> vec3(vec1);
    for (int i = 0; i < vec1.size(); ++i) {
        vec3[i] = vec1[i] + vec2[i];
    }
    return vec3;
}
inline std::vector<double> operator*(double num,
                                     const std::vector<double> &vec) {
    std::vector<double> vec_ans(vec.size(), 0);
    for (int i = 0; i < vec.size(); ++i) {
        vec_ans[i] = num * vec[i];
    }
    return vec_ans;
}
std::vector<double> RungeKutta::KineticsEqu(const std::vector<double> &kesi,
                                            double delta_f, double acc) {
    double dot_x = kesi[0];
    double dot_y = kesi[1];
    double phi = kesi[2];
    double dot_phi = kesi[3];
    // double Y = kesi[4];
    // double X = kesi[5];
    std::vector<double> kesi_new(6, 0);
    kesi_new[0] = acc;
    kesi_new[1] = -dot_x * dot_phi +
                  2 *
                      (cf_ * (delta_f - (dot_y + lf_ * dot_phi) / dot_x) +
                       cr_ * (lr_ * dot_phi - dot_y) / dot_x) /
                      mass_;
    kesi_new[2] = dot_phi;
    kesi_new[3] = 2 *
                  (lf_ * cf_ * (delta_f - (dot_y + lf_ * dot_phi) / dot_x) -
                   lr_ * cr_ * (lr_ * dot_phi - dot_y) / dot_x) /
                  iz_;
    kesi_new[4] = dot_x * sin(phi) + dot_y * cos(phi);
    kesi_new[5] = dot_x * cos(phi) - dot_y * sin(phi);
    return kesi_new;
}

std::vector<double> RungeKutta::Solver(const std::vector<double> &kesi,
                                       double delta_f, double acc,
                                       double time_len) {
    double step_len = 0.01;
    int n = int(time_len / step_len);
    if (n < 1) n = 1;
    std::vector<double> vec_ans(kesi);
    if (std::fabs(kesi[0]) < epsilon_) {
        vec_ans[0] += acc * time_len;
        vec_ans[4] += kesi[0] * sin(kesi[2]) * time_len;
        vec_ans[5] += kesi[0] * cos(kesi[2]) * time_len;

        double dx = kesi[0] * sin(kesi[2]);
        double dy = kesi[0] * cos(kesi[2]);
    } else {
        for (int i = 0; i < n; ++i) {
            std::vector<double> k1 = KineticsEqu(kesi, delta_f, acc);
            std::vector<double> k2 =
                KineticsEqu(kesi + 0.5 * step_len * k1, delta_f, acc);
            std::vector<double> k3 =
                KineticsEqu(kesi + 0.5 * step_len * k2, delta_f, acc);
            std::vector<double> k4 =
                KineticsEqu(kesi + step_len * k3, delta_f, acc);

            vec_ans =
                vec_ans + (1.0 / 6) * step_len * (k1 + 2 * k2 + 2 * k3 + k4);
        }
    }
    return vec_ans;
}
}  // namespace planning_lib
}  // namespace jarvis