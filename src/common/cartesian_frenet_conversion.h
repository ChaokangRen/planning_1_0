#pragma once
#include <array>

#include "common.h"
#include "vec2d.h"
namespace jarvis {
namespace planning_lib {

// s_condition = [s, s_dot, s_ddot]
// s: longitudinal coordinate w.r.t reference line.
// s_dot: ds / dt
// s_ddot: d(s_dot) / dt
// d_condition = [d, d_prime, d_pprime]
// d: lateral coordinate w.r.t. reference line
// d_prime: dd / ds
// d_pprime: d(d_prime) / ds
// l: the same as d.
class CartesianFrenetConverter {
public:
    CartesianFrenetConverter() = delete;

    static void CartesianToFrenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double v, const double a,
                                  const double theta, const double kappa,
                                  std::array<double, 3> *const ptr_s_condition,
                                  std::array<double, 3> *const ptr_d_condition);

    static void CartesianToFrenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double x, const double y, double *ptr_s,
                                  double *ptr_d);

    static void FrenetToCartesian(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const std::array<double, 3> &s_condition,
                                  const std::array<double, 3> &d_condition,
                                  double *const ptr_x, double *const ptr_y,
                                  double *const ptr_theta,
                                  double *const ptr_kappa, double *const ptr_v,
                                  double *const ptr_a);

    // given sl point extract x, y, theta, kappa
    static double CalculateTheta(const double rtheta, const double rkappa,
                                 const double l, const double dl);

    static double CalculateKappa(const double rkappa, const double rdkappa,
                                 const double l, const double dl,
                                 const double ddl);

    static Vec2d CalculateCartesianPoint(const double rtheta,
                                         const Vec2d &rpoint, const double l);

    static double CalculateLateralDerivative(const double theta_ref,
                                             const double theta, const double l,
                                             const double kappa_ref);

    static double CalculateSecondOrderLateralDerivative(
        const double theta_ref, const double theta, const double kappa_ref,
        const double kappa, const double dkappa_ref, const double l);

    static void FrentToCartesian(const double ref_x, const double ref_y,
                                 const double ref_theta,
                                 const std::array<double, 3> &frenet_s,
                                 const std::array<double, 3> &frenet_l,
                                 double *pose_x_ptr, double *pose_y_ptr);

    static void FrentToCartesian(const double ref_x, const double ref_y,
                                 const double ref_theta, const double s,
                                 const double l, double *pose_x_ptr,
                                 double *pose_y_ptr);
};

}  // namespace planning_lib
}  // namespace jarvis