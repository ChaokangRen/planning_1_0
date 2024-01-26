#include "quintic_polynomial_curve.h"

#include <vector>

#include "common.h"
namespace jarvis {
namespace planning_lib {
QuinticPolynomialCurve::QuinticPolynomialCurve(std::vector<double> coefs)
    : coef_(coefs) {
    start_pos_.s = 0;
    start_pos_.l = coef_[0];
    end_pos_.s = 1;
    end_pos_.l = CalcuteCurveValue(1);
}
QuinticPolynomialCurve::QuinticPolynomialCurve(
    const SLPoint &start_pos, const double dl0, const double ddl0,
    const SLPoint &end_pos, const double dl1, const double ddl1)
    : start_pos_(start_pos), end_pos_(end_pos) {
    double a0 = start_pos.l;
    double a1 = dl0;
    double a2 = 0.5 * ddl0;

    double length = end_pos_.s - start_pos_.s;
    double length2 = length * length;
    double length3 = length * length2;
    double c0 = (end_pos.l - a0 - a1 * length - a2 * length2) / length3;
    double c1 = (dl1 - a1 - 2 * a2 * length) / length2;
    double c2 = (ddl1 - 2 * a2) / length;

    double a3 = 10 * c0 - 4 * c1 + 0.5 * c2;
    double a4 = (-15 * c0 + 7 * c1 - c2) / length;
    double a5 = (12 * c0 - 6 * c1 + c2) / (2 * length2);

    coef_ = std::vector<double>{a0, a1, a2, a3, a4, a5};
}
double QuinticPolynomialCurve::CalcuteCurveValue(double pos_x) const {
    return coef_[0] + coef_[1] * pos_x + coef_[2] * pow(pos_x, 2) +
           coef_[3] * pow(pos_x, 3) + coef_[4] * pow(pos_x, 4) +
           coef_[5] * pow(pos_x, 5);
}
double QuinticPolynomialCurve::CalcuteDerivativeCurveValue(double pos_x) const {
    return coef_[1] + 2 * coef_[2] * pos_x + 3 * coef_[3] * pow(pos_x, 2) +
           4 * coef_[4] * pow(pos_x, 3) + 5 * coef_[5] * pow(pos_x, 4);
}
double QuinticPolynomialCurve::CalcuteSecDerivativeCurveValue(
    double pos_x) const {
    return 2 * coef_[2] + 6 * coef_[3] * pos_x + 12 * coef_[4] * pow(pos_x, 2) +
           20 * coef_[5] * pow(pos_x, 3);
}
double QuinticPolynomialCurve::CalcuteThdDerivativeCurveValue(
    double pos_x) const {
    return 6 * coef_[3] + 24 * coef_[4] * pos_x + 60 * coef_[5] * pow(pos_x, 2);
}
}  // namespace planning_lib
}  // namespace jarvis