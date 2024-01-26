#pragma once
#include <vector>

#include "common.h"
namespace jarvis {
namespace planning_lib {
class QuinticPolynomialCurve {
public:
    QuinticPolynomialCurve() = default;
    QuinticPolynomialCurve(std::vector<double> coefs, SLPoint start,
                           SLPoint end)
        : coef_(coefs), start_pos_(start), end_pos_(end) {}

    QuinticPolynomialCurve(std::vector<double> coefs);

    explicit QuinticPolynomialCurve(const SLPoint &start_pos, const double dl0,
                                    const double ddl0, const SLPoint &end_pos,
                                    const double dl1, const double ddl1);
    double CalcuteCurveValue(double pos_x) const;

    double CalcuteDerivativeCurveValue(double pos_x) const;

    double CalcuteSecDerivativeCurveValue(double pos_x) const;

    double CalcuteThdDerivativeCurveValue(double pos_x) const;

    std::vector<double> Coeffs() const {
        return coef_;
    }
    SLPoint Front(void) const {
        return start_pos_;
    }
    SLPoint Back(void) const {
        return end_pos_;
    }

private:
    std::vector<double> coef_;
    SLPoint start_pos_;
    SLPoint end_pos_;
};
}  // namespace planning_lib
}  // namespace jarvis