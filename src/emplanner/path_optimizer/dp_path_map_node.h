#pragma once

#include <vector>

#include "common.h"
#include "quintic_polynomial_curve.h"

namespace jarvis {
namespace planning_lib {
struct DpPathMapNode {
public:
    DpPathMapNode() {}

    DpPathMapNode(const SLPoint point, const DpPathMapNode *prev_node)
        : sl_point_(point), prev_min_cost_node_(prev_node) {}

    DpPathMapNode(const SLPoint point, const DpPathMapNode *prev_node,
                  const DpCost cost)
        : sl_point_(point), prev_min_cost_node_(prev_node), min_cost_(cost) {}

    void UpdateCost(const DpPathMapNode *prev_node,
                    const QuinticPolynomialCurve &curve, const DpCost &cost) {
        if (cost <= min_cost_) {
            min_cost_ = cost;
            min_cost_curve_ = curve;
            prev_min_cost_node_ = prev_node;
        }
    }

    SLPoint sl_point_;
    DpCost min_cost_;
    const DpPathMapNode *prev_min_cost_node_ = nullptr;
    QuinticPolynomialCurve min_cost_curve_;
};

}  // namespace planning_lib
}  // namespace jarvis
