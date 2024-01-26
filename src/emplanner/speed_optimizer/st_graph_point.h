#pragma once
#include <limits>

#include "common.h"
#include "st_point.h"
namespace jarvis {
namespace planning_lib {
class StGraphPoint {
public:
    std::uint32_t IndexS() const;
    std::uint32_t IndexT() const;

    const STPoint &Point() const;
    const StGraphPoint *PrePoint() const;

    float ReferenceCost() const;
    float ObstacleCost() const;
    float TotalCost() const;

    void Init(const std::uint32_t index_t, const std::uint32_t index_s,
              const STPoint &st_point);

    // given reference speed profile, reach the cost, including position
    void SetReferenceCost(const float reference_cost);

    // given obstacle info, get the cost;
    void SetObstacleCost(const float obs_cost);

    // total cost
    void SetTotalCost(const float total_cost);

    void SetPrePoint(const StGraphPoint &pre_point);

private:
    STPoint point_;
    const StGraphPoint *pre_point_ = nullptr;
    std::uint32_t index_s_ = 0;
    std::uint32_t index_t_ = 0;

    float reference_cost_ = 0.0;
    float obstacle_cost_ = 0.0;
    float total_cost_ = std::numeric_limits<float>::infinity();
};
}  // namespace planning_lib
}  // namespace jarvis