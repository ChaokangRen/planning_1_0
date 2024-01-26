#pragma once

#include <cmath>

namespace jarvis {
namespace planning_lib {
constexpr double kMathEpsilon = 1e-10;

class Vec2d {
public:
    constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

    constexpr Vec2d() noexcept : Vec2d(0, 0) {}

    // Creates a unit-vector with angle
    static Vec2d CreateUnitVec2d(const double angle);

    double x() const {
        return x_;
    }

    double y() const {
        return y_;
    }

    void set_x(const double x) {
        x_ = x;
    }

    void set_y(const double y) {
        y_ = y;
    }

    double Length() const;

    double LengthSquare() const;

    double Angle() const;

    void Normalize();

    double DistanceTo(const Vec2d &other) const;

    double DistanceSquareTo(const Vec2d &other) const;

    double CrossProd(const Vec2d &other) const;

    double InnerProd(const Vec2d &other) const;

    Vec2d Rotate(const double angle) const;

    void SelfRotate(const double angle);

    Vec2d operator+(const Vec2d &other) const;

    Vec2d operator-(const Vec2d &other) const;

    Vec2d operator*(const double ratio) const;

    Vec2d operator/(const double ratio) const;

    Vec2d &operator+=(const Vec2d &other);

    Vec2d &operator-=(const Vec2d &other);

    Vec2d &operator*=(const double ratio);

    Vec2d &operator/=(const double ratio);

    bool operator==(const Vec2d &other) const;

protected:
    double x_ = 0.0;
    double y_ = 0.0;
};
Vec2d operator*(const double ratio, const Vec2d &vec);
}  // namespace planning_lib
}  // namespace jarvis