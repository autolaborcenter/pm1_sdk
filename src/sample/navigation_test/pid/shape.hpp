//
// Created by User on 2019/7/3.
//

#ifndef PM1_SDK_SHAPE_HPP
#define PM1_SDK_SHAPE_HPP


#include <vector>
#include <stdexcept>
#include <cmath>

constexpr auto PI = 3.141592654;

struct point_t {
    double x, y;
};

struct shape {
    explicit shape(size_t point_count)
        : _point_count(point_count) {}
    
    [[nodiscard]] size_t
    point_count() const {
        return _point_count;
    }
    
    [[nodiscard]] virtual point_t
    operator[](size_t index) const = 0;

private:
    size_t _point_count;
};

struct circle : public shape {
    point_t center;
    double  radius;
    
    [[nodiscard]] std::vector<point_t>
    sample(size_t n) const {
        if (n == 0) throw std::logic_error("");
        
        auto step              = 2 * PI / n;
        
        std::vector<point_t> points(n);
        for (size_t          i = 0; i < n; ++i)
            points[i] = {
                radius * std::cos(i * step) + center.x,
                radius * std::sin(i * step) + center.y
            };
        return points;
    }
};


#endif //PM1_SDK_SHAPE_HPP
