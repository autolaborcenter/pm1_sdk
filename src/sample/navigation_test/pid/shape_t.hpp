//
// Created by User on 2019/7/3.
//

#ifndef PM1_SDK_SHAPE_T_HPP
#define PM1_SDK_SHAPE_T_HPP


#include "point_t.h"

#include <vector>
#include <stdexcept>
#include <cmath>
#include <thread>
#include <mutex>

constexpr auto PI = 3.141592654;

/**
 * 形状
 */
struct shape_t {
    explicit shape_t(size_t point_count)
        : _point_count(point_count),
          size_buffer(0) {
        if (point_count < 3)
            throw std::logic_error("vertex count must be more than 3");
    }
    
    [[nodiscard]] size_t
    point_count() const {
        return _point_count;
    }
    
    [[nodiscard]] virtual point_t
    operator[](size_t index) const = 0;
    
    [[nodiscard]] double
    size() const {
        std::call_once(flag, [this] { set_size(); });
        return size_buffer;
    }
    
    [[nodiscard]] virtual std::vector<point_t>
    to_vector() const {
        std::vector<point_t> result(_point_count);
        
        for (size_t i = 0; i < _point_count; ++i)
            result[i] = operator[](i);
        
        return result;
    }

protected:
    mutable double size_buffer;
    
    virtual void set_size() const {
        for (size_t i = 0; i < _point_count; ++i) {
            auto a = operator[](i),
                 b = operator[]((i + 1) % _point_count);
            size_buffer += a.x * b.y - a.y * b.x;
        }
        size_buffer /= 2;
    };

private:
    size_t                 _point_count;
    mutable std::once_flag flag;
};

/**
 * 圆
 */
struct circle_t : public shape_t {
    point_t center;
    double  direction;
    
    circle_t(size_t point_count,
             double radius,
             point_t center = {0, 0},
             double direction = 0)
        : shape_t(point_count),
          center(center),
          radius(radius),
          step(2 * PI / point_count),
          direction(direction) {}
    
    [[nodiscard]] point_t
    operator[](size_t index) const override {
        return {
            radius * std::cos(index * step) + center.x,
            radius * std::sin(index * step) + center.y
        };
    }
    
    [[nodiscard]] bool
    check_inside(point_t point) const {
        auto dx = point.x - center.x,
             dy = point.y - center.y;
        return dx * dx + dy * dy < radius * radius;
    }

private:
    double radius, step;
};

/**
 * 任意多边形
 */
struct any_shape : public shape_t {
    explicit any_shape(std::vector<point_t> vertex)
        : shape_t(vertex.size()), vertex(std::move(vertex)) {}
    
    [[nodiscard]] point_t
    operator[](size_t index) const override {
        return vertex[index];
    }
    
    [[nodiscard]] std::vector<point_t>
    to_vector() const override {
        return vertex;
    }

private:
    std::vector<point_t> vertex;
};


#endif //PM1_SDK_SHAPE_T_HPP
