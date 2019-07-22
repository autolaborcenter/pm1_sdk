//
// Created by User on 2019/7/18.
//

#ifndef PM1_SDK_TRANSFORMER_T_HPP
#define PM1_SDK_TRANSFORMER_T_HPP

#include <eigen3/Eigen/Core>

template<class scalar_t = double, size_t dimension = 2>
class transformer_t {
    using linear_t      = Eigen::Matrix<scalar_t, dimension, dimension>;
    using homogeneous_t = Eigen::Matrix<scalar_t, dimension, dimension + 1>;
    using coordinates_t = Eigen::Vector<scalar_t, dimension>;
    
    // 核心变换矩阵
    homogeneous_t core;
public:
    transformer_t() : core(homogeneous_t::Zero()) {}
    
    void build(coordinates_t x0, coordinates_t y0, linear_t a) {
        auto        b = y0 - a * x0;
        for (size_t i = 0; i < dimension; ++i) {
            for (size_t j = 0; j < dimension; ++j)
                core(i, j)     = a(i, j);
            core(i, dimension) = b(i);
        }
    }
    
    [[nodiscard]] coordinates_t
    operator()(const coordinates_t &value) const {
        Eigen::Vector<scalar_t, dimension + 1> temp;
        
        for (size_t i = 0; i < dimension; ++i)
            temp[i]     = value[i];
        temp[dimension] = 1;
        return core * temp;
    }
};

#endif //PM1_SDK_TRANSFORMER_T_HPP
