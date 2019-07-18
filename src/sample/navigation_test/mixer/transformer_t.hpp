//
// Created by User on 2019/7/18.
//

#ifndef PM1_SDK_TRANSFORMER_T_HPP
#define PM1_SDK_TRANSFORMER_T_HPP

#include <eigen3/Eigen/Core>

template<class scalar_t = double, int dimension = 2>
class transformer_t {
public:
    using linear_t      = Eigen::Matrix<scalar_t, dimension, dimension>;
    using core_t        = Eigen::Matrix<scalar_t, dimension, dimension + 1>;
    using coordinates_t = Eigen::Vector<scalar_t, dimension>;

private:
    // 核心变换矩阵
    core_t core;

public:
    transformer_t() : core(core_t::Zero()) {
        for (size_t i = 0; i < dimension; ++i)
            core(i, i) = 1;
    }
    
    void build(coordinates_t x0, coordinates_t y0, linear_t a) {
        auto        b = y0 - a * x0;
        for (size_t i = 0; i < dimension; ++i) {
            for (size_t j = 0; j < dimension; ++j)
                core(i, j)     = a(i, j);
            core(i, dimension) = b(i);
        }
    }
    
    void operator()(coordinates_t &value) const {
        using homogeneous_t = Eigen::Vector<scalar_t, dimension + 1>;
        
        homogeneous_t temp = homogeneous_t::Zero();
        
        for (size_t i = 0; i < dimension; ++i) temp[i] = value[i];
        temp[dimension - 1]                            = 1;
        value = core * temp;
    }
};

#endif //PM1_SDK_TRANSFORMER_T_HPP
