//
// Created by User on 2019/7/29.
//

#ifndef PM1_SDK_PARTICLE_FILTER_T_HH
#define PM1_SDK_PARTICLE_FILTER_T_HH


#include <vector>
#include <random>
#include <fstream>

#include <eigen3/Eigen/Core>

#include <utilities/odometry_t.hpp>

namespace autolabor {
    /**
     * 粒子滤波器
     */
    struct particle_filter_t {
        using odomtery_delta_t = autolabor::odometry_t<autolabor::odometry_type::delta>;
        
        /**
         * 粒子群
         */
        std::vector<odometry_t<>> states;
        
        explicit particle_filter_t(size_t size);
        
        /**
         * 更新相对定位
         * @param delta 里程计增量
         * @return 综合位姿
         */
        [[nodiscard]] odometry_t<>
        operator()(const odometry_t<> &) const;
        
        /**
         * 更新联合定位
         * @param delta   里程计增量
         * @param measure 绝对测量值
         * @return 综合位姿
         */
        odometry_t<>
        update(const odometry_t<> &, const Eigen::Vector2d &);
    
    private:
        void initialize(const odometry_t<> &, const Eigen::Vector2d &);
        
        std::ofstream plot;
    
        Eigen::Vector2d measure_save;
        
        odometry_t<> match_save,
                     e_save,
                     e;
        
        std::random_device random;
        std::mt19937       engine;
        
        constexpr static auto
            update_step    = 0.02,
            accept_range   = 0.05,
            measure_weight = 0.25,
            d_range        = 0.1;
    };
}


#endif //PM1_SDK_PARTICLE_FILTER_T_HH
