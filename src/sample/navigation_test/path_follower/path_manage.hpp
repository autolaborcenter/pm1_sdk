//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_PATH_MANAGE_HPP
#define PM1_SDK_PATH_MANAGE_HPP


#include "point_t.h"

#include <string>
#include <vector>
#include <fstream>

namespace path_follower {
    /**
     * 尖点检测
     *
     * @tparam iterator_t 迭代器类型
     * @param begin 迭代起点
     * @param end   迭代终点
     * @param order 阶数
     */
    template<class iterator_t>
    void check_tip(iterator_t begin, iterator_t end, uint8_t order) {
        if (order > 1)
            check_tip(begin, end, order - 1);
        
        for (auto item = begin + order; item < end - order; ++item) {
            if (item->tip_order < order)
                continue;
            auto dx0 = item->x - (item - order)->x,
                 dy0 = item->y - (item - order)->y,
                 dx1 = (item + order)->x - item->x,
                 dy1 = (item + order)->y - item->y;
            if (dx0 * dx1 + dy0 * dy1 < 0)
                item->tip_order = order;
        }
    }
    
    /**
     * 从文件加载路径
     *
     * @param filename 文件名
     * @return 路径
     */
    std::vector<point_t> load_path(const std::string &filename) {
        std::vector<point_t> path{};
        
        // 读取
        std::fstream file(filename, std::ios::in);
        for (size_t  i = 0;; ++i) {
            point_t temp{};
            if (!(file >> temp.x >> temp.y)) break;
            path.push_back(temp);
        }
        // 处理
        if (path.empty()) return {};
        path.shrink_to_fit();
        path.back().tip_order = 1;
        // 尖点检测
        check_tip(path.begin(), path.end(), 2);
        return path;
    }
}


#endif // PM1_SDK_PATH_MANAGE_HPP
