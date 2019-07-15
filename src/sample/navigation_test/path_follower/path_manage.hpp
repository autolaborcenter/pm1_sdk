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
        
        if (path.empty())
            return {};
        
        path.shrink_to_fit();
        path.back().type = point_type_t::tip;
        
        if (path.size() < 3)
            return path;
        
        // 识别尖点
        for (auto item = path.begin() + 1;
             item < path.end() - 1;
             ++item) {
            auto x0 = item->x - (item - 1)->x,
                 y0 = item->y - (item - 1)->y,
                 x1 = (item + 1)->x - item->x,
                 y1 = (item + 1)->y - item->y;
            if (x0 * x1 + y0 * y1 < 0)
                item->type = point_type_t::tip;
        }
        return path;
    }
}


#endif // PM1_SDK_PATH_MANAGE_HPP
