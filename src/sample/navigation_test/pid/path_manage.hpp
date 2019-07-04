//
// Created by User on 2019/7/4.
//

#ifndef PM1_SDK_PATH_MANAGE_HPP
#define PM1_SDK_PATH_MANAGE_HPP


#include "point_t.h"

#include <string>
#include <vector>
#include <fstream>

std::vector<point_t> load_path(const std::string &filename) {
    std::vector<point_t> path{};
    std::fstream         file(filename, std::ios::in);
    for (size_t          i = 0;; ++i) {
        point_t temp{};
        if (!(file >> temp.x >> temp.y)) break;
        path.push_back(temp);
    }
    return path;
}


#endif // PM1_SDK_PATH_MANAGE_HPP
