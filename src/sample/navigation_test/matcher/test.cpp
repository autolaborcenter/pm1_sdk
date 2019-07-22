//
// Created by User on 2019/7/19.
//

#include <iostream>
#include <conio.h>
#include <eigen3/Eigen/Core>
#include "fusion_locator_t.hh"

int main() {
    autolabor::fusion_locator_t locator(50, 0.02);
    
    std::fstream file("save.txt", std::ios::in);
    for (size_t  i = 0;; ++i) {
        Eigen::Vector2d a, b;
        if (!(file >> a[0] >> a[1] >> b[0] >> b[1])) break;
        locator.push_back_pair(a, b);
        locator.refresh();
        //        _getch();
    }
    return 0;
}
