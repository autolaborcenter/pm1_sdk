//
// Created by ydrml on 2019/2/23.
//

#ifndef PM1_SDK_TIME_EXTENSION_H
#define PM1_SDK_TIME_EXTENSION_H


#include <chrono>
#include <functional>

namespace autolabor {
    /** 时间单位：秒 */
    using seconds_floating = std::chrono::duration<double, std::ratio<1>>;
    
    /**
     * 转换为[duration]
     *
     * @param seconds 秒数
     * @return 对应的 std::chrono::duration
     */
    inline seconds_floating seconds_duration(double seconds) {
        return std::chrono::duration<double, std::ratio<1>>(seconds);
    }
    
    /**
     * 转换为[duration]
     *
     * @param seconds 秒数
     * @return 对应的 std::chrono::duration
     */
    template<class t, class t1, class t2>
    constexpr inline t duration_seconds(const std::chrono::duration<t1, t2> &duration) {
        return std::chrono::duration_cast<std::chrono::duration<t, std::ratio<1>>>(duration).count();
    }
    
    /**
     * 从高精度时钟获取当前时间
     *
     * @return 当前时间
     */
    inline auto now() -> decltype(std::chrono::high_resolution_clock::now()) {
        return std::chrono::high_resolution_clock::now();
    }
    
    /**
     * 测量一段代码的执行时间
     *
     * @tparam time_unit 时间间隔的单位
     * @param function 待测代码块
     * @return 用[TimeUnit]表示的时间间隔
     */
    template<class time_unit = seconds_floating>
    inline time_unit measure_time(const std::function<void()> &function) {
        const auto origin = now();
        function();
        return now() - origin;
    }
}

#endif //PM1_SDK_TIME_EXTENSION_H
