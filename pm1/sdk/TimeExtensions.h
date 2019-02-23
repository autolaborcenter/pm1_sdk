//
// Created by ydrml on 2019/2/23.
//

#ifndef PM1_SDK_TIMEEXTENSION_H
#define PM1_SDK_TIMEEXTENSION_H


#include <chrono>
#include <functional>

namespace mechdancer {
	namespace common {
		using unit = std::ratio<1, 1>;
		
		/**
		 * 转换为[duration]
		 *
		 * @param seconds 秒数
		 * @return 对应的 std::chrono::duration
		 */
		inline auto SecondsDuration(double seconds)
		-> std::chrono::duration<double, unit> {
			return std::chrono::duration<double, unit>(seconds);
		}
		
		/**
		 * 从高精度时钟获取当前时间
		 *
		 * @return 当前时间
		 */
		inline auto Now()
		-> decltype(std::chrono::high_resolution_clock::now()) {
			return std::chrono::high_resolution_clock::now();
		}
		
		/**
		 * 测量一段代码的执行时间
		 *
		 * @tparam TimeUnit 时间间隔的单位
		 * @param function 待测代码块
		 * @return 用[TimeUnit]表示的时间间隔
		 */
		template<class TimeUnit = std::chrono::duration<double, unit>>
		inline auto MeasureTime(const std::function<void()> &function)
		-> TimeUnit {
			const auto origin = Now();
			function();
			return Now() - origin;
		}
	}
}


#endif //PM1_SDK_TIMEEXTENSION_H
