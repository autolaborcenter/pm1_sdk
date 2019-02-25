//
// Created by ydrml on 2019/2/22.
//

#include "api.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "extensions.h"
#include "time_extensions.h"

namespace autolabor {
	namespace pm1 {
		using namespace mechdancer::common;
		
		namespace {
			// 记录暂停状态
			volatile bool paused = false;
			
			// 循环的间隔
			inline void loop_delay() {
				delay(0.05);
			}
			
			// 阻塞并抑制输出
			std::chrono::duration<double, std::ratio<1>> inhibit() {
				const auto temp = now();
				while (paused) {
					drive(0, 0);
					loop_delay();
				}
				return now() - temp;
			}
			
			// 按固定控制量运行指定时间
			void go_timing(double v, double w, double seconds) {
				auto ending = now() + seconds_duration(seconds);
				while (now() < ending) {
					if (paused) ending += inhibit();
					else drive(v, w);
					
					loop_delay();
				}
			}
		}
		
		bool initialize(std::string port) {
			throw std::runtime_error("to do");
		}
		
		bool initialize() {
			throw std::runtime_error("to do");
		}
		
		void shutdown() {
			throw std::runtime_error("to do");
		}
		
		void go_straight(double speed, double distance) {
			while (false) {
				if (paused) drive(0, 0);
				else drive(speed, speed);
				
				loop_delay();
			}
		}
		
		void go_straight_timing(double speed, double time) {
			go_timing(speed, speed, time);
		}
		
		void go_arc(double speed, double r, double rad) {
			while (false) {
				if (paused) drive(0, 0);
				else drive(speed, speed / r);
				
				loop_delay();
			}
		}
		
		void go_arc_timing(double speed, double r, double time) {
			go_timing(speed, speed / r, time);
		}
		
		void turn_around(double speed, double rad) {
			while (false) {
				if (paused) drive(0, 0);
				else drive(0, speed);
				
				loop_delay();
			}
		}
		
		void turn_around_timing(double speed, double time) {
			go_timing(0, speed, time);
		}
		
		void pause() {
			drive(0, 0);
			paused = true;
		}
		
		void resume() {
			paused = false;
		}
		
		void delay(double time) {
			std::this_thread::sleep_for(seconds_duration(time));
		}
		
		Odometry get_odometry() {
			throw std::runtime_error("to do");
		}
		
		void drive(double v, double w) {
			std::cout << "v = " << v << ", w = " << w << std::endl;
		}
	}
}
