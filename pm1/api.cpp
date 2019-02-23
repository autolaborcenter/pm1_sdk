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
				Delay(0.05);
			}
			
			// 阻塞并抑制输出
			std::chrono::duration<double, std::ratio<1>> inhibit() {
				auto temp = now();
				while (paused) {
					Drive(0, 0);
					loop_delay();
				}
				return now() - temp;
			}
			
			// 按固定控制量运行指定时间
			void go_timing(double v, double w, double seconds) {
				auto ending = now() + seconds_duration(seconds);
				while (now() < ending) {
					if (paused) ending += inhibit();
					else Drive(v, w);
					
					loop_delay();
				}
			}
		}
		
		bool Initialize(std::string port) {
			throw std::runtime_error("to do");
		}
		
		bool Initialize() {
			throw std::runtime_error("to do");
		}
		
		void Shutdown() {
			throw std::runtime_error("to do");
		}
		
		void GoStraight(double speed, double distance) {
			while (false) {
				if (paused) Drive(0, 0);
				else Drive(speed, speed);
				
				loop_delay();
			}
		}
		
		void GoStraightTiming(double speed, double time) {
			go_timing(speed, speed, time);
		}
		
		void GoArc(double speed, double r, double rad) {
			while (false) {
				if (paused) Drive(0, 0);
				else Drive(speed, speed / r);
				
				loop_delay();
			}
		}
		
		void GoArcTiming(double speed, double r, double time) {
			go_timing(speed, speed / r, time);
		}
		
		void TurnAround(double speed, double rad) {
			while (false) {
				if (paused) Drive(0, 0);
				else Drive(0, speed);
				
				loop_delay();
			}
		}
		
		void TurnAroundTiming(double speed, double time) {
			go_timing(0, speed, time);
		}
		
		void Pause() {
			Drive(0, 0);
			paused = true;
		}
		
		void Resume() {
			paused = false;
		}
		
		void Delay(double time) {
			std::this_thread::sleep_for(seconds_duration(time));
		}
		
		Odometry GetOdometry() {
			throw std::runtime_error("to do");
		}
		
		void Drive(double v, double w) {
			std::cout << "v = " << v << ", w = " << w << std::endl;
		}
	}
}
