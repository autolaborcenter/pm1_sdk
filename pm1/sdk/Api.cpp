//
// Created by ydrml on 2019/2/22.
//

#include "Api.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "Extensions.h"
#include "TimeExtensions.h"

namespace pm1 {
	namespace sdk {
		using namespace mechdancer::common;
		
		namespace {
			// 循环的间隔
			constexpr auto LOOP_DELAY = 0.05;
			
			// 记录暂停状态
			volatile bool paused = false;
			
			// 阻塞并抑制输出
			inline std::chrono::duration<double, std::ratio<1>> Inhibit() {
				auto temp = Now();
				while (paused) {
					Drive(0, 0);
					Delay(LOOP_DELAY);
				}
				return Now() - temp;
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
			while (true) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(LOOP_DELAY);
			}
		}
		
		void GoStraightTiming(double speed, double time) {
			auto ending = Now() + SecondsDuration(time);
			while (Now() < ending) {
				if (paused) ending += Inhibit();
				else throw std::runtime_error("to do");
				
				Delay(LOOP_DELAY);
			}
		}
		
		void GoArc(double speed, double r, double rad) {
			while (true) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(LOOP_DELAY);
			}
		}
		
		void GoArcTiming(double speed, double r, double time) {
			auto ending = Now() + SecondsDuration(time);
			while (Now() < ending) {
				if (paused) ending += Inhibit();
				else throw std::runtime_error("to do");
				
				Delay(LOOP_DELAY);
			}
		}
		
		void TurnAround(double speed, double rad) {
			while (true) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(LOOP_DELAY);
			}
		}
		
		void TurnAroundTiming(double speed, double time) {
			auto ending = Now() + SecondsDuration(time);
			while (Now() < ending) {
				if (paused) ending += Inhibit();
				else throw std::runtime_error("to do");
				
				Delay(LOOP_DELAY);
			}
		}
		
		void Pause() {
			Drive(0, 0);
			paused = true;
		}
		
		void Resume() {
			paused = false;
		}
		
		void Delay(double time) {
			std::this_thread::sleep_for(std::chrono::nanoseconds((long long) (time * 1E9)));
		}
		
		Odometry GetOdometry() {
			throw std::runtime_error("to do");
		}
		
		void Drive(double v, double w) {
			std::cout << "v = " << v << ", w = " << w << std::endl;
		}
	}
}
