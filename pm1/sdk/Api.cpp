//
// Created by ydrml on 2019/2/22.
//

#include "Api.h"

#include <string>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>

namespace pm1 {
	namespace sdk {
		// 记录暂停状态
		volatile bool paused = false;
		
		// 获取当前时间
		inline auto now() -> decltype(std::chrono::high_resolution_clock::now()) {
			return std::chrono::high_resolution_clock::now();
		}
		
		inline std::chrono::duration<double, std::ratio<1, 1>> timePass(decltype(now()) origin) {
			return now() - origin;
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
				
				Delay(0.01);
			}
		}
		
		void GoStraightTiming(double speed, double time) {
			auto origin = now();
			while (timePass(origin).count() < time) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(0.01);
			}
		}
		
		void GoArc(double speed, double r, double rad) {
			while (true) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(0.01);
			}
		}
		
		void GoArcTiming(double speed, double r, double time) {
			auto origin = now();
			while (timePass(origin).count() < time) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(0.01);
			}
		}
		
		void TurnAround(double speed, double rad) {
			while (true) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(0.01);
			}
		}
		
		void TurnAroundTiming(double speed, double time) {
			auto origin = now();
			while (timePass(origin).count() < time) {
				if (paused) Drive(0, 0);
				else throw std::runtime_error("to do");
				
				Delay(0.01);
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
