//
// Created by ydrml on 2019/3/12.
//

#ifndef PM1_SDK_ADJUST_RUDDER_H
#define PM1_SDK_ADJUST_RUDDER_H

#include <iostream>
#include <conio.h>
#include "../main/pm1_sdk.h"
#include "../main/internal/rudder_controller.hh"

namespace autolabor {
	namespace pm1 {
		int adjust_rudder(const std::string &port = "") {
			constexpr static auto degree = mechanical::encoder_rudder / 360;
			
			std::shared_ptr<rudder_controller> controller;
			
			if (port.empty())
				for (const auto &name : serial_ports())
					try {
						controller = std::make_shared<rudder_controller>(name);
						break;
					} catch (...) {
						controller = nullptr;
					}
			else
				try {
					controller = std::make_shared<rudder_controller>(port);
				} catch (...) {
					controller = nullptr;
				}
			
			if (!controller) {
				std::cerr << "open failed." << std::endl;
				return 1;
			}
			
			while (true) {
				if (_kbhit()) {
					auto c = _getch();
					switch (c) {
						case '+':
						case '=':
							controller->adjust(+degree);
							break;
						case '-':
						case '_':
							controller->adjust(-degree);
							break;
						case ' ':
							controller->done();
							break;
						case 27:
							return 0;
						default:
							break;
					}
				}
			}
		}
	}
}

#endif //PM1_SDK_ADJUST_RUDDER_H
