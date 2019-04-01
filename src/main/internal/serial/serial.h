/*!
 * \file serial/serial.h
 * \author  William Woodall <wjwwood@gmail.com>
 * \author  John Harrison   <ash.gti@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 William Woodall
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for interacting with Serial Ports.
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <vector>
#include <string>

namespace serial {
	/*!
	 * Structure that describes a serial device.
	 */
	struct PortInfo {
		
		/*! Address of the serial port (this can be passed to the constructor of Serial). */
		std::string port;
		
		/*! Human readable description of serial device if available. */
		std::string description;
		
		/*! Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available. */
		std::string hardware_id;
		
	};
	
	/* Lists the serial ports available on the system
	 *
	 * Returns a vector of available serial ports, each represented
	 * by a serial::PortInfo data structure:
	 *
	 * \return vector of serial::PortInfo.
	 */
	std::vector<PortInfo> list_ports();
	
} // namespace serial

#endif
