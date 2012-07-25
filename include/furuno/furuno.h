/*!
 * \file furuno/furuno.h
 * \author CHANGEME
 * \version 1.0
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 CHANGEME
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
 * This provides an interface for the Furuno GPS receivers
 *
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 *
 */

#ifndef FURUNO_H
#define FURUNO_H

#include <string>
#include <cstring> // for size_t
#include <fstream>

// Structure definition headers
#include "furuno_enums.h"
#include "furuno_structures.h"
// Boost Headers
#include <boost\function.hpp>
#include <boost\thread.hpp>
#include <boost\bind.hpp>
// Serial Headers
#include "serial\serial.h"

namespace furuno {

// Messaging callbacks
typedef boost::function<void(const std::string&)> LogMsgCallback;

// GPS Callbacks
typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void(PositionOutput&, double&)> PositionCallback;
typedef boost::function<void(NavMessage&, GpsEphemeris&, double&)> NavMessageCallback;
//typedef boost::function<void(Velocity&, double&)> BestVelocityCallback;

class Furuno
{
public:
	Furuno();
	virtual ~Furuno();

	/*!
	 * Connects to the Furuno receiver given a serial port.
	 *
	 * @param port Defines which serial port to connect to in serial mode.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 *
	 * @throws ConnectionFailedException connection attempt failed.
	 * @throws UnknownErrorCodeException unknown error code returned.
	 */
	bool Connect(std::string port, int baudrate=115200);

    /*!
     * Disconnects from the serial port
     */
    void Disconnect();

	//////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    LogMsgCallback on_log_debug;
    LogMsgCallback on_log_info;
    LogMsgCallback on_log_warning;
    LogMsgCallback on_log_error;

	/*! Function pointer to callback function for timestamping */
	GetTimeCallback on_get_time;

	/*! Function pointer to callbacks from the gps parser */
    PositionCallback on_position;
	NavMessageCallback on_nav_message;

private:

	/*!
	 * Starts a thread to continuously read from the serial port.
	 *
	 * Starts a thread that runs 'ReadSerialPort' which constatly reads
	 * from the serial port.  When valid data is received, parse and then
	 *  the data callback functions are called.
	 */
	void StartReading();

	/*!
	 * Starts the thread that reads from the serial port
	 */
	void StopReading();

	/*!
	 * Method run in a seperate thread that continuously reads from the
	 * serial port.  When a complete packet is received, the parse
	 * method is called to process the data
	 */
	void ReadSerialPort();


	void BufferIncomingData(unsigned char *message, unsigned int length);

	/*!
	 * Parses a packet of data from the GPS.  The
	 */
	void ParseBinary(unsigned char *message, unsigned char message_id);
	void ParsePositionMessage(Position &position);
	void ParseNavMessage(NavMessage &nav_message);

	GpsEphemeris furuno_ephemeris_;
	PositionOutput furuno_position_data_;
	

	//bool ParseVersion(std::string packet);

    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from Furuno
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.

	//////////////////////////////////////////////////////
	// Incoming data buffers
	//////////////////////////////////////////////////////
	unsigned char data_buffer_[MAX_NOUT_SIZE];	//!< data currently being buffered to read
	unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
	size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
	size_t buffer_index_;		//!< index into data_buffer_
	size_t header_length_;	//!< length of the current header being read
	bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	double parse_timestamp_;		//!< time stamp when last parse began
};
}// furuno namespace
#endif
