/*
 * SerialPort.h
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <inttypes.h>
#include <vector>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <my_car_msgs/Uint8Array.h>

using namespace std;
using namespace boost;
using namespace boost::asio;


// SerialPort communication parameter
class SerialParams {
public:
	string serialPort; 				//device file
	unsigned int baudRate; 				// baud rate
	unsigned int flowControl; 			// flow control
	unsigned int parity; 				// parity
	unsigned int stopBits;
	SerialParams() :
			serialPort(), baudRate(115200), flowControl(0), parity(0), stopBits(0)
	{
	}
	SerialParams(
			string _serialPort,
			unsigned int _baudRate,
			unsigned int _flowControl,
			unsigned int _parity,
			unsigned int _stopBits
			) :
			serialPort(_serialPort),
			baudRate(_baudRate),
			flowControl(_flowControl),
			parity(_parity),
			stopBits(_stopBits)
	{
	}
};

typedef vector<uint8_t> ByteVector;
typedef shared_ptr<ByteVector> pByteVector;

class SerialPort {
private:
	shared_ptr<deadline_timer>	m_ptimer;
	shared_ptr<io_service> 	m_pios;
	shared_ptr<serial_port>	m_pSerial;
	mutex 					m_serialMutex;

	enum {HEADER_LEN = 4};

	enum STATE {
		WAITING_FF, READING_DATA, WAITING_FD
	} m_state;

	SerialParams	m_serialParams;
	int				m_timeOut;

	ByteVector		m_tempBuf;

	ByteVector 		m_currentData;
	size_t 			m_DataBytesRead;

	queue<pByteVector>	m_writeQueue;
	mutex									m_writeQueueMutex;

	boost::thread m_thread;

	void mainRun();

	void start_a_read();
	void start_a_write();


	void readHandler(const system::error_code &ec, size_t bytesTransferred);

	void writeHandler(const system::error_code &ec);

	void timeoutHandler(const system::error_code &ec);

public:
	SerialPort();
	virtual ~SerialPort();

	void setSerialParams(const SerialParams &params);

	void setTimeOut(int timeout);

	bool startThread();
	bool stopThread();

	bool            m_ready;
	condition_variable   cv;
	mutex           m_read_lock;
	my_car_msgs::Uint8Array outputData;

	bool writeDataGram(const my_car_msgs::Uint8Array &datagram);

	bool writeRaw(const ByteVector &rawData);
};




#endif /* SERIALPORT_H_ */
