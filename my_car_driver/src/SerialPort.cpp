/*
 * SerialPort.cpp
 *
 *  Created on: Dec 14, 2015
 *      Author: liao
 */

#include <vector>
#include <boost/make_shared.hpp>
#include <myCarDriver/SerialPort.h>

using namespace std;
using namespace boost;


SerialPort::SerialPort()
{
	cout << "SerialPort Object created!" << endl;

	m_state = WAITING_FF;
	m_ready=false;
	m_timeOut = 5000;
	m_DataBytesRead = 10; // receive 10 byte in total
	m_tempBuf.resize(1024, 0);
}

SerialPort::~SerialPort()
{
	m_pios->stop();
	m_thread.join();
}

void SerialPort::start_a_read()
{
	cout << "SerialPort::start_a_read called !!" << endl;

	mutex::scoped_lock lock(m_serialMutex);

	m_pSerial->async_read_some(buffer(m_tempBuf), boost::bind(&SerialPort::readHandler,
			this,
			placeholders::error,
			placeholders::bytes_transferred
			));
}

void SerialPort::start_a_write()
{
	mutex::scoped_lock lock(m_serialMutex);

	async_write(*m_pSerial, buffer(*(m_writeQueue.front())),
			bind(&SerialPort::writeHandler, this, placeholders::error));
}

void SerialPort::mainRun()
{
	cout << "SerialPort mainThread STARTED!" << endl;

	m_state = WAITING_FF;

	start_a_read();

	m_pios->run();

	cout << "SerialPort mainThread EXITED!" << endl;
}

void SerialPort::readHandler(const system::error_code &ec, size_t bytesTransferred)
{
	if (ec)
	{
		cout << "SerialPort read error !!" << endl;
		return;
	}

	for (size_t i=0; i<bytesTransferred; i++)
	{
		uint8_t byte = m_tempBuf.at(i);
		printf("State=%d, Processing byte: 0x%X \n", m_state, byte);

		switch (m_state) {
		case WAITING_FF:
			if (byte == (uint8_t)0xFF)
			{
				 m_state = READING_DATA;

				m_ptimer.reset(new deadline_timer(*m_pios,
						posix_time::milliseconds(m_timeOut)));
				m_ptimer->async_wait(bind(
						&SerialPort::timeoutHandler,
						this,
						placeholders::error));
			}
			break;
		case READING_DATA:
			m_currentData.push_back(byte);
			if (m_currentData.size() == m_DataBytesRead)
				m_state = WAITING_FD;
			break;

		case WAITING_FD:
			if (byte == (uint8_t)0xFD)
			{
				m_ptimer->cancel();
				m_ptimer.reset();
				my_car_msgs::Uint8Array data;

				data.data = m_currentData;
				cout << "A new datagram received !!" << endl;

				outputData = data;
				m_currentData.clear();
				m_state = WAITING_FF;
				{
					unique_lock<mutex> lck(m_read_lock);
					m_ready = true;
				}
				cv.notify_all();
				break;
			}
		}
	}
	start_a_read();
}

void SerialPort::writeHandler(const system::error_code &ec)
{
	if (ec)
	{
		// TODO: 报错
	}

	{
		mutex::scoped_lock lock(m_writeQueueMutex);
		m_writeQueue.pop();
		if (m_writeQueue.empty()==false)
		{
			start_a_write();
		}
	}
}

void SerialPort::timeoutHandler(const system::error_code &ec)
{
	if (!ec)
	{
		cout << "Time Out !" << endl;
		m_state = WAITING_FF;
	}
}

void SerialPort::setSerialParams(const SerialParams &params)
{
	m_serialParams = params;
}

void SerialPort::setTimeOut(int timeout)
{
	m_timeOut = timeout;
}

bool SerialPort::startThread()
{
	cout << "SerialPort::startThread() called!" << endl;

	m_pios = make_shared<io_service>();

	try {
		m_pSerial = make_shared<serial_port>(ref(*m_pios), m_serialParams.serialPort);

		m_pSerial->set_option(
				serial_port::baud_rate(m_serialParams.baudRate));
		m_pSerial->set_option(
				serial_port::flow_control((serial_port::flow_control::type)m_serialParams.flowControl));
		m_pSerial->set_option(
				serial_port::parity((serial_port::parity::type)m_serialParams.parity));
		m_pSerial->set_option(
				serial_port::stop_bits((serial_port::stop_bits::type)m_serialParams.stopBits));
		m_pSerial->set_option(serial_port::character_size(8));
	}
	catch (std::exception &e) {
		cout << "Failed to open serial port !" << endl;
		cout << "Error Info: " << e.what() << endl;
		return false;
	}

	try {
		// 创建线程
		m_thread = boost::thread(boost::bind(&SerialPort::mainRun, this));
	}
	catch (std::exception &e) {
		cout << "Failed to create thread !" << endl;
		cout << "Error Info: " << e.what() << endl;
		return false;
	}
	return true;
}

bool SerialPort::stopThread()
{
	m_pios->stop();
	return true;
}

bool SerialPort::writeDataGram(const my_car_msgs::Uint8Array &datagram)
{
	return writeRaw(datagram.data);
}

bool SerialPort::writeRaw(const ByteVector &rawData)
{
	 mutex::scoped_lock lock(m_writeQueueMutex);

	bool writeIdle = m_writeQueue.empty();
	pByteVector data(new ByteVector(rawData));
	m_writeQueue.push(data);

	if (writeIdle) start_a_write();

	return true;
}



