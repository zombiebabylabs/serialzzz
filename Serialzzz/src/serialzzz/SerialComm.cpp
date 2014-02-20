/*
 * SerialComm.cpp
 *
 *  Created on: Feb 17, 2014
 *      Author: Timothy C. Fanelli
 *              Zombie Baby Labs
 *              tim@fanel.li
 */

#include "SerialComm.h"
#include "SerialCommListener.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <termios.h>
#include <strings.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <iostream>
#include <list>
#include <iterator>
#include <sstream>
using namespace std;

namespace zbl {
SerialComm::SerialComm( const string & portPath ) {
	char * temp = new char[ portPath.size()+1 ];
	strncpy( temp, portPath.c_str(), portPath.size() );

	this->port = temp;
	this->fd = -1;
	this->listeners = new std::list<SerialCommListener*>();
}

SerialComm::~SerialComm() {
	if ( this->fd != -1 ) {
		cerr << "Warning: closing " << port << " in destructor... please close explicitly!" << endl;
		closePort();
	}

	if ( this->port )
		delete port;

	if ( this->listeners )
		delete listeners;
}

/*
 * Opens the port - right now, this method is hard coded for 9600 bps, 8-bits, no parity, 1 stop bit.
 * I will expose open flags in a bit, as well as termios overrides.
 *
 * After the port is opened, asynchronous reading is also executing.
 */
bool SerialComm::openPort() {
	struct stat termstat;
	struct termios oldtio, newtio;

	this->fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (this->fd == -1) {
		cerr << "Could not open " << port << endl;
		this->fd = -1;
		return false;
	}

	int st = ::fstat(this->fd, &termstat);
	if (st == -1) {
		cerr << "Could not stat " << port << endl;
		this->fd = -1;
		return false;
	}


	::tcgetattr(this->fd, &oldtio); /* save current serial port settings */
	::bzero(&newtio, sizeof(newtio)); /* clear the new struct for settings */

	/*
	 BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	 CRTSCTS : output hardware flow control (only used if the cable has
	 all necessary lines. See sect. 7 of Serial-HOWTO)
	 CS8     : 8n1 (8bit,no parity,1 stopbit)
	 CLOCAL  : local connection, no modem contol
	 CREAD   : enable receiving characters
	 */
	newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;

	/*
	 IGNPAR  : ignore bytes with parity errors
	 ICRNL   : map CR to NL (otherwise a CR input on the other computer
	 will not terminate input)
	 otherwise make device raw (no other input processing)
	 */
	newtio.c_iflag = IGNPAR; // | ICRNL;

	/*
	 Raw output.
	 */
	newtio.c_oflag = 0;

	/*
	 ICANON  : enable canonical input
	 disable all echo functionality, and don't send signals to calling program
	 */
	newtio.c_lflag = ICANON;

	::tcflush(this->fd, TCIOFLUSH);
	::tcsetattr(this->fd, TCSANOW, &newtio);

	this->stopPolling.store(false);
	this->pollthread = new std::thread( &SerialComm::beginAsyncRead,this );
	return true;
}

bool SerialComm::closePort() {
	cout << "Closing" << endl;

	if ( this->fd != -1 )
	{
		this->stopPolling.store(true);
		this->pollthread->join();

		::close(this->fd);
		this->fd = -1;

		return true;
	}


	return false;
}

bool SerialComm::writeMessage( const std::string &message ) {
	this->lock.lock();
	::write( fd, message.c_str(), (size_t)message.size() );
	this->lock.unlock();
	return true;
}

void SerialComm::registerSerialCommListener( SerialCommListener *listener ) {
	this->lock.lock();
	this->listeners->push_back( listener );
	this->lock.unlock();
}

void SerialComm::beginAsyncRead()  {
	struct pollfd fds[1];
	fds[0].fd = this->fd;
	fds[0].events = POLLIN;

	bool interrupt;

	do  {
		// Lock to prevent writes while we're checking for incoming... write should also lock.
		this->lock.lock();

		// Poll the serial port -- blocks for 1000ms.
		int pollrc = ::poll( &fds[0], 1, 1000 );

		if ( pollrc < 0 )
			cerr << "Error polling " << port << endl;
		else if ( pollrc > 0 )
		{
			// If there is data available to read
			if ( fds[0].revents & POLLIN ) {
				// Parse the messages
				int count = 0;
				char ** msgs = parseMessage( this->fd, count );

				// Notify all registsered listeners of the message.
				for ( int i = 0; i < count; ++i ) {
					char * msg = msgs[i];

					list<SerialCommListener*>::iterator it = this->listeners->begin();
					while ( it != this->listeners->end() ) {
						SerialCommListener* listener = *it;
						listener->handleMessage( msg );
						++it;
					}
				}

				delete * msgs;
			}
		}

		this->lock.unlock();

		interrupt = this->stopPolling.load();
	} while ( ! interrupt );
}

/*
 * Default implementation of parse message reads all available data and
 * returns it as a single message.
 *
 * Messages returns are dynamically allocated, and should be deleted
 * after all listeners have been notified.
 */
char ** SerialComm::parseMessage( int fd, int &msgcount ) {
	std::stringstream sst;
	sst.str("");

	char buff[256]={'\0'};
	ssize_t bytes = read( fd, buff, sizeof(buff) );
	while ( bytes > 0 ) {
		sst << buff;
		bytes = read(fd,buff,sizeof(buff));
	}

	char ** msgs = (char**) new char*[1];
	msgs[0] = strdup(sst.str().c_str());
	msgcount = 1;
	return msgs;
}

}
