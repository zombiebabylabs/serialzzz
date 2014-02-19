/*
 * SerialComm.h
 *
 *  Created on: Feb 17, 2014
 *      Author: Timothy C. Fanelli
 *              Zombie Baby Labs
 *
 *  Describes the base class for a serial communication ("SerialComm") object.
 */

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_1
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_2
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_4
#define __GCC_HAVE_SYNC_COMPARE_AND_SWAP_8

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <list>

#include "SerialCommListener.h"

namespace zbl {

/*
 * SerialComm provides asynchronous serial I/O to a UART port.
 *
 * After the SerialComm is opened, it begins asynchronous reads, and notifies registered
 * instances of SerialCommListener objects when a message is available.
 */
class SerialComm {
public:
	SerialComm( const std::string &portPath );
	virtual ~SerialComm();

	bool openPort();
	bool closePort();

	/*
	 * Writes a message to the serial port.
	 */
	bool writeMessage( const std::string & message );

	/*
	 * When the serial port has data avaialble, this method is called to parse
	 * a message from the given file descriptor. The default behavior is to
	 * read all the available data as one message; for situations where multiple
	 * messages are available, override this method.
	 *
	 * Returns an array of c-strings, where each c-string is a complete message.
	 */
	virtual char ** parseMessage( int fd, int &messagecount );

	void registerSerialCommListener( SerialCommListener *listener );
private:
	char * port;
	int fd; // The file descriptor is set during open, so we may close it later.

	std::atomic_bool stopPolling;
	std::thread *pollthread;
	std::mutex lock;

	std::list<SerialCommListener*> *listeners;

	void beginAsyncRead();
};
}


#endif /* SERIALCOMM_H_ */
