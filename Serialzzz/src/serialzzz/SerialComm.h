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
	/*
	 * Constructs a SerialComm object against the port specified in portPath, for
	 * instance: /dev/ttyO1
	 */
	SerialComm( const std::string &portPath );

	/*
	 * Destructor will invoke closePort() if the file handle is still open, but it
	 * is strongly recommended that you explicitly call closePort() when you are
	 * finished, as this also terminates the asynchronous read.
	 */
	virtual ~SerialComm();

	/*
	 * Opens the serial port, and spawns the asynchronous read thread.
	 */
	bool openPort();

	/*
	 * Interrupts and joins the asynchronous read thread, and closes the serial port.
	 */
	bool closePort();

	/*
	 * Writes a message to the serial port. This method is thread-safe with
	 * the asynchronous read, and will block if a message is currently being
	 * read.
	 */
	bool writeMessage( const std::string & message );

	/*
	 * Register a SerialCommListener instance to receive notification from this
	 * SerialComm's aysnchronous read thread when a new message is available.
	 *
	 * This message is invoked by the background thread directly.
	 */
	void registerSerialCommListener( SerialCommListener *listener );

protected:

	/*
	 * When the serial port has data avaialble, this method is called to parse
	 * a message from the given file descriptor. The default behavior is to
	 * read all the available data as one message; for situations where multiple
	 * messages are available, override this method.
	 *
	 * Returns an array of c-strings, where each c-string is a complete message.
	 */
	virtual char ** parseMessage( int fd, int &messagecount );


private:
	char * port;	// The name of the serial port file, e.g.: /dev/ttyS0
	int fd; 		// The file descriptor is set during open, so we may close it later.

	std::atomic_bool stopPolling;	// Used to interrupt the asynchronous read thread.
	std::thread *pollthread;		// The asynchronous read thread, created during openPort()
	std::mutex lock;				// Mutex used to synchronize the asynchronous read thread
								    //   with writeMessage() and registerSerialCommListener()

	std::list<SerialCommListener*> *listeners;	// All listeners to receive messages.

	void beginAsyncRead();
};
}


#endif /* SERIALCOMM_H_ */
