/*
 * SerialCommListener.h
 *
 *  Created on: Feb 19, 2014
 *      Author: Timothy C. Fanelli
 *              Zombie Baby Labs
 */

#ifndef SERIALCOMMLISTENER_H_
#define SERIALCOMMLISTENER_H_

class SerialCommListener {
public:
	virtual ~SerialCommListener() {}

	/**
	 * A null-terminated message recieved from the UART.
	 */
	virtual void handleMessage( const char * const message ) = 0;
};


#endif /* SERIALCOMMLISTENER_H_ */
