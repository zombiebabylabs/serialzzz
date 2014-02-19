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
	virtual void notify( const char * const message, int length ) = 0;
};


#endif /* SERIALCOMMLISTENER_H_ */
