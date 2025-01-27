/*
 * ros_rodos.h
 *
 *  Created on: 01.12.2020
 *      Author: Michael Strohmeier
 */

#pragma once

#include "rodos.h"

#ifdef MAX_SUBSCRIBERS
#undef MAX_SUBSCRIBERS
#endif

#define RODOS_INPUT_SIZE				 UART_BUF_SIZE
#define RODOS_OUTPUT_SIZE				 UART_BUF_SIZE
#define RODOS_MAX_SUBSCRIBERS			 25
#define RODOS_MAX_PUBLISHERS			 25

class ROS_RODOS_CLIENT_UART{
  public:
	ROS_RODOS_CLIENT_UART(){
	  this->io_ptr = NULL;
      this->read_index = 0;
      this->msg_len = 0;
    }

    void init(char* io_ptr){
    	this->io_ptr = (RODOS::HAL_UART*) io_ptr;
    }

    int read(){
    	if(io_ptr->isDataReady()){
    		read_index = 0;
    		msg_len = io_ptr->read((char*) message_in,RODOS_INPUT_SIZE);
    	}
    	if(read_index < msg_len)
    		return (uint8_t) message_in[read_index++];
    	return -1;
    };

	void write(uint8_t* data, int length){
		if( (size_t) length > UART_BUF_SIZE){
			guard.enter();
			size_t sent = 0;
			do {
				sent += io_ptr->write((char*) data+sent, UART_BUF_SIZE-sent);
			} while(sent < UART_BUF_SIZE);
			write(data+UART_BUF_SIZE, length - (int) UART_BUF_SIZE);
			guard.leave();
		} else {
			guard.enter();
			size_t sent = 0;
			do {
				sent += io_ptr->write((char*) data+sent, (size_t) length-sent);
			} while(sent < (size_t) length);
			guard.leave();
		}
	};

    unsigned long time(){return (unsigned long) (NOW() / MILLISECONDS);}

  protected:
    size_t read_index, msg_len;
    Semaphore guard;
    uint8_t message_in[RODOS_INPUT_SIZE];
    RODOS::HAL_UART *io_ptr;
};

