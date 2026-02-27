#ifndef CAN_UTILS_SEND_HPP
#define CAN_UTILS_SEND_HPP

#include <map>
#include <vector>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <net/if.h>

#include "logger.hpp"

#define CAN_SEND "can_utils_send"

namespace net::wavem::can
{
	/**
		@class   Send
		@date    2025-01-15
		@author  reidlo(naru5135@wavem.net)
		@version 0.1.0
	*/
	class Send final
	{
	private:
		std::map<std::string, int> sock_map_;
		int socket_open(char *device);

	public:
		explicit Send();
		virtual ~Send();
		void socket_close();
		int send(std::vector<unsigned char> body, unsigned int msg_id, char *device);
		int socket_open(std::vector<std::string> device);
		bool is_connected(char *device);

	public:
		using SharedPtr = std::shared_ptr<Send>;
	};
}

#endif