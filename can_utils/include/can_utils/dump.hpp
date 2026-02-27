#ifndef CAN_UTILS_DUMP_HPP
#define CAN_UTILS_DUMP_HPP

#include <iostream>
#include <vector>
#include <ctype.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <memory>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>

#include "lib.hpp"
#include "adaptor.hpp"
#include "logger.hpp"

#define CAN_DUMP "can_utils_dump"
#define TIMESTAMPSZ 50 /* string 'absolute with date' requires max 49 bytes */
#define MAXCN 2		   /* max. number of CAN channel */
#define MAXSOCK 16	   /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30  /* size of receive name index to omit ioctls */
#define ANYDEV "any"   /* name of interface to receive from any CAN interface */
#define ANL "\r\n"	   /* newline in ASC mode */

#define MAXANI 4

#define DEVICE_EXCEPTION (-1)

namespace net::wavem::can
{
	class Adaptor;

	/**
		@class   if_info
		@date    2023-02-14
		@author  ys.kwon(ys.kwon@wavem.net)
		@date 2025-01-15
		@author reidlo(naru5135@wavem.net)
		@brief   collection group information per open socket
		@version 0.0.1
		@warning
	*/
	struct if_info
	{
		int s;
		char *cmdlinename;
		__u32 dropcnt;
		__u32 last_dropcnt;
	};

	/**
		@class   Dump
		@date    2023-02-14
		@author  ys.kwon(ys.kwon@wavem.net)
		@date 2025-01-15
		@author reidlo(naru5135@wavem.net)
		@brief   Can network data listening class
		@version 0.0.1
		@warning
	*/
	class Dump final
	{
	private:
		struct if_info sock_info_[MAXSOCK];
		int fd_epoll_;
		int curr_max_;
		char *prog_name_;
		char dev_name_[MAXIFNAMES][IFNAMSIZ + 1];
		int d_index_[MAXIFNAMES];
		int max_dev_name_len_;

	public:
		explicit Dump();
		virtual ~Dump();
		int socket_open(
			int argc,
			std::vector<std::string> arg_val,
			Adaptor *p_class_type,
			void (Adaptor::*func)(unsigned char *data, int can_id));
		void sock_close();

	public:
		using SharedPtr = std::shared_ptr<Dump>;
	};
}
#endif