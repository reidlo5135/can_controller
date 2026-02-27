#include "dump.hpp"

namespace net::wavem::can
{
	static volatile int running = 1;

	Dump::Dump()
		: sock_info_{}, fd_epoll_(0), curr_max_(0), prog_name_{}, dev_name_{}, d_index_{}, max_dev_name_len_(0)
	{
	}

	Dump::~Dump() = default;

	/**
	 * @brief Channel open for receiving CAN data
	 * @details
	 * @param argc number of channels
	 * @param arg_val Channel to open and filtering message information
	 * @param p_class_type Class for calling callback functions
	 * @param func Callback function when receiving CAN data
	 * @return  Result of processing, 0 if successful
	 * @warning
	 * @exception
	 */
	int
	Dump::socket_open(
		int argc,
		std::vector<std::string> arg_val,
		Adaptor *p_class_type,
		void (Adaptor::*func)(unsigned char *data, int can_id))
	{
		std::string argv[MAXCN];
		int idx = 0;

		unsigned char view = 0;

		int rcv_buff_size = 0;
		int num_events;
		int num_filter;
		char *ptr, *n_ptr;
		struct sockaddr_can addr{};
		char ctrl_msg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(3 * sizeof(struct timespec)) + CMSG_SPACE(sizeof(__u32))];
		struct iovec iov{};
		struct msghdr msg{};
		struct can_filter *r_filter;
		can_err_mask_t err_mask;
		struct canfd_frame frame{};
		int n_bytes, sock_count, max_d_len;
		struct ifreq ifr{};
		struct timeval tv{};
		int timeout_ms = -1;
		bool retry = true;
		this->curr_max_ = argc;

		if (MAXCN < argc)
		{
			RCUTILS_INFO(CAN_DUMP, "More than %d CAN devices given on commandline! (%d)", MAXCN, argc);
			return -1;
		}

		for (const std::string &arg : arg_val)
		{
			argv[idx++] = arg;
		}

		struct epoll_event events_pending[MAXSOCK]{};
		struct epoll_event event_setup = {.events = EPOLLIN};

		if (this->curr_max_ > MAXSOCK)
		{
			RCUTILS_ERROR(CAN_DUMP, "More than %d CAN devices given on commandline!", MAXSOCK);
			return 1;
		}

		while (retry)
		{
			try
			{
				this->fd_epoll_ = epoll_create(1);

				if (this->fd_epoll_ < 0)
				{
					return 1;
				}

				for (sock_count = 0; sock_count < this->curr_max_; sock_count++)
				{
					struct if_info *obj = &this->sock_info_[sock_count];
					RCUTILS_INFO(CAN_DUMP, "call open !!! [%s]", (char *)argv[sock_count].c_str());
					ptr = (char *)argv[sock_count].c_str();
					n_ptr = strchr(ptr, ',');
					obj->s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

					if (obj->s < 0)
					{
						throw DEVICE_EXCEPTION;
					}

					event_setup.data.ptr = obj;
					if (epoll_ctl(this->fd_epoll_, EPOLL_CTL_ADD, obj->s, &event_setup))
					{
						throw DEVICE_EXCEPTION;
					}
					obj->cmdlinename = ptr;

					if (n_ptr)
					{
						n_bytes = n_ptr - ptr;
					}
					else
					{
						n_bytes = static_cast<int>(strlen(ptr));
					}

					if (n_bytes >= IFNAMSIZ)
					{
						RCUTILS_ERROR(CAN_DUMP, "name of CAN device '%s' is too long!", ptr);
						return 1;
					}

					if (n_bytes > this->max_dev_name_len_)
					{
						this->max_dev_name_len_ = n_bytes;
					}

					addr.can_family = AF_CAN;
					memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
					strncpy(ifr.ifr_name, ptr, n_bytes);
					RCUTILS_INFO(CAN_DUMP, "using interface name [%s]", ifr.ifr_name);

					if (strcmp(ANYDEV, ifr.ifr_name) != 0)
					{
						if (ioctl(obj->s, SIOCGIFINDEX, &ifr) < 0)
						{
							perror("SIOCGIFINDEX");
							throw DEVICE_EXCEPTION;
						}
						addr.can_ifindex = ifr.ifr_ifindex;
					}
					else
					{
						addr.can_ifindex = 0;
					}

					if (n_ptr)
					{
						num_filter = 0;
						ptr = n_ptr;
						while (ptr)
						{
							num_filter++;
							ptr++;
							ptr = strchr(ptr, ',');
						}

						r_filter = (struct can_filter *)malloc(sizeof(struct can_filter) * num_filter);

						if (!r_filter)
						{
							RCUTILS_ERROR(CAN_DUMP, "Failed to create filter space!");
							return 1;
						}

						num_filter = 0;
						err_mask = 0;

						while (n_ptr)
						{
							ptr = n_ptr + 1;
							RCUTILS_INFO(CAN_DUMP, "n_ptr : %s", n_ptr);
							RCUTILS_INFO(CAN_DUMP, "ptr : %s", ptr);
							n_ptr = strchr(ptr, ',');

							if (sscanf(ptr, "%d:%x", &r_filter[num_filter].can_id, &r_filter[num_filter].can_mask) == 2)
							{
								r_filter[num_filter].can_mask &= ~CAN_ERR_FLAG;

								if (*(ptr + 8) == ':')
								{
									r_filter[num_filter].can_id |= CAN_EFF_FLAG;
								}

								RCUTILS_INFO(CAN_DUMP, "filter : '0x%02x' , '0x%02x' %d\n", r_filter[num_filter].can_id, r_filter[num_filter].can_mask, __LINE__);
								num_filter++;
							}
							else if (sscanf(ptr, "#%x", &err_mask) != 1)
							{
								RCUTILS_ERROR(CAN_DUMP, "Error in filter option parsing: '%s'\n", ptr);
								free(r_filter);
								return 1;
							}
						}

						if (err_mask)
						{
							setsockopt(obj->s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
						}

						if (num_filter)
						{
							setsockopt(obj->s, SOL_CAN_RAW, CAN_RAW_FILTER, r_filter, num_filter * sizeof(struct can_filter));
						}
						free(r_filter);
					}

					if (rcv_buff_size)
					{
						int curr_rcv_buff_size;
						socklen_t curr_rcv_buff_size_len = sizeof(curr_rcv_buff_size);

						if (setsockopt(obj->s, SOL_SOCKET, SO_RCVBUFFORCE, &rcv_buff_size, sizeof(rcv_buff_size)) < 0)
						{
							RCUTILS_INFO(CAN_DUMP, "SO_RCV_BUFF_FORCE failed so try SO_RCVBUF ...\n");

							if (setsockopt(obj->s, SOL_SOCKET, SO_RCVBUF, &rcv_buff_size, sizeof(rcv_buff_size)) < 0)
							{
								perror("setsockopt SO_RCVBUF");
								return 1;
							}

							if (getsockopt(obj->s, SOL_SOCKET, SO_RCVBUF, &curr_rcv_buff_size, &curr_rcv_buff_size_len) < 0)
							{
								perror("getsockopt SO_RCVBUF");
								return 1;
							}

							if (!sock_count && curr_rcv_buff_size < rcv_buff_size * 2)
							{
								RCUTILS_ERROR(CAN_DUMP, "The socket receive buffer size was adjusted due to /proc/sys/net/core/rmem_max.\n");
							}
						}
					}

					if (bind(obj->s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
					{
						perror("bind");
						return 1;
					}
				}

				iov.iov_base = &frame;
				msg.msg_name = &addr;
				msg.msg_iov = &iov;
				msg.msg_iovlen = 1;
				msg.msg_control = &ctrl_msg;

				while (running)
				{
					num_events = epoll_wait(this->fd_epoll_, events_pending, curr_max_, timeout_ms);

					if (num_events == -1)
					{
						if (errno != EINTR)
						{
							running = 0;
						}
						continue;
					}

					if (!num_events && timeout_ms >= 0)
					{
						running = 0;
						continue;
					}

					for (int i = 0; i < num_events; i++)
					{
						struct if_info *obj = (struct if_info *)events_pending[i].data.ptr;

						if (ioctl(obj->s, SIOCGIFNAME, &ifr) < 0)
						{
							perror("SIOCGIFNAME");
							throw DEVICE_EXCEPTION;
						}

						iov.iov_len = sizeof(frame);
						msg.msg_namelen = sizeof(addr);
						msg.msg_controllen = sizeof(ctrl_msg);
						msg.msg_flags = 0;

						n_bytes = recvmsg(obj->s, &msg, 0);

						if (static_cast<size_t>(n_bytes) == CAN_MTU)
						{
							max_d_len = CAN_MAX_DLEN;
						}
						else if (static_cast<size_t>(n_bytes) == CANFD_MTU)
						{
							max_d_len = CANFD_MAX_DLEN;
						}
						else
						{
							RCUTILS_ERROR(CAN_DUMP, "read: incomplete CAN frame %ld\n", static_cast<size_t>(n_bytes));
							sleep(3);
							num_events = 0;
							throw DEVICE_EXCEPTION;
						}

						if (frame.can_id & CAN_EFF_FLAG)
						{
							view |= CANLIB_VIEW_INDENT_SFF;
						}

						fprint_long_canframe(stdout, &frame, NULL, view, max_d_len);
						printf("\n");

						fflush(stdout);
						std::function<void(unsigned char *, int)> handler = std::move(std::bind(func, p_class_type, std::placeholders::_1, std::placeholders::_2));

						int s_can_id;
						if (frame.can_id & CAN_EFF_FLAG)
						{
							s_can_id = frame.can_id & CAN_EFF_MASK;
						}
						else
						{
							s_can_id = frame.can_id;
						}
						handler(frame.data, s_can_id);
					}
				}
			}
			catch (int e)
			{
				if (e == DEVICE_EXCEPTION)
				{
					this->sock_close();
					sleep(2);
					retry = true;
					continue;
				}
			}
		}

		RCUTILS_INFO(CAN_DUMP, "end while in can_dump\n");
		sock_close();
		RCUTILS_INFO(CAN_DUMP, "can receive end\n");
		return 0;
	}

	void
	Dump::sock_close()
	{
		for (int i = 0; i < this->curr_max_; i++)
		{
			close(this->sock_info_[i].s);
			memset(&this->sock_info_[i], 0x00, sizeof(if_info));
		}
		close(this->fd_epoll_);
		RCUTILS_INFO(CAN_DUMP, "close socket\n");
	}
}
