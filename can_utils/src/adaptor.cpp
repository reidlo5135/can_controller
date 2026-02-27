#include "adaptor.hpp"

namespace net::wavem::can
{
	Adaptor::Adaptor()
		: is_big_endian_(0), register_check_(false), can_dump_(nullptr), can_send_(nullptr)
	{
	}

	Adaptor::~Adaptor()
	{
		this->release();
	}

	/**
	 * @brief initialization task
	 * @details
	 * @param endian system's endian type
	 * @return result of processing, 0 if successful
	 * @exception
	 */
	int
	Adaptor::initialize(const bool &endian)
	{
		this->is_big_endian_ = endian;

		this->can_dump_ = std::make_shared<Dump>();
		this->can_send_ = std::make_shared<Send>();

		if (this->can_dump_ != nullptr && this->can_send_ != nullptr)
		{
			RCUTILS_INFO(CAN_ADAPTOR, "initialized");
		}
		else
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "initialize failed");
		}

		return 0;
	}

	/**
	 * @brief release task
	 * @details
	 * @param
	 * @return void
	 * @exception
	 */
	void
	Adaptor::release() const
	{
		if (this->can_send_ != nullptr)
		{
			this->can_send_->socket_close();
		}
	}

	/**
	 * @brief Open a can channel.
	 * @details
	 * @param
	 * @return  Result of processing, 0 if successful
	 * @warning After registering all callback functions, call them.
	 * @exception
	 */
	int
	Adaptor::open(std::vector<std::string> device)
	{
		RCUTILS_INFO(CAN_ADAPTOR, "device : [%s]", device[0].c_str());

		if (this->can_socket_open(device) == 0 && this->can_reception_open() == 0)
		{
			return 0;
		}

		return -1;
	}

	/**
	 * @brief Open a can channel for send.
	 * @details
	 * @param device Channel name to open
	 * @return  Result of processing, 0 if successful
	 * @warning After registering all callback functions, call them.
	 * @exception
	 */
	int
	Adaptor::can_socket_open(std::vector<std::string> device)
	{
		RCUTILS_INFO(CAN_ADAPTOR, "CanSocketOpen, send socket open");

		if (this->can_send_ == nullptr)
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "CanSocketOpen, invalid can send object");
			return -1;
		}

		if (this->can_send_->socket_open(device) < 0)
		{
			return -1;
		}

		return 0;
	}

	/**
	 * @brief Open a can channel for reception.
	 * @details
	 * @return  Result of processing, 0 if successful
	 * @warning After registering all callback functions, call them.
	 * @exception
	 */
	int
	Adaptor::can_reception_open()
	{
		std::map<std::string, std::string> parameter_map;

		RCUTILS_INFO(CAN_ADAPTOR, "receive channel open");
		RCUTILS_INFO(CAN_ADAPTOR, "Number of messages waiting to be received : [%d]", this->funcs_map_.size());

		if (this->funcs_map_.size() < 0)
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "not found messages waiting to be received");
			return -1;
		}

		for (const std::pair<const int, std::shared_ptr<CanCallback>> &iter : this->funcs_map_)
		{
			CanCallback *obj = (CanCallback *)iter.second.get();
			std::string ch = obj->get_channel();
			RCUTILS_INFO(CAN_ADAPTOR, "<registered message> channel : (%s), can_id : (%d)", obj->get_channel().c_str(), obj->get_can_id());

			if (parameter_map.count(ch))
			{
				const std::map<std::string, std::string>::iterator &val = parameter_map.find(ch);
				std::string param = val->second;
				RCUTILS_INFO(CAN_ADAPTOR, "map ch : %s, param : %s", ch.c_str(), param.c_str());

				if (obj->get_can_id() == CAN_MCU_STATUS_2_ID || obj->get_can_id() == CAN_BMS_ID || obj->get_can_id() == CAN_MCU_GENERAL_STATUS_ID)
				{
					RCUTILS_INFO(CAN_ADAPTOR, "extended id : %d", obj->get_can_id());
					param.append(ch).append(",").append(std::to_string(obj->get_can_id())).append(":").append(CAN_EFF);
				}
				else
				{
					param.append(",").append(std::to_string(obj->get_can_id())).append(":").append(ONLY_SFF);
				}
				val->second = param;
			}
			else
			{
				std::string param;
				if (obj->get_can_id() == CAN_MCU_STATUS_2_ID || obj->get_can_id() == CAN_BMS_ID || obj->get_can_id() == CAN_MCU_GENERAL_STATUS_ID)
				{
					RCUTILS_INFO(CAN_ADAPTOR, "extended id : %d", obj->get_can_id());
					param.append(ch).append(",").append(std::to_string(obj->get_can_id())).append(":").append(CAN_EFF);
				}
				else
				{
					param.append(ch).append(",").append(std::to_string(obj->get_can_id())).append(":").append(ONLY_SFF);
				}
				parameter_map.insert(std::make_pair(ch, param));
			}
		}

		std::vector<std::string> arg_val;
		int idx = 0;
		for (const std::pair<const std::string, std::string> &iter : parameter_map)
		{
			RCUTILS_INFO(CAN_ADAPTOR, "parameter_map argval : %s", iter.second.c_str());
			arg_val.push_back(iter.second);
			idx++;
		}

		try
		{
			std::thread recv_thread(
				[&](int argc,
					std::vector<std::string> argv,
					Adaptor *p_class_type,
					void (Adaptor::*func)(unsigned char *data, int can_id))
				{
					this->can_open(argc, argv, p_class_type, func);
				},
				parameter_map.size(),
				arg_val,
				this,
				&Adaptor::can_receive);

			recv_thread.detach();
		}
		catch (std::exception &expn)
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "Exception : %s", expn.what());
		}

		RCUTILS_INFO(CAN_ADAPTOR, "<Start detecting receive data>");

		return 0;
	}

	/**
	 * @brief Function to open can channel
	 * @details
	 * @param
	 * @return  Result of processing, 0 if successful
	 * @warning
	 * @exception
	 */
	int
	Adaptor::can_open(int argc, std::vector<std::string> argv, Adaptor *p_class_type, void (Adaptor::*func)(unsigned char *data, int can_id))
	{
		if (this->can_dump_ == nullptr)
		{
			return -1;
		}

		this->can_dump_->socket_open(argc, argv, this, func);

		return 0;
	}

	/**
	 * @brief Check whether the can channel is activated.
	 * @details
	 * @param device Channel name to check
	 * @param callback_func Function pointer to be called when a fault occurs
	 * @return
	 * @warning
	 * @exception
	 */
	void
	Adaptor::check_socket_status(std::vector<std::string> device, std::function<void(short, short, char, char, short)> callback_func)
	{
		int (Adaptor::*pFunc1)(std::vector<std::string>) = &Adaptor::can_socket_open;
		std::function<int(std::vector<std::string>)> open_func = std::move(std::bind(pFunc1, this, _1));

		std::thread([&](std::vector<std::string> dev, std::function<void(short, short, char, char, short)> func)
					{
	        while (true)
	        {
				sleep(2);

				for (std::vector<std::string>::iterator iter = dev.begin(); iter != dev.end(); ++iter)
	            {

	                if (this->is_connected((char *)iter->c_str()) == false)
	                {
						RCUTILS_ERROR(CAN_ADAPTOR, "Socket check result, invalid can device : %s", iter->c_str());
	                    func(CAN_DEVICE_FAULT, 0x00, 0x00, 0x00, 0x00);
	                    this->release();

						RCUTILS_WARN(CAN_ADAPTOR, "try reopen socket");

	                    if (this->can_socket_open(dev) == 0)
	                    {
	                        func(CAN_NO_FAULT, 0x00, 0x00, 0x00, 0x00);
	                    }
	                    break;
	                }
	            }
	        } }, device, callback_func)
			.detach();
	}

	/**
	 * @brief Receive data from can network
	 * @details Search and call a function mapped with canid in the function map.
	 * @param data received data body
	 * @param can_id received data can id
	 * @return void
	 * @exception
	 */
	void
	Adaptor::can_receive(unsigned char *data, int can_id)
	{
		const std::map<int, std::shared_ptr<CanCallback>>::iterator &func_it = this->funcs_map_.find(can_id);

		if (func_it != this->funcs_map_.end())
		{
			const std::shared_ptr<CanCallback> &object = func_it->second;

			if (object == nullptr)
			{
				RCUTILS_ERROR(CAN_ADAPTOR, "CanReceive, object is null");
				return;
			}

			CanCallback *lp_cls = (CanCallback *)object.get();
			std::function<void(byte *)> func = lp_cls->get_handler();

			if (func == nullptr)
			{
				RCUTILS_ERROR(CAN_ADAPTOR, "CanReceive, func is null");
				return;
			}

			func(data);
		}
		else
		{
			return;
		}
	}

	/**
	 * @brief Each data type is transmitted through the CAN network.
	 * @details
	 * @param can_id can id
	 * @param device can channel
	 * @return  void
	 * @warning
	 * @exception
	 */
	void
	Adaptor::post_message_by_type(byte *data, unsigned int can_id, std::string device)
	{
		if (this->is_connected(device) == false)
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "PostMessageByType, The socket is invalid and does not transmit");
			return;
		}

		byte temp[CAN_MAX_DLEN];
		std::memcpy(temp, (void *)data, CAN_MAX_DLEN);
		std::vector<byte> body;
		for (byte value : temp)
		{
			body.emplace_back(value);
		}

		int (Adaptor::*p_func)(std::vector<byte>, unsigned int, std::string) = &Adaptor::can_send;
		std::function<void(std::vector<byte>, unsigned int, std::string)> post_message_func = std::move(std::bind(p_func, this, _1, std::placeholders::_2, std::placeholders::_3));

		std::thread([&](std::function<void(std::vector<byte>, unsigned int, std::string)> func, std::vector<byte> data, unsigned int id, std::string dev)
					{ func(data, id, (char *)dev.c_str()); }, post_message_func, body, can_id, device)
			.detach();
	}

	/**
	 * @brief Each data type is transmitted repeatedly over the CAN network
	 * @details
	 * @param body transmission body
	 * @param can_id can id
	 * @param device can channel
	 * @param duration transmission frequency (milliseconds)
	 * @return  void
	 * @warning currently not used
	 * @exception
	 */
	void
	Adaptor::post_message_by_type(byte *data, unsigned int can_id, std::string device, int duration)
	{
		byte temp[CAN_MAX_DLEN];
		std::memcpy(temp, (void *)data, CAN_MAX_DLEN);
		std::vector<byte> body;
		for (byte value : temp)
		{
			body.emplace_back(value);
		}

		int (Adaptor::*p_func)(std::vector<byte>, unsigned int, std::string) = &Adaptor::can_send;
		std::function<void(std::vector<byte>, unsigned int, std::string)> post_message_func = std::move(std::bind(p_func, this, _1, std::placeholders::_2, std::placeholders::_3));

		this->stop_post_message(can_id);

		std::thread send_thread(([&](std::function<void(std::vector<byte>, unsigned int, std::string)> func, std::vector<byte> data, unsigned int id, std::string dev, int dur)
								 {
			while (true)
			{
				const std::chrono::time_point<std::chrono::steady_clock> &ms = std::chrono::steady_clock::now() + std::chrono::milliseconds(dur);
				func(data, id, (char*)dev.c_str());

				if (dur <= 0)
				{
					return;
				}
				std::this_thread::sleep_until(ms);
			} }),
								post_message_func, body, can_id, device, duration);

		this->post_msg_thread_map_[can_id] = send_thread.native_handle();
		send_thread.detach();
	}

	/**
	 * @brief Stop messages that are sent repeatedly over the CAN network
	 * @details
	 * @param can_id can id
	 * @return  void
	 * @warning currently not used
	 * @exception
	 */
	void
	Adaptor::stop_post_message(unsigned int can_id)
	{
		const ThreadMap::const_iterator &it = this->post_msg_thread_map_.find(can_id);

		if (it != this->post_msg_thread_map_.end())
		{
			pthread_cancel(it->second);
			this->post_msg_thread_map_.erase(can_id);
		}
	}

	/**
	 * @brief Send data to the CAN network.
	 * @details
	 * @param data transmission body
	 * @param msg_id can id
	 * @param device channel
	 * @return  int
	 * @warning
	 * @exception
	 */
	int
	Adaptor::can_send(std::vector<byte> data, unsigned int msg_id, std::string device)
	{
		if (this->can_send_->send(data, msg_id, (char *)device.c_str()) != 0)
		{
			return -1;
		}

		return 0;
	}

	/**
	 * @brief Check can device connection status
	 * @details
	 * @param device channel
	 * @return  true if successful, false otherwise
	 * @warning
	 * @exception
	 */
	bool
	Adaptor::is_connected(std::string device)
	{
		if (this->can_send_ == nullptr)
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "IsConnected, invalid can send object");
			return false;
		}

		if (this->can_send_->is_connected((char *)device.c_str()) == false)
		{
			RCUTILS_ERROR(CAN_ADAPTOR, "IsConnected, Not currently connected to CAN network, device : %s", device.c_str());
			return false;
		}

		return true;
	}
}
