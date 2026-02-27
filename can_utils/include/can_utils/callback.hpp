#ifndef CAN_UTILS_CALLBACK_HPP
#define CAN_UTILS_CALLBACK_HPP

#include <string>
#include <functional>

/**
    @file    can_callback.hpp
    @namespace net::wavem::can
    @date    2023-02-14
    @author  ys.kwon(ys.kwon@wavem.net)
	@date    2025-01-13
	@author  reidlo(naru5135@wavem.net)
    @brief
*/
namespace net::wavem::can
{
    typedef std::function<void(unsigned char*)> callback_function;

    /**
    @class   CanCallback
    @date    2023-02-14
    @author  ys.kwon(ys.kwon@wavem.net)
    @date    2025-01-13
    @author  reidlo(naru5135@wavem.net)
    @brief   Callback function storage class mapped with Can Id
    @version 0.0.1
    @warning
*/
    class CanCallback final
    {
    private:
        int can_id_;
        std::string channel_;
        callback_function handler_;

    public:
        explicit CanCallback(const int &can_id, const std::string &channel, callback_function func)
        {
            this->handler_ = move(func);
            this->can_id_ = can_id;
            this->channel_ = channel;
        }

        virtual ~CanCallback() = default;

        void set_handler(callback_function func)
        {
            this->handler_ = move(func);
        }

        callback_function get_handler()
        {
            return this->handler_;
        }

        int get_can_id() const
        {
            return this->can_id_;
        }

        std::string get_channel() const
        {
            return this->channel_;
        }
    };
}

#endif
