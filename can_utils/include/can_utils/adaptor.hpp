#ifndef CAN_UTILS_ADAPTOR_HPP
#define CAN_UTILS_ADAPTOR_HPP

#include <string.h>
#include <unistd.h>
#include <thread>
#include <functional>
#include <iostream>
#include <sstream>
#include <memory>
#include <map>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <net/if.h>
#include <linux/can.h>
#include <can_dbc/dbc1.hpp>
#include <can_dbc/dbc2.hpp>

#include "callback.hpp"
#include "adaptor.hpp"
#include "dump.hpp"
#include "send.hpp"
#include "logger.hpp"

#define CAN_ADAPTOR "can_utils_adaptor"
#define byte unsigned char
#define ONLY_SFF "7FF"
#define CAN_EFF "1FFFFFFF"
#define CAN_NO_FAULT 0x00
#define CAN_DEVICE_FAULT 0x01
#define CAN_MCU_STATUS_2_ID 418513903
#define CAN_BMS_ID 418448883
#define CAN_MCU_GENERAL_STATUS_ID 82969071

using std::placeholders::_1;

namespace net::wavem::can
{
    class Dump;
    class Send;

    /**
     * @class   Adaptor
     * @date    2025-01-13
     * @author  reidlo(naru5135@wavem.net)
     * @brief   CAN network relaying class
     * @version 0.1.0
     */
    class Adaptor final
    {
    private:
        std::map<int, std::shared_ptr<CanCallback>> funcs_map_;
        bool is_big_endian_;
        bool register_check_;
        std::shared_ptr<Dump> can_dump_;
        std::shared_ptr<Send> can_send_;

        /**
         * @brief CAN -> ROS
         */
        typedef std::function<void(net::wavem::can::can1::VCU_Vehicle_Error_Status)> VCU_Vehicle_Error_Status_func;
        VCU_Vehicle_Error_Status_func VCU_Vehicle_Error_Status_handler_;

        typedef std::function<void(net::wavem::can::can1::DBS_Status2)> DBS_Status2_func;
        DBS_Status2_func DBS_Status2_handler_;

        typedef std::function<void(net::wavem::can::can1::DBS_Status)> DBS_Status_func;
        DBS_Status_func DBS_Status_handler_;

        typedef std::function<void(net::wavem::can::can1::Vehicle_Odometer_Status)> Vehicle_Odometer_Status_func;
        Vehicle_Odometer_Status_func Vehicle_Odometer_Status_handler_;

        typedef std::function<void(net::wavem::can::can2::MCU_Request)> MCU_Request_func;
        MCU_Request_func MCU_Request_handler_;

        typedef std::function<void(net::wavem::can::can2::BMS_Pack_Status)> BMS_Pack_Status_func;
        BMS_Pack_Status_func BMS_Pack_Status_handler_;

        typedef std::function<void(net::wavem::can::can2::BMS_Error)> BMS_Error_func;
        BMS_Error_func BMS_Error_handler_;

        typedef std::function<void(net::wavem::can::can2::BMS_Charge_Status)> BMS_Charge_Status_func;
        BMS_Charge_Status_func BMS_Charge_Status_handler_;

        typedef std::function<void(net::wavem::can::can2::MCU_General_Status)> MCU_General_Status_func;
        MCU_General_Status_func MCU_General_Status_handler_;

        typedef std::function<void(net::wavem::can::can2::MCU_Status_2)> MCU_Status_2_func;
        MCU_Status_2_func MCU_Status_2_handler_;

        typedef std::map<unsigned int, pthread_t> ThreadMap;
        ThreadMap post_msg_thread_map_;

        int can_send(std::vector<byte> body, unsigned int msg_id, std::string device);
        void can_receive(unsigned char *data, int can_id);
        int can_open(int arc, std::vector<std::string> argv, Adaptor *p_class_type, void (Adaptor::*func)(unsigned char *data, int can_id));

        /**
         * @brief ROS -> CAN
         */
        void post_message_by_type(byte *body, unsigned int can_id, std::string device);
        void post_message_by_type(byte *data, unsigned int can_id, std::string device, int duration);

        int can_socket_open(std::vector<std::string> device);
        int can_reception_open();

    public:
        explicit Adaptor();
        virtual ~Adaptor();

        int initialize(const bool &endian);
        void release() const;
        int open(std::vector<std::string> device);
        bool is_connected(std::string device);
        void check_socket_status(std::vector<std::string> device, std::function<void(short, short, char, char, short)> func);
        void stop_post_message(unsigned int can_id);

        using SharedPtr = std::shared_ptr<Adaptor>;

    public:
        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can1::VCU_Vehicle_Error_Status),
            int can_id,
            std::string device)
        {
            this->VCU_Vehicle_Error_Status_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can1::VCU_Vehicle_Error_Status r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->VCU_Vehicle_Error_Status_handler_((net::wavem::can::can1::VCU_Vehicle_Error_Status)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(VCU_Vehicle_Error_Status) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can1::DBS_Status2),
            int can_id,
            std::string device)
        {
            this->DBS_Status2_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can1::DBS_Status2 r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->DBS_Status2_handler_((net::wavem::can::can1::DBS_Status2)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(DBS_Status2) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can1::DBS_Status),
            int can_id,
            std::string device)
        {
            this->DBS_Status_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can1::DBS_Status r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->DBS_Status_handler_((net::wavem::can::can1::DBS_Status)r);
                });

            std::cout << "setHandler(DBS_Status) : " + device << ", can_id : " << can_id << '\n';
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can1::Vehicle_Odometer_Status),
            int can_id,
            std::string device)
        {
            this->Vehicle_Odometer_Status_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can1::Vehicle_Odometer_Status r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->Vehicle_Odometer_Status_handler_((net::wavem::can::can1::Vehicle_Odometer_Status)r);
                });

            std::cout << "SetHandler(Vehicle_Odometer_Status) : " + device << ", can_id : " << can_id << '\n';
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can2::BMS_Pack_Status),
            int can_id,
            std::string device)
        {
            this->BMS_Pack_Status_handler_ = std::move(bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can2::BMS_Pack_Status r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->BMS_Pack_Status_handler_((net::wavem::can::can2::BMS_Pack_Status)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(BMS_Pack_Status) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can2::BMS_Error),
            int can_id,
            std::string device)
        {
            this->BMS_Error_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can2::BMS_Error r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->BMS_Error_handler_((net::wavem::can::can2::BMS_Error)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(BMS_Error) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can2::BMS_Charge_Status),
            int can_id,
            std::string device)
        {
            this->BMS_Charge_Status_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can2::BMS_Charge_Status r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->BMS_Charge_Status_handler_((net::wavem::can::can2::BMS_Charge_Status)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(BMS_Charge_Status) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can2::MCU_General_Status),
            int can_id,
            std::string device)
        {
            this->MCU_General_Status_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can2::MCU_General_Status r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->MCU_General_Status_handler_((net::wavem::can::can2::MCU_General_Status)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(MCU_General_Status) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void set_handler(
            T *p_class_type,
            void (T::*p_func)(net::wavem::can::can2::MCU_Status_2),
            int can_id,
            std::string device)
        {
            this->MCU_Status_2_handler_ = std::move(std::bind(p_func, p_class_type, std::placeholders::_1));

            std::shared_ptr<CanCallback> object = std::make_shared<CanCallback>(
                can_id,
                device,
                [&](byte *data)
                {
                    net::wavem::can::can2::MCU_Status_2 r;
                    std::memcpy((void *)&r, data, CAN_MAX_DLEN);
                    this->MCU_Status_2_handler_((net::wavem::can::can2::MCU_Status_2)r);
                });

            RCUTILS_INFO(CAN_ADAPTOR, "SetHandler(MCU_Status_2) device : %s, can_id : %d", device.c_str(), can_id);
            this->funcs_map_.insert(std::make_pair(can_id, object));
            this->register_check_ = true;
        }

        template <typename T>
        void post_can_message(T struct_type_data, int msg_id, std::string device)
        {
            byte body[CAN_MAX_DLEN];
            std::memcpy(body, (void *)&struct_type_data, CAN_MAX_DLEN);
            post_message_by_type(body, msg_id, device);
        }

        template <typename T>
        void post_can_message(T struct_type_data, int msg_id, std::string device, int duration)
        {
            byte body[CAN_MAX_DLEN];
            std::memcpy(body, (void *)&struct_type_data, CAN_MAX_DLEN);
            post_message_by_type(body, msg_id, device, duration);
        }
    };
}

#endif