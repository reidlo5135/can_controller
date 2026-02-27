#include "send.hpp"

namespace net::wavem::can
{
    Send::Send() = default;
    Send::~Send() = default;

    /**
     * @brief Send data to the CAN network.
     * @date    2025-01-15
     * @author  reidlo(naru5135@wavem.net)
     * @details
     * @param data transmission body
     * @param msg_id can id
     * @param device channel
     * @return  int
     * @warning
     * @exception
     */
    int
    Send::send(std::vector<unsigned char> data, unsigned int msg_id, char *device)
    {
        struct ifreq ifr{};
        unsigned char body[CAN_MAX_DLEN];
        int idx = 0;

        for (unsigned char &iter : data)
        {
            body[idx++] = iter;
        }

        const std::map<std::string, int>::iterator &item = this->sock_map_.find(device);
        if (item == this->sock_map_.end())
        {
            RCUTILS_INFO(CAN_SEND, "device does not exist! ( %s )\r\n", device);
            return -1;
        }

        const int &s = item->second;
        const int &required_mtu = CAN_MTU;
        struct canfd_frame frame{};

        frame.can_id = msg_id;
        frame.len = CAN_MAX_DLEN;
        memcpy(frame.data, body, CAN_MAX_DLEN);

        RCUTILS_INFO(CAN_SEND, "<channel> %s, <can_id> = 0x%X, %d <can_dlc> = %d\r\n", device, frame.can_id, frame.can_id, frame.len);

        if (write(s, &frame, required_mtu) != required_mtu)
        {
            perror("write");
            return -1;
        }

        std::string msg("  <data> = ");
        char buf[10];
        memset(buf, 0x00, 10);

        for (int i = 0; i < CAN_MAX_DLEN; i++)
        {
            sprintf(buf, "0x%02x", frame.data[i]);
            msg.append(buf);
            msg.append(" ");
        }

        msg.append(" <data transfer success>\n");
        RCUTILS_INFO(CAN_SEND, "%s", msg.c_str());

        return 0;
    }

    /**
     * @brief open the socket
     * @date    2025-01-15
     * @author  reidlo(naru5135@wavem.net)
     * @details
     * @param device channel names
     * @return
     * @warning
     * @exception
     */
    int
    Send::socket_open(std::vector<std::string> device)
    {
        for (const std::string &iter : device)
        {
            if (socket_open((char *)iter.c_str()) != 0)
            {
                return -1;
            }
        }
        return 0;
    }

    /**
     * @brief open the socket
     * @date    2025-01-15
     * @author  reidlo(naru5135@wavem.net)
     * @details
     * @param device channel name
     * @return
     * @warning
     * @exception
     */
    int
    Send::socket_open(char *device)
    {
        int ret;
        int s;

        struct sockaddr_can addr{};
        struct canfd_frame frame{};
        struct ifreq ifr{};

        const std::map<std::string, int>::iterator &item = this->sock_map_.find(device);

        if (item != this->sock_map_.end())
        {
            RCUTILS_INFO(CAN_SEND, "device exists! -  %s ; %d", item->first.c_str(), item->second);
            return -1;
        }

        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            return -1;
        }

        strcpy(ifr.ifr_name, device);
        ret = ioctl(s, SIOCGIFINDEX, &ifr);

        if (ret < 0)
        {
            return -1;
        }

        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

        if (!ifr.ifr_ifindex)
        {
            perror("ifr_ifindex");
            return -1;
        }

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            return -1;
        }

        this->sock_map_.insert(std::make_pair(device, s));
        RCUTILS_INFO(CAN_SEND, "can device open ( %s ) , sock_fd : %d\n", device, s);

        return 0;
    }

    /**
     * @brief close the socket
     * @date    2025-01-15
     * @author  reidlo(naru5135@wavem.net)
     * @details close the socketof all channels stored in the this object
     * @param
     * @return
     * @warning
     * @exception
     */
    void
    Send::socket_close()
    {
        for (const std::pair<const std::string, int> &iter : this->sock_map_)
        {
            close(iter.second);
            RCUTILS_INFO(CAN_SEND, "(%s, %d)\n", iter.first.c_str(), iter.second);
        }
        this->sock_map_.clear();
    }

    /**
     * @brief Check can device connection status
     * @date    2025-01-15
     * @author  reidlo(naru5135@wavem.net)
     * @details
     * @param device channel name
     * @return  true if successful, false otherwise
     * @warning
     * @exception
     */
    bool
    Send::is_connected(char *device)
    {
        const std::map<std::string, int>::iterator &item = this->sock_map_.find(device);
        if (item == this->sock_map_.end())
        {
            RCUTILS_INFO(CAN_SEND, "device does not exist! ( %s )\r\n", device);
            return false;
        }

        const int &s = item->second;

        struct ifreq ifr{};
        memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
        strcpy(ifr.ifr_name, device);
        ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

        if (!ifr.ifr_ifindex)
        {
            return false;
        }

        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
        {
            return false;
        }

        if (ioctl(s, SIOCGIFMTU, &ifr) < 0)
        {
            return false;
        }

        return true;
    }
}