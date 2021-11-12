#ifndef RM_BASE__FIXED_PACKET_TOOL_HPP_
#define RM_BASE__FIXED_PACKET_TOOL_HPP_

#include <iostream>
#include <memory>
#include <stdexcept>

#include "rm_base/transporter_interface.hpp"
#include "rm_base/fixed_packet.hpp"
#include <stdio.h>

namespace rm_base
{
    /**
     * @brief 数据包串口处理类
     * 
     * @tparam capacity 
     */
    template <int capacity = 32>
    class FixedPacketTool
    {
    public:
        using SharedPtr = std::shared_ptr<FixedPacketTool>;
        FixedPacketTool() = delete;
        explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
            : transporter_(transporter)
        {
            if (!transporter)
            {
                throw std::invalid_argument("transporter is nullptr");
            }
        }

    public:
        /**
         * @brief 返回串口打开状态
         * 
         * @return true 
         * @return false 
         */
        bool is_open() { return transporter_->is_open(); } 

        /**
         * @brief 将buffer缓存数据写入串口
         * 
         * @param {FixedPacket<capacity>} &packet 
         * @return true 
         * @return false 
         */
        bool send_packet(const FixedPacket<capacity> &packet); 

        /**
         * @brief 接收串口数据，存入buffer缓存区
         * 
         * @param {FixedPacket<capacity>} &packet 
         * @return true 
         * @return false 
         */
        bool recv_packet(FixedPacket<capacity> &packet);

    private:
        /**
         * @brief 校验检测BCC校验码
         * 
         * @param {uint8_t} *tmp_buffer 
         * @param {uint8_t} check_byte 
         * @return true 
         * @return false 
         */
        bool BCCcheck(uint8_t *tmp_buffer, uint8_t check_byte);

        /**
         * @brief BCC校验码计算
         * 
         * @param {uint8_t} *buffer 
         * @return uint8_t
         */
        uint8_t BCCcode(uint8_t *buffer);

        /**
         * @brief 检测帧是否完整
         * 
         * @param tmp_buffer 
         * @param recv_len 
         * @return true 
         * @return false 
         */
        bool check_packet(uint8_t *tmp_buffer, int recv_len);

        std::shared_ptr<TransporterInterface> transporter_;
        // 数据
        uint8_t tmp_buffer_[capacity];
        uint8_t recv_buffer_[capacity * 2];
        int recv_buf_len_;
        // 错误码枚举变量
        int send_err_code_{0};
        int recv_err_code_{0};
    };

    // BCC校验码计算
    template <int capacity>
    uint8_t FixedPacketTool<capacity>::BCCcode(uint8_t *buffer)
    {
        uint8_t code = 0x00;
        for (int i=1; i<capacity-2; i++)
            code ^= buffer[i];
        return code;
    }

    // 校验检测BCC校验码
    template<int capacity>
    bool FixedPacketTool<capacity>::BCCcheck(uint8_t *buffer, uint8_t check_byte)
    {
        if (BCCcode(buffer) == check_byte) 
            return true;
        return false;
    }

    template <int capacity>
    bool FixedPacketTool<capacity>::check_packet(uint8_t *buffer, int recv_len)
    {
        // 检查长度是否等于capacity
        if (recv_len != capacity)   {return false;}
        // 检查帧头，帧尾是否为默认值
        if ((buffer[0] != 0xff) || (buffer[capacity - 1] != 0x0d))  {return false;}
        // TODO：检查check_byte(buffer[capacity-2],可用异或校验BCC)
        if (!BCCcheck(buffer, buffer[capacity-2])) {return false;}
        return true;
    }

    template <int capacity>
    bool FixedPacketTool<capacity>::send_packet(const FixedPacket<capacity> &packet)
    {
        printf("send:  ");
        for (int i = 0; i < capacity; i++){
            printf("%x ",+packet.buffer()[i]);
        }
        printf("\n");

        if (transporter_->write(packet.buffer(), capacity) == capacity)
        {
            return true;
        }
        else
        {
            //重连
            transporter_->close();
            transporter_->open();
            return false;
        }
    }

    template <int capacity>
    bool FixedPacketTool<capacity>::recv_packet(FixedPacket<capacity> &packet)
    {
        int recv_len = transporter_->read(tmp_buffer_, capacity);

        printf("recv:  ");
        for (int i = 0; i < capacity; i++){
            printf("%x ",+tmp_buffer_[i]);
        }
        printf("\n");

        if (recv_len > 0)
        {
            //检测帧是否有错误
            if (check_packet(tmp_buffer_, recv_len))
            {
                //如果帧无错误，则读取
                packet.copy_from(tmp_buffer_);
                return true;
            }
            else
            {
                //如果帧有错误，拼接buffer，遍历校验，得到合法数据
                if (recv_buf_len_ + recv_len > capacity * 2)
                {
                    recv_buf_len_ = 0;
                }
                //拼接
                memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len);
                recv_buf_len_ = recv_buf_len_ + recv_len;
                //遍历校验
                for (int i = 0; (i + capacity) <= recv_buf_len_; i++)
                {
                    if (check_packet(recv_buffer_ + i, capacity))
                    {
                        packet.copy_from(recv_buffer_ + i);
                        //读取一帧后，更新接收缓存
                        int k = 0;
                        for (int j = i + capacity; j < recv_buf_len_; j++, k++)
                        {
                            recv_buffer_[k] = recv_buffer_[j];
                        }
                        recv_buf_len_ = k;
                        return true;
                    }
                }
                // 断帧，错误帧
                recv_err_code_ = -1;
                return false;
            }
        }
        else
        {
            //重连
            transporter_->close();
            transporter_->open();
            //串口错误
            recv_err_code_ = -2;
            return false;
        }
    }

    using FixedPacketTool16 = FixedPacketTool<16>;
    using FixedPacketTool32 = FixedPacketTool<32>;
    using FixedPacketTool64 = FixedPacketTool<64>;
}

#endif
