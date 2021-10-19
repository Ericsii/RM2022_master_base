#ifndef RM_BASE__FIXED_PACKET_HPP_
#define RM_BASE__FIXED_PACKET_HPP_

#include <cstring>
#include <memory>
#include <iostream>

namespace rm_base
{
    //定长数据包封装： 16/32/64字节 [head_byte(0xff) , data_bytes , check_byte , tail_byte(0x0d)]
    template <int capacity = 16>
    class FixedPacket
    {
    public:
        using SharedPtr = std::shared_ptr<FixedPacket>;
        FixedPacket()
        {
            memset(buffer_, 0, capacity);
            buffer_[0] = 0xff;            //帧头
            buffer_[capacity - 1] = 0x0d; //帧尾
        }

        // 清除缓存，将data_bytes、check_byte都用0填充
        void clear() { memset(buffer_ + 1, 0, capacity - 2); }
        // 设置check_byte
        void set_check_byte() {
            uint8_t code = 0x00;
            for (int i=1; i<capacity-2; i++)
                code ^= buffer_[i];
            buffer_[capacity - 2] = code; 
        }
        // 将数据传入buffer_中
        void copy_from(const void *src) { memcpy(buffer_, src, capacity); }
        // 获取buffer_
        const uint8_t *buffer() const { return buffer_; }

        // 装载数据，将data数据传入buffer中的data_types
        template <typename T, int data_len = sizeof(T)>
        bool load_data(T const &data, int index)
        {
            // std::cout << index << " : " << +data ;
            // std::cout << "\n";
            
            //越界检测，data_types--buffer_(1,capacity-2)
            if (index > 0 && ((index + data_len) < (capacity - 2)))
            {
                memcpy(buffer_ + index, &data, data_len);
                // std::cout << data_len << ":";
                // for (int i = 0; i < int(sizeof(T)); i++){
                //     std::cout << +buffer_[index+i]<< " ";
                // }
                // std::cout << "\n";
                return true;
            }
            return false;
        }
        // 解析数据，将buffer中的data_types取出到data
        template <typename T, int data_len = sizeof(T)>
        bool unload_data(T &data, int index)
        {
            //越界检测
            if (index > 0 && ((index + data_len) < (capacity - 2)))
            {
                memcpy(&data, buffer_ + index, data_len);
                return true;
            }
            return false;
        }

    private:
        uint8_t buffer_[capacity]; //数据包缓存
    };
    using FixedPacket16 = FixedPacket<16>;
    using FixedPacket32 = FixedPacket<32>;
    using FixedPacket64 = FixedPacket<64>;
}
#endif
