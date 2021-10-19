#ifndef RM_BASE__TRANSPORTER_INTERFACE_HPP_
#define RM_BASE__TRANSPORTER_INTERFACE_HPP_

#include <memory>

namespace rm_base
{
    //Serial Transporter Interface
    class TransporterInterface
    {
    public:
        using SharedPtr = std::shared_ptr<TransporterInterface>;
        virtual bool open() = 0;
        virtual void close() = 0;
        virtual bool is_open() = 0;
        //return recv/send len >0, error <0
        virtual int read(void *buffer, size_t len) = 0;
        virtual int write(const void *buffer, size_t len) = 0;
    };
}

#endif // RM_BASE__TRANSPORTER_INTERFACE_
