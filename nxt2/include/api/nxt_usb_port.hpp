#ifndef __NXT_USB_PORT_HPP__
#define __NXT_USB_PORT_HPP__

#include "nxt_usb_data.hpp"

namespace nxt
{
class USBPort
{
  public:
    USBPort() = default;

    bool isConnected();

    bool read(nxt::USBData& data);
    bool write(nxt::USBData& data);

    void init();
    void exit();
};
} // namespace nxt

#endif /* __NXT_USB_PORT_HPP__ */
