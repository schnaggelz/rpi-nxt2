/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ communication library
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __NXT_COM_USB_DEVICE_HPP__
#define __NXT_COM_USB_DEVICE_HPP__

#include "nxt/com/protocol.hpp"
#include "nxt/com/protocol/generic_packet.hpp"

#include <libusb-1.0/libusb.h>

#include <cstdint>

#ifndef NULL
#define NULL ((void*)0)
#endif

namespace nxt
{
namespace com
{
namespace usb
{
using GenericPacket = nxt::com::protocol::generic::Packet;

constexpr std::uint16_t VENDOR_ID = 0x0694;
constexpr std::uint16_t PRODUCT_ID = 0x0002;

constexpr std::uint8_t LIBUSB_ENDPOINT_OUT = 0x01;
constexpr std::uint8_t LIBUSB_ENDPOINT_IN = 0x82;

constexpr std::uint16_t LIBUSB_RX_TIMEOUT = 1000;

constexpr std::uint16_t TX_RX_BYTES = GenericPacket::NUM_BYTES;

class Device
{
    using BufferPtr = const unsigned char*;

  public:
    Device()
        : _dev_handle(NULL)
        , _dev_ready(false){};

    bool init();
    bool open();
    void close();
    void exit();

    bool isReady()
    {
        return _dev_ready;
    }

    bool read(GenericPacket&);
    bool write(const GenericPacket&);
    bool callback(const GenericPacket&);

    std::int32_t getLastReturnCode()
    {
        return _return_code;
    }

  private:
    static void LIBUSB_CALL callbackWrapper(struct libusb_transfer* transfer);
    static void unpack(GenericPacket& packet, BufferPtr buf,
                       std::uint32_t nbytes);

  private:
    struct libusb_device_handle* _dev_handle{NULL};
    bool _dev_ready{false};

    std::int32_t _return_code{0};
};

} // namespace usb
} // namespace com
} // namespace nxt

#endif /* __NXT_COM_USB_DEVICE_HPP__ */
