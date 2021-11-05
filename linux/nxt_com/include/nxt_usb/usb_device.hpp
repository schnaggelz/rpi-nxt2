/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT C++ communication library
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#ifndef __NXT_COM_USB_DEVICE_HPP__
#define __NXT_COM_USB_DEVICE_HPP__

#include <libusb-1.0/libusb.h>

#include <cstdint>

#ifndef NULL
#define NULL ((void*)0)
#endif

namespace nxt_com
{
namespace usb
{
struct DataPacket
{
    std::uint16_t id;
    std::uint16_t size;
    std::int32_t data[8];
};

static_assert(sizeof(DataPacket) == 36);

constexpr std::uint16_t VENDOR_ID = 0x0694;
constexpr std::uint16_t PRODUCT_ID = 0x0002;

constexpr std::uint8_t LIBUSB_ENDPOINT_OUT = 0x01;
constexpr std::uint8_t LIBUSB_ENDPOINT_IN = 0x82;

constexpr std::uint16_t LIBUSB_RX_TIMEOUT = 1000;

constexpr std::uint8_t DEFAULT_CMD_ID = 42;
constexpr std::uint8_t MAX_CMD_VALUES = 4;

constexpr std::uint16_t TX_RX_BYTES = MAX_CMD_VALUES * 4 + 2;

class USBDevice
{
  public:
    USBDevice() : _dev_handle(NULL), _dev_ready(false){};

    bool init();
    bool open();
    void close();
    void exit();

    inline bool isReady()
    {
        return _dev_ready;
    };

    void read(DataPacket&);
    void write(const DataPacket&);

  private:
    struct libusb_device_handle* _dev_handle{NULL};
    bool _dev_ready{false};
};
} // namespace usb
} // namespace nxt_com

#endif /* __NXT_COM_USB_DEVICE_HPP__ */
