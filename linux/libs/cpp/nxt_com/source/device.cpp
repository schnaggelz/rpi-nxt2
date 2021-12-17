/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ communication library
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "nxt/usb/device.hpp"

namespace nxt
{
namespace com
{
namespace usb
{
bool Device::init()
{
    return libusb_init(0) == 0;
}

bool Device::open()
{
    libusb_device** dev_list;
    libusb_device* nxt_dev = NULL;

    ssize_t num_dev = libusb_get_device_list(0, &dev_list);

    for (size_t dev_idx = 0; dev_idx < num_dev; ++dev_idx)
    {
        libusb_device* dev = dev_list[dev_idx];
        libusb_device_descriptor dev_desc = {0};

        _return_code = libusb_get_device_descriptor(dev, &dev_desc);
        if (_return_code >= 0)
        {
            if (dev_desc.idVendor == VENDOR_ID &&
                dev_desc.idProduct == PRODUCT_ID)
            {
                nxt_dev = dev;
            }
        }
    }

    libusb_free_device_list(dev_list, 1);

    if (nxt_dev)
    {
        _return_code = libusb_open(nxt_dev, &_dev_handle);
        if (_return_code >= 0 && _dev_handle != 0)
        {
            libusb_detach_kernel_driver(_dev_handle, 0);

            _return_code = libusb_set_configuration(_dev_handle, 1);
            if (_return_code >= 0)
            {
                _return_code = libusb_claim_interface(_dev_handle, 0);
                if (_return_code >= 0)
                {
                    _dev_ready = true;

                    return true;
                }
            }
            else
            {
                close();
            }
        }
    }

    return false;
}

bool Device::read(GenericPacket& packet)
{
    unsigned char buf[sizeof(packet)];
    int nbytes;

    if (_dev_ready)
    {
        _return_code =
            libusb_bulk_transfer(_dev_handle, LIBUSB_ENDPOINT_IN, buf,
                                 sizeof(buf), &nbytes, LIBUSB_RX_TIMEOUT);

        if (_return_code >= 0 && nbytes == TX_RX_BYTES)
        {
            unpack(packet, buf, nbytes);

            return true;
        }
    }

    return false;
}

bool Device::write(const GenericPacket& packet)
{
    unsigned char buf[sizeof(packet)];
    int nbytes;

    if (_dev_ready)
    {
        buf[0] = (packet.type >> 0) & 0xFF;
        buf[1] = (packet.type >> 8) & 0xFF;
        buf[2] = (packet.size >> 0) & 0xFF;
        buf[3] = (packet.size >> 8) & 0xFF;

        for (int i = 0; i < packet.size; ++i)
        {
            unsigned char* ptr = &(buf[4 * i + 4]);
            ptr[0] = (packet.data[i] >> 0) & 0xFF;
            ptr[1] = (packet.data[i] >> 8) & 0xFF;
            ptr[2] = (packet.data[i] >> 16) & 0xFF;
            ptr[3] = (packet.data[i] >> 24) & 0xFF;
        }

        _return_code =
            libusb_bulk_transfer(_dev_handle, LIBUSB_ENDPOINT_OUT, buf,
                                 sizeof(buf), &nbytes, LIBUSB_RX_TIMEOUT);

        return _return_code >= 0;
    }

    return false;
}

void Device::close()
{
    if (_dev_ready)
    {
        libusb_release_interface(_dev_handle, 0);
        libusb_close(_dev_handle);

        _dev_ready = false;
    }
}

void Device::exit()
{
    libusb_exit(0);
}

void Device::unpack(GenericPacket& packet, BufferPtr buf, std::uint32_t nbytes)
{
    packet.type = ((uint16_t)buf[1] << 8) | (uint8_t)buf[0];
    packet.size = ((uint16_t)buf[3] << 8) | (uint8_t)buf[2];

    size_t ndata = (nbytes - 4) / 4;

    for (int i = 0; i < ndata; ++i)
    {
        auto ptr = &(buf[4 * i + 4]);

        std::int32_t v =
            (((std::int32_t)ptr[3]) << 24) | (((std::int32_t)ptr[2]) << 16) |
            (((std::int32_t)ptr[1]) << 8) | (((std::int32_t)ptr[0]) << 0);

        packet.data[i] = v;
    }
}

void LIBUSB_CALL Device::callbackWrapper(struct libusb_transfer* transfer)
{
    GenericPacket packet;

    Device* device = reinterpret_cast<Device*>(transfer->user_data);

    if (transfer->actual_length == TX_RX_BYTES)
    {
        Device::unpack(packet, transfer->buffer, transfer->actual_length);

        device->callback(packet);
    }
}

} // namespace usb
} // namespace com
} // namespace nxt