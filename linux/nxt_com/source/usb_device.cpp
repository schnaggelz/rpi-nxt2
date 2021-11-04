/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT C++ communication library
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "nxt_usb/usb_device.hpp"

namespace nxt_com
{

bool USBDevice::init()
{
    return libusb_init(0) == 0;
}

bool USBDevice::open()
{
    int rc = 0;

    libusb_device** dev_list;
    libusb_device* nxt_dev = NULL;

    ssize_t num_dev = libusb_get_device_list(0, &dev_list);

    for (size_t dev_idx = 0; dev_idx < num_dev; ++dev_idx)
    {
        libusb_device* dev = dev_list[dev_idx];
        libusb_device_descriptor dev_desc = {0};

        rc = libusb_get_device_descriptor(dev, &dev_desc);
        if (rc >= 0)
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
        rc = libusb_open(nxt_dev, &m_dev_handle);
        if (rc >= 0 && m_dev_handle != 0)
        {
            libusb_detach_kernel_driver(m_dev_handle, 0);

            rc = libusb_set_configuration(m_dev_handle, 1);
            if (rc >= 0)
            {
                rc = libusb_claim_interface(m_dev_handle, 0);
                if (rc >= 0)
                {
                    m_dev_ready = true;
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

void USBDevice::read(DataPacket& pkg)
{
    int rc = 0;

    unsigned char buf[sizeof(pkg)];
    int nbytes;

    if (m_dev_ready)
    {
        rc = libusb_bulk_transfer(m_dev_handle, LIBUSB_ENDPOINT_IN, buf,
                                  sizeof(buf), &nbytes, LIBUSB_RX_TIMEOUT);

        if (rc == 0 && nbytes >= 4)
        {
            pkg.id = ((uint16_t)buf[1] << 8) | (uint8_t)buf[0];
            pkg.size = ((uint16_t)buf[3] << 8) | (uint8_t)buf[2];

            size_t ndata = (nbytes - 4) / 4;

            for (int i = 0; i < ndata; ++i)
            {
                unsigned char* ptr = &(buf[4 * i + 4]);
                int32_t v = (((int32_t)ptr[3]) << 24) |
                            (((int32_t)ptr[2]) << 16) |
                            (((int32_t)ptr[1]) << 8) | (((int32_t)ptr[0]) << 0);

                pkg.data[i] = v;
            }
        }
    }
}

void USBDevice::write(const DataPacket& pkg)
{
    int rc = 0;

    unsigned char buf[sizeof(pkg)];
    int nbytes;

    if (m_dev_ready)
    {
        buf[0] = (pkg.id >> 0) & 0xFF;
        buf[1] = (pkg.id >> 8) & 0xFF;
        buf[2] = (pkg.size >> 0) & 0xFF;
        buf[3] = (pkg.size >> 8) & 0xFF;

        for (int i = 0; i < 2; ++i)
        {
            unsigned char* ptr = &(buf[4 * i + 4]);
            ptr[0] = (pkg.data[i] >> 0) & 0xFF;
            ptr[1] = (pkg.data[i] >> 16) & 0xFF;
            ptr[2] = (pkg.data[i] >> 16) & 0xFF;
            ptr[3] = (pkg.data[i] >> 24) & 0xFF;
        }

        rc = libusb_bulk_transfer(m_dev_handle, LIBUSB_ENDPOINT_OUT, buf,
                                  sizeof(buf), &nbytes, LIBUSB_RX_TIMEOUT);
    }
}

void USBDevice::close()
{
    if (m_dev_ready)
    {
        libusb_release_interface(m_dev_handle, 0);
        libusb_close(m_dev_handle);

        m_dev_ready = false;
    }
}

void exit()
{
    libusb_exit(0);
}

} // namespace nxt_com