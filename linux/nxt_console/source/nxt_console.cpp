/*******************************************************************************
 * Copyright (C) 2015 T. Reich
 *
 * NXT communication tool.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "simple_logger/logger.hpp"

#include "nxt_usb/usb_device.hpp"

#include <ncurses.h>

#include <atomic>
#include <chrono>
#include <thread>

std::atomic<bool> exiting = false;

nxt_com::usb::USBDevice nxt_usb_dev;
nxt_com::usb::DataPacket nxt_pkg_tx;
nxt_com::usb::DataPacket nxt_pkg_rx;

bool connect()
{
    if (nxt_usb_dev.init())
    {
        if (nxt_usb_dev.open())
        {
            return true;
        }
        else
        {
            nxt_usb_dev.exit();
        }
    }

    return false;
}

void send(uint16_t id)
{
    if (nxt_usb_dev.isReady())
    {
        nxt_pkg_tx.id = id;
        nxt_pkg_tx.size = 1;
        nxt_pkg_tx.data[0] = 42;

        nxt_usb_dev.write(nxt_pkg_tx);
    }
}

void receive()
{
    int counter = 0;

    mvprintw(3, 0, "RCV    : SQ:%8d|", 0);
    mvprintw(4, 0, "PACKET : ID=%8d| SZ=%8d", 0, 0);
    mvprintw(6, 0, "GENERIC: V0=%8d| V1=%8d| V2=%5d| V3=%8d|", 0, 0, 0, 0);
    mvprintw(7, 0, "GENERIC: V4=%8d| V5=%8d| V6=%5d| V7=%8d|", 0, 0, 0, 0);
    mvprintw(8, 0, "SONAR  : L0=%8d| C0=%8d| R0=%8d|", 0, 0, 0);
    mvprintw(9, 0, "COLOR  : R0=%8d| G0=%8d| B0=%8d|", 0, 0, 0);

    while (!exiting)
    {
        if (nxt_usb_dev.isReady())
        {
            nxt_usb_dev.read(nxt_pkg_rx);

            mvprintw(3, 0, "RCV    : SQ:%8d|", counter++);
            mvprintw(4, 0, "PACKET : ID=%8d| SZ=%8d", nxt_pkg_rx.id,
                     nxt_pkg_rx.size);

            switch (nxt_pkg_rx.id)
            {
            case 0x00: // GENERIC
            {
                mvprintw(6, 0, "GENERIC: V0=%8d| V1=%8d| V2=%8d| V3=%8d|",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2], nxt_pkg_rx.data[3]);

                mvprintw(7, 0, "GENERIC: V4=%8d| V5=%8d| V6=%8d| V7=%8d|",
                         nxt_pkg_rx.data[4], nxt_pkg_rx.data[5],
                         nxt_pkg_rx.data[6], nxt_pkg_rx.data[7]);
                break;
            }
            case 0x10: // SONAR
            {
                mvprintw(8, 0, "SONAR  : L0=%8d| C0=%8d| R0=%8d|",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2]);
                break;
            }
            case 0x11: // COLOR
            {
                mvprintw(9, 0, "COLOR  : R0=%8d| G0=%8d| B0=%8d|",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2]);
                break;
            }
            }

            refresh();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void disconnect()
{
    nxt_usb_dev.close();
}

int main()
{
    LOG_INFO("Connect to NXT ...");

    if (!connect())
    {
        LOG_ERROR("Creating NXT connection failed");
        return 1;
    }

    LOG_INFO("... connected (quit wit <BACKSPACE>)");

    std::this_thread::sleep_for(std::chrono::seconds(1));

    initscr();
    keypad(stdscr, TRUE);
    noecho();

    mvprintw(0, 0, "CMD:");
    mvprintw(0, 5, "< >");
    refresh();

    std::thread rt(&receive);
    rt.detach();

    int ch;

    while ((ch = getch()) != KEY_BACKSPACE)
    {
        switch (ch)
        {
        case KEY_F(0):
        case 'd':
        {
            mvprintw(0, 5, "<D>");
            send(0x10);
        }
        break;
        case KEY_F(1):
        case 'c':
        {
            mvprintw(0, 5, "<C>");
            send(0x11);
        }
        break;
        case KEY_HOME:
        case ' ':
        {
            mvprintw(0, 5, "<_>");
            send(0xA0);
        }
        break;
        case KEY_BACKSPACE:
        case '!':
        {
            mvprintw(0, 5, "<!>");
            send(0xA1);
        }
        break;
        case KEY_UP:
        case 'w':
        {
            mvprintw(0, 5, "<F>");
            send(0xA2);
        }
        break;
        case KEY_DOWN:
        case 'y':
        {
            mvprintw(0, 5, "<R>");
            send(0xA3);
        }
        break;
        case KEY_LEFT:
        case 'a':
        {
            mvprintw(0, 5, "<L>");
            send(0xA4);
        }
        break;
        case KEY_RIGHT:
        case 's':
        {
            mvprintw(0, 5, "<R>");
            send(0xA5);
        }
        break;
        case KEY_NPAGE:
        case '+':
        {
            mvprintw(0, 5, "<+>");
            send(0xA6);
        }
        break;
        case KEY_PPAGE:
        case '-':
        {
            mvprintw(0, 5, "<->");
            send(0xA7);
        }
        break;
        }

        refresh();
    }

    exiting = true;

    endwin();

    disconnect();

    LOG_INFO("Disconnected from NXT");

    return 0;
}
