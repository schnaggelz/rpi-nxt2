/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT communication tool.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "simple_logger/logger.hpp"

#include "nxt/utils/conversion.hpp"

#include "nxt/usb/device.hpp"

#include <ncurses.h>

#include <atomic>
#include <chrono>
#include <thread>

std::atomic<bool> exiting = false;

nxt_com::usb::Device nxt_usb_dev;
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

void send(const nxt::com::protocol::Command command, const nxt::com::protocol::Data& data)
{
    if (nxt_usb_dev.isReady())
    {
        nxt_pkg_tx.command = nxt::utils::to_underlying(command);
        nxt_pkg_tx.size = 1;

        for (auto idx = 0U; idx < nxt::com::protocol::Packet::NUM_DATA_ELEMENTS;
             ++idx)
        {
            nxt_pkg_tx.data[idx] = data[idx];
        }

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
            mvprintw(4, 0, "PACKET : ID=%8d| SZ=%8d", nxt_pkg_rx.command,
                     nxt_pkg_rx.size);

            switch (
                nxt::utils::to_enum<nxt::com::protocol::Command>(nxt_pkg_rx.command))
            {
            case nxt::com::protocol::Command::GENERIC_M: // GENERIC
            {
                mvprintw(6, 0, "GENERIC: V0=%8d| V1=%8d| V2=%8d| V3=%8d|",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2], nxt_pkg_rx.data[3]);

                mvprintw(7, 0, "GENERIC: V4=%8d| V5=%8d| V6=%8d| V7=%8d|",
                         nxt_pkg_rx.data[4], nxt_pkg_rx.data[5],
                         nxt_pkg_rx.data[6], nxt_pkg_rx.data[7]);
                break;
            }
            case nxt::com::protocol::Command::GET_SONAR: // SONAR
            {
                mvprintw(8, 0, "SONAR  : L0=%8d| C0=%8d| R0=%8d|",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2]);
                break;
            }
            case nxt::com::protocol::Command::GET_COLOR: // COLOR
            {
                mvprintw(9, 0, "COLOR  : R0=%8d| G0=%8d| B0=%8d|",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2]);
                break;
            }
            case nxt::com::protocol::Command::GET_LIGHT: // LIGHT
            {
                mvprintw(9, 0, "COLOR  : A0=%8d|", nxt_pkg_rx.data[0]);
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

    LOG_INFO("... connected (quit wit <ESC>)");

    std::this_thread::sleep_for(std::chrono::seconds(1));

    initscr();
    keypad(stdscr, TRUE);
    noecho();

    mvprintw(0, 0, "CMD:");
    mvprintw(0, 5, "<  >");
    refresh();

    std::thread rt(&receive);
    rt.detach();

    int ch;

    while ((ch = getch()) != 27)
    {
        switch (ch)
        {
        case KEY_F(0):
        case KEY_F(1):
        case 'f':
        case 'r':
        {
            std::int32_t no = 0;

            auto ch2 = getch();

            switch (ch2)
            {
            case '1':
                no = 1;
                break;
            case '2':
                no = 2;
                break;
            case '3':
                no = 3;
                break;
            default:
                continue;
            }

            switch (ch)
            {
            case KEY_F(0):
            case 'f':
                mvprintw(0, 5, "<f%d>", no);
                send(nxt::com::protocol::Command::MOTOR_FWD, {no, 50});
                break;
            case KEY_F(1):
            case 'r':
                mvprintw(0, 5, "<r%d>", no);
                send(nxt::com::protocol::Command::MOTOR_REV, {no, 50});
                break;
            }
        }
        break;
        case KEY_BACKSPACE:
        case '!':
        {
            std::int32_t no = 0;

            auto ch2 = getch();

            switch (ch2)
            {
            case '1':
                no = 1;
                break;
            case '2':
                no = 2;
                break;
            case '3':
                no = 3;
                break;
            default:
                continue;
            }

            mvprintw(0, 5, "<!%d>", no);
            send(nxt::com::protocol::Command::MOTOR_STP, {no});
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
