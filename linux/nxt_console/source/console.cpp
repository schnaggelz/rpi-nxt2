/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT communication tool.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "simple_logger/logger.hpp"

#include "nxt/utils/conversion.hpp"

#include "nxt/remote/remote.hpp"

#include <ncurses.h>

#include <atomic>
#include <chrono>
#include <thread>

std::atomic<bool> exiting = false;

nxt::remote::Remote remote;

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
        if (remote.isConnected())
        {
            remote.poll();

            mvprintw(3, 0, "RCV    : SQ:%8d|", counter++);

            mvprintw(6, 0, "GENERIC: V0=%8d| V1=%8d| V2=%8d| V3=%8d|",
                     remote.sensorRcv(nxt::com::protocol::Port::PORT_1),
                     remote.sensorRcv(nxt::com::protocol::Port::PORT_2),
                     remote.sensorRcv(nxt::com::protocol::Port::PORT_3),
                     remote.sensorRcv(nxt::com::protocol::Port::PORT_4));

            counter++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main()
{
    LOG_INFO("Connect to NXT ...");

    if (!remote.connect())
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
            auto port = nxt::com::protocol::Port::NONE;

            auto ch2 = getch();

            switch (ch2)
            {
            case '1':
                port = nxt::com::protocol::Port::PORT_A;
                break;
            case '2':
                port = nxt::com::protocol::Port::PORT_B;
                break;
            case '3':
                port = nxt::com::protocol::Port::PORT_C;
                break;
            default:
                continue;
            }

            switch (ch)
            {
            case KEY_F(0):
            case 'f':
                mvprintw(0, 5, "<f%d>", nxt::utils::to_underlying(port));
                remote.motorFwd(port, 50);
                break;
            case KEY_F(1):
            case 'r':
                mvprintw(0, 5, "<r%d>", nxt::utils::to_underlying(port));
                remote.motorRev(port, 50);
                break;
            }
        }
        break;
        case KEY_BACKSPACE:
        case '!':
        {
            auto port = nxt::com::protocol::Port::NONE;

            auto ch2 = getch();

            switch (ch2)
            {
            case '1':
                port = nxt::com::protocol::Port::PORT_A;
                break;
            case '2':
                port = nxt::com::protocol::Port::PORT_B;
                break;
            case '3':
                port = nxt::com::protocol::Port::PORT_C;
                break;
            default:
                continue;
            }

            mvprintw(0, 5, "<!%d>", nxt::utils::to_underlying(port));
            remote.motorStop(port);
        }
        break;
        }

        refresh();
    }

    exiting = true;

    endwin();

    remote.disconnect();

    LOG_INFO("Disconnected from NXT");

    return 0;
}
