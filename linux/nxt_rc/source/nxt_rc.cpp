
#include <ncurses.h>

#include <chrono>
#include <iostream>
#include <thread>

//#include "simple_logger.hpp"

#include "nxt_usb/usb_device.hpp"

bool exiting = false;

nxt_com::USBDevice nxt_usb_dev;
nxt_com::DataPacket nxt_pkg_tx;
nxt_com::DataPacket nxt_pkg_rx;

using namespace std;

bool connect()
{
    bool success = true;

    if (success = nxt_usb_dev.init())
    {
        success = nxt_usb_dev.open();
    }

    return success;
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

    mvprintw(2, 0, "RCV: %5d", 0);
    mvprintw(3, 0, "ID:  %5d, SIZE: %5d", 0, 0);
    mvprintw(4, 0, "GENERIC: V0=%5d V1=%5d V2=%5d V3=%5d", 0, 0, 0, 0);
    mvprintw(5, 0, "GENERIC: V4=%5d V5=%5d V6=%5d V7=%5d", 0, 0, 0, 0);
    mvprintw(6, 0, "SONAR: L=%4d C=%4d R=%4d", 0, 0, 0);
    mvprintw(7, 0, "COLOR: R=%4d G=%4d B=%4d", 0, 0, 0);

    while (!exiting)
    {
        if (nxt_usb_dev.isReady())
        {
            nxt_usb_dev.read(nxt_pkg_rx);

            mvprintw(2, 0, "RCV: %5d", counter++);
            mvprintw(3, 0, "ID:  %5d, SIZE: %5d", nxt_pkg_rx.id,
                     nxt_pkg_rx.size);

            switch (nxt_pkg_rx.id)
            {
            case 0x00: // GENERIC
            {
                mvprintw(4, 0, "GENERIC: V0=%4d;V1=%4d;V2=%4d;V3=%4d",
                         nxt_pkg_rx.data[0], nxt_pkg_rx.data[1],
                         nxt_pkg_rx.data[2], nxt_pkg_rx.data[3]);
                mvprintw(5, 0, "GENERIC: V4=%4d;V5=%4d;V6=%5d;V7=4d",
                         nxt_pkg_rx.data[4], nxt_pkg_rx.data[5],
                         nxt_pkg_rx.data[6], nxt_pkg_rx.data[7]);
                break;
            }
            case 0x10: // SONAR
            {
                mvprintw(6, 0, "SONAR: L=%4d;C=%4d;R=%4d", nxt_pkg_rx.data[0],
                         nxt_pkg_rx.data[1], nxt_pkg_rx.data[2]);
                break;
            }
            case 0x11: // COLOR
            {
                mvprintw(7, 0, "COLOR: R=%4d;G=%4d;B=%4d", nxt_pkg_rx.data[0],
                         nxt_pkg_rx.data[1], nxt_pkg_rx.data[2]);
                break;
            }
            }

            refresh();
        }

        this_thread::sleep_for(chrono::milliseconds(50));
    }
}

void disconnect()
{
    nxt_usb_dev.close();
}

int main()
{
    //LOG_INFO("Connect to NXT ...");

    if (!connect())
    {
        //LOG_ERROR("Creating NXT connection failed");
        return 1;
    }

    initscr();
    keypad(stdscr, TRUE);
    noecho();

    mvprintw(0, 0, "CMD:");
    mvprintw(0, 5, "< >");
    refresh();

    thread rt(&receive);
    rt.detach();

    int ch;

    while ((ch = getch()) != '#')
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

    //LOG_INFO("Disconnected from NXT");

    return 0;
}
