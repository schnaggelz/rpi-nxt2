/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT remote control library, Python3 binding.
 *
 * License notes see LICENSE.txt
 ******************************************************************************/

#include "nxt/remote/remote.hpp"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(nxt_remote_py, m)
{
    m.doc() = "Mindstorms NXT remote control library";

    py::class_<nxt::remote::Remote>(m, "Remote")
        .def(py::init<>())
        .def("connect", &nxt::remote::Remote::connect)
        .def("disconnect", &nxt::remote::Remote::disconnect)
        .def("connected", &nxt::remote::Remote::isConnected)
        .def("poll", &nxt::remote::Remote::poll)
        .def("shutdown", &nxt::remote::Remote::shutdown)
        .def("sensor_rcv", &nxt::remote::Remote::sensorRcv)
        .def("system_rcv", &nxt::remote::Remote::systemRcv)
        .def("motor_fwd", &nxt::remote::Remote::motorFwd)
        .def("motor_rev", &nxt::remote::Remote::motorRev)
        .def("motor_stop", &nxt::remote::Remote::motorStop);

    py::enum_<nxt::remote::Remote::Port>(m, "Port")
        .value("PORT_1", nxt::remote::Remote::Port::PORT_1)
        .value("PORT_2", nxt::remote::Remote::Port::PORT_2)
        .value("PORT_3", nxt::remote::Remote::Port::PORT_3)
        .value("PORT_4", nxt::remote::Remote::Port::PORT_4)
        .value("PORT_A", nxt::remote::Remote::Port::PORT_A)
        .value("PORT_B", nxt::remote::Remote::Port::PORT_B)
        .value("PORT_C", nxt::remote::Remote::Port::PORT_C)
        .export_values();

    py::enum_<nxt::remote::Remote::Info>(m, "Info")
        .value("BATTERY_VOLTAGE", nxt::remote::Remote::Info::BATTERY_VOLTAGE)
        .export_values();
}
