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
        .def("disconnect", &nxt::remote::Remote::disconnect);
}
