/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C++ driver API
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "nxt/nxt_ht_ir_seeker.hpp"

#include "drivers/nxt_ht_ir_seeker.h"

namespace nxt
{
namespace fw
{
namespace ht
{
void InfraredSeeker::init()
{
    nxt_ht_ir_seeker_init(_port_number);
}

void InfraredSeeker::exit()
{
    nxt_ht_ir_seeker_term(_port_number);
}

void InfraredSeeker::read()
{
    nxt_ht_ir_seeker_get_data(_port_number, _ir_data.data());
}

} // namespace ht
} // namespace fw
} // namespace nxt
