#include "nxt_utils.hpp"

#include "nxt_ht_ir_seeker.hpp"

#include "nxt_ht_ir_seeker.h"

using namespace nxt::ht;

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
