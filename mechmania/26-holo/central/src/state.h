#ifndef CAMLEAF_COPROC
#define CAMLEAF_COPROC

#include <cstdint>
struct  [[gnu::packed]] RobotState{
    uint16_t buttons = 0;
    uint8_t dpad = 0;
    uint8_t empty = 0;
    RobotState() {}
};

union StateAssign{
    uint32_t raw = 0; // make size match robotState
    RobotState state;
    StateAssign() {}
};

extern RobotState* rState;
extern StateAssign rStateAssign;

#endif
