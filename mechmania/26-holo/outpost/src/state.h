#ifndef CAMLEAF_COPROC
#define CAMLEAF_COPROC

#include <cstdint>
struct  [[gnu::packed]] RobotState{
    uint16_t buttons = 0;
};

union StateAssign{
    uint16_t raw = 0; // make match robotState
    RobotState state;
};

extern RobotState rState;
extern StateAssign rStateAssign;

#endif
