#ifndef CAMLEAF_COPROC
#define CAMLEAF_COPROC

#include <cstdint>
struct  [[gnu::packed]] RobotState{
    uint16_t buttons = 0;
    uint8_t dpad = 0;
    int getInt(){
      return buttons << 16 | dpad;
    }

    void unload(int raw){
      dpad = raw | ((1UL<<8) - 1);
      raw >>= 8;
      buttons = raw | ((1UL<<16)-1);
    }
};

extern RobotState* rState;

#endif
