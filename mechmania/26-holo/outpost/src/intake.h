#ifndef CAMLEAF_INTAKE
#define CAMLEAF_INTAKE

#include <cstdint>
class Intake{
    public:
        //     bottom roller motor outs | upper roller motor outs
        Intake(uint8_t kb1, uint8_t kb2, uint8_t ku1, uint8_t ku2);
};


#endif
