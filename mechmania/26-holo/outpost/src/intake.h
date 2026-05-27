#ifndef CAMLEAF_INTAKE
#define CAMLEAF_INTAKE

#include <cstdint>

class Intake{
    public:
        //   bottom roller motor outs | upper roller motor outs    // beam break
        Intake(uint8_t kb1, uint8_t kb2, uint8_t ku1, uint8_t ku2, uint8_t kbm);
        
        void enabled(bool en);
        void reversed(bool en); // for outtaking if needbe
        void stageInterrupt(); // auto lock on beam interrupt between intake stages. 

    private:
        uint8_t kb1 = 0;
        uint8_t kb2 = 0;
        uint8_t ku1 = 0;
        uint8_t ku2 = 0;
        uint8_t kbm = 0;
};


#endif
