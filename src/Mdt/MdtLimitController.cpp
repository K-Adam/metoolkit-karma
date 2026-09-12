#include <algorithm>

#include "MdtImpl.h"

extern "C" {

// A proportional speed controller: full speed outside the gap around the desired position, scaled down
// linearly inside it.
void MEAPI MdtLimitController(const MdtLimitID limit, const MeReal desiredPosition, const MeReal gap,
                              const MeReal maxSpeed, const MeReal maxForce)
{
    MeReal error = desiredPosition - MdtLimitGetPosition(limit);
    MeReal scale = gap > 0 ? std::clamp(error / gap, (MeReal)-1, (MeReal)1) : (error > 0) - (error < 0);
    MdtLimitSetLimitedForceMotor(limit, maxSpeed * scale, maxForce);
}

} // extern "C"
