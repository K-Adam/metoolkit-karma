#include "MdtImpl.h"

namespace {

// Body 1's primary axis stays perpendicular to body 2's orthogonal axis.
void rows(MdtConstraintID, MdtJointRows& r)
{
    r.lockPoint();
    btVector3 a = r.axisA(0), b = r.axisB(1);
    r.lock(r.angular(a.cross(b)), a.dot(b));
}

const MdtConstraintOps ops = mdtJointOps<MdtUniversal>(rows);

} // namespace

extern "C" {

MdtUniversalID MEAPI MdtUniversalCreate(const MdtWorldID w)
{
    MdtUniversalID j = mdtJointCreate<MdtUniversal>(w, MdtBclUNIVERSAL, ops, MdtBclGETMAXROWSUNIVERSAL);
    MdtUniversalReset(j);
    return j;
}

void MEAPI MdtUniversalReset(const MdtUniversalID j)
{
    mdtConstraintResetFrames(j->head);
}

MdtConstraintID MEAPI MdtUniversalQuaConstraint(const MdtUniversalID j)
{
    return mdtQua(j);
}

MdtUniversalID MEAPI MdtConstraintDCastUniversal(const MdtConstraintID c)
{
    return mdtDCast<MdtUniversal>(c, MdtBclUNIVERSAL);
}

} // extern "C"
