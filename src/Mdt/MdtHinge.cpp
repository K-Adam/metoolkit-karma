#include "MdtImpl.h"

namespace {

// The hinge angle is the rotation of body 1 relative to body 2 about the hinge axis, measured between the
// orthogonal axes of the two constraint frames.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtHingeID j = MdtConstraintDCastHinge(c);
    r.lockPoint();
    r.lockAxis();
    btVector3 a = r.axisA(0), ya = r.axisA(1), yb = r.axisB(1);
    btScalar angle = btAtan2(yb.cross(ya).dot(a), yb.dot(ya));
    mdtLimitRows(r, j->limit, r.angular(a), mdtUnwrap(angle, mdtLimitPrevious(j->limit, angle)));
}

const MdtConstraintOps ops = mdtJointOps<MdtHinge>(rows);

} // namespace

extern "C" {

MdtHingeID MEAPI MdtHingeCreate(const MdtWorldID w)
{
    MdtHingeID j = mdtJointCreate<MdtHinge>(w, MdtBclHINGE, ops, MdtBclGETMAXROWSHINGE);
    MdtHingeReset(j);
    return j;
}

void MEAPI MdtHingeReset(const MdtHingeID j)
{
    mdtConstraintResetFrames(j->head);
    MdtLimitReset(&j->limit);
}

MdtConstraintID MEAPI MdtHingeQuaConstraint(const MdtHingeID j)
{
    return mdtQua(j);
}

MdtHingeID MEAPI MdtConstraintDCastHinge(const MdtConstraintID c)
{
    return mdtDCast<MdtHinge>(c, MdtBclHINGE);
}

MdtLimitID MEAPI MdtHingeGetLimit(const MdtHingeID j)
{
    return &j->limit;
}

void MEAPI MdtHingeSetLimit(const MdtHingeID j, const MdtLimitID NewLimit)
{
    j->limit = *NewLimit;
}

} // extern "C"
