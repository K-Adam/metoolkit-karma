#include "MdtImpl.h"

namespace {

// Adds a row only while the angle between the two primary axes exceeds the cone half angle.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtConeLimitID j = MdtConstraintDCastConeLimit(c);
    btVector3 a = r.axisA(0), b = r.axisB(0), u = b.cross(a);
    btScalar angle = btAcos(btClamped(a.dot(b), btScalar(-1), btScalar(1)));
    if (angle < j->cone_half_angle || u.length2() < SIMD_EPSILON)
        return;
    if (MdtRow* row = r.lock(r.angular(u.normalized()), angle - j->cone_half_angle, j->stiffness, j->damping))
        r.oneSided(*row, true);
}

const MdtConstraintOps ops = mdtJointOps<MdtConeLimit>(rows);

} // namespace

extern "C" {

MdtConeLimitID MEAPI MdtConeLimitCreate(const MdtWorldID w)
{
    MdtConeLimitID j = mdtJointCreate<MdtConeLimit>(w, MdtBclCONELIMIT, ops, MdtBclGETMAXROWSCONELIMIT);
    MdtConeLimitReset(j);
    return j;
}

void MEAPI MdtConeLimitReset(const MdtConeLimitID j)
{
    mdtConstraintResetFrames(j->head);
    MdtConeLimitSetConeHalfAngle(j, ME_PI);
    j->stiffness = MEINFINITY;
    j->damping = MEINFINITY;
}

MdtConstraintID MEAPI MdtConeLimitQuaConstraint(const MdtConeLimitID j)
{
    return mdtQua(j);
}

MdtConeLimitID MEAPI MdtConstraintDCastConeLimit(const MdtConstraintID c)
{
    return mdtDCast<MdtConeLimit>(c, MdtBclCONELIMIT);
}

MeReal MEAPI MdtConeLimitGetConeHalfAngle(const MdtConeLimitID j)
{
    return j->cone_half_angle;
}

MeReal MEAPI MdtConeLimitGetStiffness(const MdtConeLimitID j)
{
    return j->stiffness;
}

MeReal MEAPI MdtConeLimitGetDamping(const MdtConeLimitID j)
{
    return j->damping;
}

void MEAPI MdtConeLimitSetConeHalfAngle(const MdtConeLimitID j, const MeReal theta)
{
    j->cone_half_angle = theta;
    j->cos_cone_half_angle = btCos(theta);
}

void MEAPI MdtConeLimitSetStiffness(const MdtConeLimitID j, const MeReal kp)
{
    j->stiffness = kp;
}

void MEAPI MdtConeLimitSetDamping(const MdtConeLimitID j, const MeReal kd)
{
    j->damping = kd;
}

} // extern "C"
