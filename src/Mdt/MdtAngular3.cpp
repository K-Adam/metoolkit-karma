#include "MdtImpl.h"

namespace {

void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtAngular3ID j = MdtConstraintDCastAngular3(c);
    if (j->bEnableRotation)
        r.lockAxis(j->stiffness, j->damping);
    else
        r.lockRotation(j->stiffness, j->damping);
}

const MdtConstraintOps ops = mdtJointOps<MdtAngular3>(rows);

// Attaching the bodies keeps their current relative orientation. Karma expresses both frames in the
// orientation given by body 1's transposed rotation.
void MEAPI setBodies(const MdtConstraintID c, const MdtBodyID b1, const MdtBodyID b2)
{
    btMatrix3x3 f = mdtBodyTransform(b1).getBasis().transpose();
    for (unsigned i = 0; i < 2; ++i)
    {
        btTransform r = mdtRef(c, i);
        r.setBasis(mdtBodyTransform(i ? b2 : b1).getBasis().transpose() * f);
        mdtSetRef(c, i, r);
    }
}

} // namespace

extern "C" {

MdtAngular3ID MEAPI MdtAngular3Create(const MdtWorldID w)
{
    MdtAngular3ID j = mdtJointCreate<MdtAngular3>(w, MdtBclANGULAR3, ops, MdtBclGETMAXROWSANGULAR3);
    j->head.setBodyFunc = setBodies;
    MdtAngular3Reset(j);
    return j;
}

void MEAPI MdtAngular3Reset(const MdtAngular3ID j)
{
    mdtConstraintResetFrames(j->head);
    j->bEnableRotation = 0;
    j->stiffness = MEINFINITY;
    j->damping = MEINFINITY;
}

MdtConstraintID MEAPI MdtAngular3QuaConstraint(const MdtAngular3ID j)
{
    return mdtQua(j);
}

MdtAngular3ID MEAPI MdtConstraintDCastAngular3(const MdtConstraintID c)
{
    return mdtDCast<MdtAngular3>(c, MdtBclANGULAR3);
}

MeBool MEAPI MdtAngular3RotationIsEnabled(const MdtAngular3ID j)
{
    return j->bEnableRotation;
}

MeReal MEAPI MdtAngular3GetStiffness(const MdtAngular3ID j)
{
    return j->stiffness;
}

MeReal MEAPI MdtAngular3GetDamping(const MdtAngular3ID j)
{
    return j->damping;
}

void MEAPI MdtAngular3EnableRotation(const MdtAngular3ID j, const MeBool NewRotationState)
{
    j->bEnableRotation = NewRotationState != 0;
}

void MEAPI MdtAngular3SetStiffness(const MdtAngular3ID j, const MeReal s)
{
    j->stiffness = s;
}

void MEAPI MdtAngular3SetDamping(const MdtAngular3ID j, const MeReal d)
{
    j->damping = d;
}

} // extern "C"
