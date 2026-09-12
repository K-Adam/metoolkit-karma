#include "MdtImpl.h"

namespace {

void rows(MdtConstraintID, MdtJointRows& r)
{
    r.lockPoint();
}

const MdtConstraintOps ops = mdtJointOps<MdtBSJoint>(rows);

} // namespace

extern "C" {

MdtBSJointID MEAPI MdtBSJointCreate(const MdtWorldID w)
{
    MdtBSJointID j = mdtJointCreate<MdtBSJoint>(w, MdtBclBSJOINT, ops, MdtBclGETMAXROWSBSJOINT);
    MdtBSJointReset(j);
    return j;
}

void MEAPI MdtBSJointReset(const MdtBSJointID j)
{
    mdtConstraintResetFrames(j->head);
}

MdtConstraintID MEAPI MdtBSJointQuaConstraint(const MdtBSJointID j)
{
    return mdtQua(j);
}

MdtBSJointID MEAPI MdtConstraintDCastBSJoint(const MdtConstraintID c)
{
    return mdtDCast<MdtBSJoint>(c, MdtBclBSJOINT);
}

} // extern "C"
