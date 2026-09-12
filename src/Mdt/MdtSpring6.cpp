#include <algorithm>

#include "MdtImpl.h"

// The header also defines this name as a macro over MdtConstraintGetPosition.
#undef MdtSpring6GetPosition

namespace {

void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtSpring6ID j = MdtConstraintDCastSpring6(c);
    btVector3 pa = r.fa.getOrigin(), d = pa - r.fb.getOrigin();
    btVector3 e = mdtRotationError(r.fa.getBasis(), r.fb.getBasis());
    for (int k = 0; k < 3; ++k)
    {
        btVector3 u = r.axisA(k);
        r.lock(r.linear(u, r.fb.getOrigin(), r.fb.getOrigin()), d.dot(u), j->linearStiffness[k], j->linearDamping[k]);
        r.lock(r.angular(u), e.dot(u), j->angularStiffness[k], j->angularDamping[k]);
    }
}

const MdtConstraintOps ops = mdtJointOps<MdtSpring6>(rows);

bool valid(int axis) { return axis >= 0 && axis < 3; }

} // namespace

extern "C" {

MdtSpring6ID MEAPI MdtSpring6Create(const MdtWorldID w)
{
    MdtSpring6ID j = mdtJointCreate<MdtSpring6>(w, MdtBclSPRING6, ops, MdtBclGETMAXROWSSPRING6);
    MdtSpring6Reset(j);
    return j;
}

void MEAPI MdtSpring6Reset(const MdtSpring6ID j)
{
    mdtConstraintResetFrames(j->head);
    for (int k = 0; k < 3; ++k)
    {
        j->linearStiffness[k] = j->angularStiffness[k] = MEINFINITY;
        j->linearDamping[k] = j->angularDamping[k] = 0;
        j->worldLinearVel[k] = j->worldAngularVel[k] = 0;
    }
    MdtConstraintSetWorldLinearVelocity(mdtQua(j), 0, 0, 0);
    MdtConstraintSetWorldAngularVelocity(mdtQua(j), 0, 0, 0);
}

MdtConstraintID MEAPI MdtSpring6QuaConstraint(const MdtSpring6ID j)
{
    return mdtQua(j);
}

MdtSpring6ID MEAPI MdtConstraintDCastSpring6(const MdtConstraintID c)
{
    return mdtDCast<MdtSpring6>(c, MdtBclSPRING6);
}

void MEAPI MdtSpring6GetPosition(const MdtSpring6ID j, MeVector3 v)
{
    MdtConstraintGetPosition(mdtQua(j), v);
}

MeReal MEAPI MdtSpring6GetLinearStiffness(const MdtSpring6ID j, int axisindex)
{
    return valid(axisindex) ? j->linearStiffness[axisindex] : 0;
}

MeReal MEAPI MdtSpring6GetLinearDamping(const MdtSpring6ID j, int axisindex)
{
    return valid(axisindex) ? j->linearDamping[axisindex] : 0;
}

MeReal MEAPI MdtSpring6GetAngularStiffness(const MdtSpring6ID j, int axisindex)
{
    return valid(axisindex) ? j->angularStiffness[axisindex] : 0;
}

MeReal MEAPI MdtSpring6GetAngularDamping(const MdtSpring6ID j, int axisindex)
{
    return valid(axisindex) ? j->angularDamping[axisindex] : 0;
}

void MEAPI MdtSpring6GetWorldLinearVelocity(const MdtSpring6ID j, MeVector3 v)
{
    std::copy_n(j->worldLinearVel, 3, v);
}

void MEAPI MdtSpring6GetWorldAngularVelocity(const MdtSpring6ID j, MeVector3 v)
{
    std::copy_n(j->worldAngularVel, 3, v);
}

void MEAPI MdtSpring6SetLinearStiffness(const MdtSpring6ID j, int axisindex, MeReal s)
{
    if (valid(axisindex))
        j->linearStiffness[axisindex] = s;
}

void MEAPI MdtSpring6SetLinearDamping(const MdtSpring6ID j, int axisindex, MeReal d)
{
    if (valid(axisindex))
        j->linearDamping[axisindex] = d;
}

void MEAPI MdtSpring6SetAngularStiffness(const MdtSpring6ID j, int axisindex, MeReal s)
{
    if (valid(axisindex))
        j->angularStiffness[axisindex] = s;
}

void MEAPI MdtSpring6SetAngularDamping(const MdtSpring6ID j, int axisindex, MeReal d)
{
    if (valid(axisindex))
        j->angularDamping[axisindex] = d;
}

void MEAPI MdtSpring6SetWorldLinearVelocity(const MdtSpring6ID j, MeReal vx, MeReal vy, MeReal vz)
{
    fromBt(btVector3(vx, vy, vz), j->worldLinearVel);
    MdtConstraintSetWorldLinearVelocity(mdtQua(j), vx, vy, vz);
}

void MEAPI MdtSpring6SetWorldAngularVelocity(const MdtSpring6ID j, MeReal vx, MeReal vy, MeReal vz)
{
    fromBt(btVector3(vx, vy, vz), j->worldAngularVel);
    MdtConstraintSetWorldAngularVelocity(mdtQua(j), vx, vy, vz);
}

} // extern "C"
