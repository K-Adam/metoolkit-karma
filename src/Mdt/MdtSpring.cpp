#include "MdtImpl.h"

namespace {

// Karma's natural length is added to the separation of the first step, so the limited position is the
// change in distance between the attachment points since then. Karma's angular Jacobian takes the unit
// direction scaled by the current separation, so an off-centre spring turns its bodies that many times
// harder, as observed in the original.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtSpringID j = MdtConstraintDCastSpring(c);
    btVector3 pa = r.fa.getOrigin(), pb = r.fb.getOrigin(), d = pa - pb;
    btScalar length = d.length();
    btVector3 n = length > SIMD_EPSILON ? d / length : r.axisA(0);
    MdtBclLimit& l = j->limit;
    if (!l.bPositionInitialised)
        l.offset -= length;
    MdtRow dof = r.linear(n, pa, pb);
    dof.angA *= length;
    dof.angB *= length;
    mdtLimitRows(r, l, dof, length);
}

const MdtConstraintOps ops = mdtJointOps<MdtSpring>(rows);

} // namespace

extern "C" {

MdtSpringID MEAPI MdtSpringCreate(const MdtWorldID w)
{
    MdtSpringID j = mdtJointCreate<MdtSpring>(w, MdtBclSPRING, ops, MdtBclGETMAXROWSSPRING);
    MdtSpringReset(j);
    return j;
}

void MEAPI MdtSpringReset(const MdtSpringID j)
{
    mdtConstraintResetFrames(j->head);
    j->pos1[0] = j->pos1[1] = j->pos1[2] = 0;
    j->pos2[0] = j->pos2[1] = j->pos2[2] = 0;
    MdtLimitReset(&j->limit);
    MdtLimitActivateLimits(&j->limit, 1);
    j->limit.limit[0].stiffness = j->limit.limit[1].stiffness = 1;
}

MdtConstraintID MEAPI MdtSpringQuaConstraint(const MdtSpringID j)
{
    return mdtQua(j);
}

MdtSpringID MEAPI MdtConstraintDCastSpring(const MdtConstraintID c)
{
    return mdtDCast<MdtSpring>(c, MdtBclSPRING);
}

void MEAPI MdtSpringGetPosition(const MdtSpringID j, MeVector3 v, const unsigned int bodyindex)
{
    MdtConstraintBodyGetPosition(mdtQua(j), bodyindex, v);
}

MdtLimitID MEAPI MdtSpringGetLimit(const MdtSpringID j)
{
    return &j->limit;
}

void MEAPI MdtSpringSetPosition(const MdtSpringID j, const unsigned int bodyindex, const MeReal x, const MeReal y,
                                const MeReal z)
{
    MdtConstraintID c = mdtQua(j);
    MdtConstraintBodySetPosition(c, bodyindex, x, y, z);
    MdtConstraintBodyGetPositionRel(c, bodyindex, bodyindex ? j->pos2 : j->pos1);
}

void MEAPI MdtSpringSetLimit(const MdtSpringID j, const MdtLimitID NewLimit)
{
    j->limit = *NewLimit;
}

void MEAPI MdtSpringSetNaturalLength(const MdtSpringID j, const MeReal NewNaturalLength)
{
    j->limit.limit[0].stop = j->limit.limit[1].stop = NewNaturalLength;
}

void MEAPI MdtSpringSetStiffness(const MdtSpringID j, const MeReal NewStiffness)
{
    MdtSingleLimitSetStiffness(&j->limit.limit[0], NewStiffness);
    MdtSingleLimitSetStiffness(&j->limit.limit[1], NewStiffness);
}

void MEAPI MdtSpringSetDamping(const MdtSpringID j, const MeReal NewDamping)
{
    MdtSingleLimitSetDamping(&j->limit.limit[0], NewDamping);
    MdtSingleLimitSetDamping(&j->limit.limit[1], NewDamping);
}

} // extern "C"
