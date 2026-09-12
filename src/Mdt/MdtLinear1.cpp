#include "MdtImpl.h"

namespace {

// Body 1's attachment point stays in the plane through body 2's attachment, normal to body 2's primary axis.
void rows(MdtConstraintID, MdtJointRows& r)
{
    btVector3 n = r.axisB(0), pa = r.fa.getOrigin();
    r.lock(r.linear(n, pa, pa), (pa - r.fb.getOrigin()).dot(n));
}

const MdtConstraintOps ops = mdtJointOps<MdtLinear1>(rows);

} // namespace

extern "C" {

MdtLinear1ID MEAPI MdtLinear1Create(const MdtWorldID w)
{
    MdtLinear1ID j = mdtJointCreate<MdtLinear1>(w, MdtBclLINEAR1, ops, MdtBclGETMAXROWSLINEAR1);
    MdtLinear1Reset(j);
    return j;
}

void MEAPI MdtLinear1Reset(const MdtLinear1ID j)
{
    mdtConstraintResetFrames(j->head);
    j->pos1[0] = j->pos1[1] = j->pos1[2] = 0;
    j->pos2[0] = j->pos2[1] = j->pos2[2] = 0;
    j->displacement = 0;
}

MdtConstraintID MEAPI MdtLinear1QuaConstraint(const MdtLinear1ID j)
{
    return mdtQua(j);
}

MdtLinear1ID MEAPI MdtConstraintDCastLinear1(const MdtConstraintID c)
{
    return mdtDCast<MdtLinear1>(c, MdtBclLINEAR1);
}

void MEAPI MdtLinear1GetPosition(const MdtLinear1ID j, MeVector3 position)
{
    MdtConstraintGetPosition(mdtQua(j), position);
}

void MEAPI MdtLinear1SetPosition(const MdtLinear1ID j, const MeReal x, const MeReal y, const MeReal z)
{
    MdtConstraintID c = mdtQua(j);
    MdtConstraintSetPosition(c, x, y, z);
    MdtConstraintBodyGetPositionRel(c, 0, j->pos1);
    MdtConstraintBodyGetPositionRel(c, 1, j->pos2);
    btVector3 n = mdtFrameWorld(c, 1).getBasis().getColumn(0);
    btVector3 d = mdtBodyTransform(j->head.mdtbody[1]).getOrigin() - mdtBodyTransform(j->head.mdtbody[0]).getOrigin();
    j->displacement = d.dot(n);
}

} // extern "C"
