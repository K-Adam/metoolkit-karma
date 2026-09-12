#include "MdtImpl.h"

namespace {

// Body 1's attachment point stays on the line through body 2's attachment along body 2's primary axis.
void rows(MdtConstraintID, MdtJointRows& r)
{
    btVector3 pa = r.fa.getOrigin(), d = pa - r.fb.getOrigin();
    for (int k = 1; k < 3; ++k)
        r.lock(r.linear(r.axisB(k), pa, pa), d.dot(r.axisB(k)));
}

const MdtConstraintOps ops = mdtJointOps<MdtLinear2>(rows);

} // namespace

extern "C" {

MdtLinear2ID MEAPI MdtLinear2Create(const MdtWorldID w)
{
    MdtLinear2ID j = mdtJointCreate<MdtLinear2>(w, MdtBclLINEAR2, ops, MdtBclGETMAXROWSLINEAR2);
    MdtLinear2Reset(j);
    return j;
}

void MEAPI MdtLinear2Reset(const MdtLinear2ID j)
{
    mdtConstraintResetFrames(j->head);
    MeReal* v[] = {j->pos1, j->pos2, j->direction, j->vec1, j->vec2};
    for (MeReal* p : v)
        p[0] = p[1] = p[2] = 0;
}

MdtConstraintID MEAPI MdtLinear2QuaConstraint(const MdtLinear2ID j)
{
    return mdtQua(j);
}

MdtLinear2ID MEAPI MdtConstraintDCastLinear2(const MdtConstraintID c)
{
    return mdtDCast<MdtLinear2>(c, MdtBclLINEAR2);
}

void MEAPI MdtLinear2GetPosition(const MdtLinear2ID j, MeVector3 position)
{
    MdtConstraintGetPosition(mdtQua(j), position);
}

void MEAPI MdtLinear2GetDirection(const MdtLinear2ID j, MeVector3 direction)
{
    MdtConstraintBodyGetAxis(mdtQua(j), 1, direction);
}

void MEAPI MdtLinear2SetPosition(const MdtLinear2ID j, const MeReal x, const MeReal y, const MeReal z)
{
    MdtConstraintID c = mdtQua(j);
    MdtConstraintSetPosition(c, x, y, z);
    MdtConstraintBodyGetPositionRel(c, 0, j->pos1);
    MdtConstraintBodyGetPositionRel(c, 1, j->pos2);
}

void MEAPI MdtLinear2SetDirection(const MdtLinear2ID j, const MeReal x, const MeReal y, const MeReal z)
{
    MdtConstraintID c = mdtQua(j);
    MdtConstraintSetAxis(c, x, y, z);
    btMatrix3x3 m = mdtRef(c, 1).getBasis();
    fromBt(m.getColumn(0), j->direction);
    fromBt(m.getColumn(2), j->vec1);
    fromBt(-m.getColumn(1), j->vec2);
}

} // extern "C"
