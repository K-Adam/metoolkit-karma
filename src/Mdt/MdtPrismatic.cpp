#include "MdtImpl.h"

namespace {

// The slide position is the displacement of body 1's attachment from body 2's along the axis.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtPrismaticID j = MdtConstraintDCastPrismatic(c);
    btVector3 pa = r.fa.getOrigin(), d = pa - r.fb.getOrigin();
    r.lockRotation();
    for (int k = 1; k < 3; ++k)
        r.lock(r.linear(r.axisA(k), pa, pa), d.dot(r.axisA(k)));
    mdtLimitRows(r, j->limit, r.linear(r.axisA(0), pa, pa), d.dot(r.axisA(0)));
}

const MdtConstraintOps ops = mdtJointOps<MdtPrismatic>(rows);

// Setting the axis also makes the bodies' current placement the zero of the slide.
void MEAPI setAxis(const MdtConstraintID c, const MeReal px, const MeReal py, const MeReal pz)
{
    btVector3 o = mdtOrthogonal(btVector3(px, py, pz));
    MdtConstraintSetAxes(c, px, py, pz, o.x(), o.y(), o.z());
    if (MdtBodyID b = c->head.mdtbody[0])
        MdtConstraintSetPosition(c, b->bodyTM[3][0], b->bodyTM[3][1], b->bodyTM[3][2]);
}

} // namespace

extern "C" {

MdtPrismaticID MEAPI MdtPrismaticCreate(const MdtWorldID w)
{
    MdtPrismaticID j = mdtJointCreate<MdtPrismatic>(w, MdtBclPRISMATIC, ops, MdtBclGETMAXROWSPRISMATIC);
    j->head.setAxisFunc = setAxis;
    MdtPrismaticReset(j);
    return j;
}

void MEAPI MdtPrismaticReset(const MdtPrismaticID j)
{
    mdtConstraintResetFrames(j->head);
    MdtLimitReset(&j->limit);
}

MdtConstraintID MEAPI MdtPrismaticQuaConstraint(const MdtPrismaticID j)
{
    return mdtQua(j);
}

MdtPrismaticID MEAPI MdtConstraintDCastPrismatic(const MdtConstraintID c)
{
    return mdtDCast<MdtPrismatic>(c, MdtBclPRISMATIC);
}

MdtLimitID MEAPI MdtPrismaticGetLimit(const MdtPrismaticID j)
{
    return &j->limit;
}

void MEAPI MdtPrismaticSetLimit(const MdtPrismaticID j, const MdtLimitID NewLimit)
{
    j->limit = *NewLimit;
}

} // extern "C"
