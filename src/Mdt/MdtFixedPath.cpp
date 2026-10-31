#include <algorithm>

#include <MeMath.h>

#include "MdtImpl.h"

namespace {

btVector3 bodyRotated(const btTransform& frame, MdtConstraintID c, unsigned i, const MeReal* v)
{
    return frame.getBasis() * mdtRef(c, i).getBasis().transpose() * toBt(v);
}

// A ball and socket whose attachment points move along their bodies at the given velocities, so the solver
// feeds the path's motion into the bodies instead of relaxing a moved joint.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtFixedPathID j = MdtConstraintDCastFixedPath(c);
    int first = r.n;
    r.lockPoint();
    btVector3 path = bodyRotated(r.fa, c, 0, j->vel1) - bodyRotated(r.fb, c, 1, j->vel2);
    for (int k = first; k < r.n; ++k)
        r.rows[k].rhs -= path.dot(r.rows[k].linA);
}

const MdtConstraintOps ops = mdtJointOps<MdtFixedPath>(rows);

} // namespace

extern "C" {

MdtFixedPathID MEAPI MdtFixedPathCreate(const MdtWorldID w)
{
    MdtFixedPathID j = mdtJointCreate<MdtFixedPath>(w, MdtBclFIXEDPATH, ops, MdtBclGETMAXROWSFIXEDPATH);
    MdtFixedPathReset(j);
    return j;
}

void MEAPI MdtFixedPathReset(const MdtFixedPathID j)
{
    mdtConstraintResetFrames(j->head);
    MeReal* v[] = {j->pos1, j->pos2, j->vel1, j->vel2};
    for (MeReal* p : v)
        p[0] = p[1] = p[2] = 0;
}

MdtConstraintID MEAPI MdtFixedPathQuaConstraint(const MdtFixedPathID j)
{
    return mdtQua(j);
}

MdtFixedPathID MEAPI MdtConstraintDCastFixedPath(const MdtConstraintID c)
{
    return mdtDCast<MdtFixedPath>(c, MdtBclFIXEDPATH);
}

void MEAPI MdtFixedPathGetPosition(const MdtFixedPathID j, const unsigned int bodyindex, MeVector3 position)
{
    MdtConstraintBodyGetPositionRel(mdtQua(j), bodyindex, position);
}

void MEAPI MdtFixedPathGetVelocity(const MdtFixedPathID j, const unsigned int bodyindex, MeVector3 velocity)
{
    std::copy_n(bodyindex ? j->vel2 : j->vel1, 3, velocity);
}

void MEAPI MdtFixedPathSetPosition(const MdtFixedPathID j, const unsigned int bodyindex, const MeReal x,
                                   const MeReal y, const MeReal z)
{
    MdtConstraintBodySetPositionRel(mdtQua(j), bodyindex, x, y, z);
    MeVector3Set(bodyindex ? j->pos2 : j->pos1, x, y, z);
}

void MEAPI MdtFixedPathSetVelocity(const MdtFixedPathID j, const unsigned int bodyindex, const MeReal dx,
                                   const MeReal dy, const MeReal dz)
{
    MeVector3Set(bodyindex ? j->vel2 : j->vel1, dx, dy, dz);
}

} // extern "C"
