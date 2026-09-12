#include <algorithm>

#include "MdtImpl.h"

namespace {

// The attachment frame on body 2 is driven to q_rel relative to the one on body 1, turning at omega relative
// to it, with each row's force capped by the joint's strengths.
void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtRPROJointID j = MdtConstraintDCastRPROJoint(c);
    btVector3 pa = r.fa.getOrigin(), pb = r.fb.getOrigin(), d = pa - pb;
    btMatrix3x3 q(toBtQuat(j->q_rel[0], j->q_rel[1], j->q_rel[2], j->q_rel[3]).normalized());
    btVector3 e = mdtRotationError(r.fa.getBasis(), r.fb.getBasis() * q.transpose());
    btVector3 omega = r.fa.getBasis() * toBt(j->omega);
    for (int k = 0; k < 3; ++k)
    {
        btVector3 u = r.axisA(k);
        MdtRow* lin = r.lock(r.linear(u, pb, pb), d.dot(u));
        lin->lo = -j->linear_fmax[k] * r.h;
        lin->hi = j->linear_fmax[k] * r.h;
        MdtRow* ang = r.lock(r.angular(u), e.dot(u));
        ang->rhs -= omega.dot(u);
        ang->lo = -j->angular_fmax[k] * r.h;
        ang->hi = j->angular_fmax[k] * r.h;
    }
}

const MdtConstraintOps ops = mdtJointOps<MdtRPROJoint>(rows);

} // namespace

extern "C" {

MdtRPROJointID MEAPI MdtRPROJointCreate(const MdtWorldID w)
{
    MdtRPROJointID j = mdtJointCreate<MdtRPROJoint>(w, MdtBclRPROJOINT, ops, MdtBclGETMAXROWSRPROJOINT);
    MdtRPROJointReset(j);
    return j;
}

void MEAPI MdtRPROJointReset(const MdtRPROJointID j)
{
    mdtConstraintResetFrames(j->head);
    const MeReal identity[4] = {1, 0, 0, 0};
    std::copy_n(identity, 4, j->q1);
    std::copy_n(identity, 4, j->q2);
    std::copy_n(identity, 4, j->q_rel);
    j->use_q1 = j->use_q2 = 0;
    for (int k = 0; k < 4; ++k)
        j->p_rel[k] = 0;
    for (int k = 0; k < 3; ++k)
    {
        j->v_rel[k] = j->omega[k] = 0;
        j->angular_fmax[k] = j->linear_fmax[k] = MEINFINITY;
    }
}

MdtConstraintID MEAPI MdtRPROJointQuaConstraint(const MdtRPROJointID j)
{
    return mdtQua(j);
}

MdtRPROJointID MEAPI MdtConstraintDCastRPROJoint(const MdtConstraintID c)
{
    return mdtDCast<MdtRPROJoint>(c, MdtBclRPROJOINT);
}

void MEAPI MdtRPROJointGetAttachmentPosition(const MdtRPROJointID j, const unsigned int bodyindex,
                                             MeVector3 position)
{
    MdtConstraintBodyGetPositionRel(mdtQua(j), bodyindex, position);
}

void MEAPI MdtRPROJointGetAttachmentQuaternion(const MdtRPROJointID j, const unsigned int bodyindex,
                                               MeVector4 quaternion)
{
    std::copy_n(bodyindex ? j->q2 : j->q1, 4, quaternion);
}

void MEAPI MdtRPROJointGetRelativeQuaternion(const MdtRPROJointID j, MeVector4 q)
{
    std::copy_n(j->q_rel, 4, q);
}

void MEAPI MdtRPROJointGetRelativeAngularVelocity(const MdtRPROJointID j, MeVector3 w)
{
    std::copy_n(j->omega, 3, w);
}

void MEAPI MdtRPROJointGetAngularStrength(const MdtRPROJointID j, MeVector3 v)
{
    std::copy_n(j->angular_fmax, 3, v);
}

void MEAPI MdtRPROJointGetLinearStrength(const MdtRPROJointID j, MeVector3 v)
{
    std::copy_n(j->linear_fmax, 3, v);
}

void MEAPI MdtRPROJointSetAttachmentPosition(const MdtRPROJointID j, const MeReal x, const MeReal y, const MeReal z,
                                             const unsigned int bodyindex)
{
    MdtConstraintBodySetPositionRel(mdtQua(j), bodyindex, x, y, z);
}

void MEAPI MdtRPROJointSetAttachmentQuaternion(const MdtRPROJointID j, const MeReal q0, const MeReal q1,
                                               const MeReal q2, const MeReal q3, const unsigned int bodyindex)
{
    const MeReal q[4] = {q0, q1, q2, q3};
    std::copy_n(q, 4, bodyindex ? j->q2 : j->q1);
    (bodyindex ? j->use_q2 : j->use_q1) = 1;
    btTransform r = mdtRef(mdtQua(j), bodyindex);
    r.setRotation(toBtQuat(q0, q1, q2, q3).normalized());
    mdtSetRef(mdtQua(j), bodyindex, r);
}

void MEAPI MdtRPROJointSetRelativeQuaternion(const MdtRPROJointID j, const MeVector4 q)
{
    std::copy_n(q, 4, j->q_rel);
}

void MEAPI MdtRPROJointSetRelativeAngularVelocity(MdtRPROJointID j, MeVector3 w)
{
    fromBt(toBt(w), j->omega);
}

void MEAPI MdtRPROJointSetAngularStrength(const MdtRPROJointID j, const MeReal sX, const MeReal sY, const MeReal sZ)
{
    fromBt(btVector3(sX, sY, sZ), j->angular_fmax);
}

void MEAPI MdtRPROJointSetLinearStrength(const MdtRPROJointID j, const MeReal sX, const MeReal sY, const MeReal sZ)
{
    fromBt(btVector3(sX, sY, sZ), j->linear_fmax);
}

} // extern "C"
