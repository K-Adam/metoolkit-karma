#include "MdtImpl.h"

namespace {

MeReal angleOf(MeReal cosHalf) { return 2 * btAcos(cosHalf); }
MeReal cosHalfOf(MeReal angle) { return btCos(angle / 2); }

// Twist of body 1's frame relative to body 2's about the primary axis, from a swing-twist decomposition.
btScalar twist(const btMatrix3x3& ra, const btMatrix3x3& rb)
{
    btQuaternion q;
    (rb.transpose() * ra).getRotation(q);
    return btNormalizeAngle(2 * btAtan2(q.x(), q.w()));
}

void oneSided(MdtJointRows& r, const MdtRow& j, btScalar error, btScalar kp, btScalar kd, bool upper)
{
    if (MdtRow* row = r.lock(j, error, kp, kd))
        r.oneSided(*row, upper);
}

// Keeps the swing of body 1's primary axis inside an elliptical cone around body 2's, with limit angle a1
// towards body 2's orthogonal axis and a2 towards the third axis.
void cone(MdtJointRows& r, btScalar a1, btScalar a2, btScalar kp, btScalar kd)
{
    btVector3 a = r.axisA(0), x = r.axisB(0), u = x.cross(a);
    btScalar s = u.length();
    if (s < SIMD_EPSILON)
        return;
    btScalar swing = btAtan2(s, a.dot(x));
    btScalar cy = a.dot(r.axisB(1)) / s, cz = a.dot(r.axisB(2)) / s;
    btScalar d = btSqrt(a2 * a2 * cy * cy + a1 * a1 * cz * cz);
    // A zero angle collapses the ellipse, and both zero locks the swing.
    btScalar limit = d < SIMD_EPSILON ? btMin(a1, a2) : a1 * a2 / d;
    if (swing > limit)
        oneSided(r, r.angular(u / s), swing - limit, kp, kd, true);
}

void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtSkeletalID j = MdtConstraintDCastSkeletal(c);
    r.lockPoint();

    MeReal a1 = angleOf(j->cos_half_cone_limit_angle_1), a2 = angleOf(j->cos_half_cone_limit_angle_2);
    switch (j->coneOption)
    {
    case MdtSkeletalConeOptionSlot:
    {
        btVector3 a = r.axisA(0), z = r.axisB(2);
        r.lock(r.angular(a.cross(z)), a.dot(z), j->cone_stiffness, j->cone_damping);
        cone(r, a1, a1, j->cone_stiffness, j->cone_damping);
        break;
    }
    case MdtSkeletalConeOptionCone:
        cone(r, a1, a2, j->cone_stiffness, j->cone_damping);
        break;
    case MdtSkeletalConeOptionFixed:
        r.lockAxis(j->cone_stiffness, j->cone_damping);
        break;
    default:
        break;
    }

    btVector3 axis = r.axisA(0) + r.axisB(0);
    axis = axis.length2() > SIMD_EPSILON ? axis.normalized() : r.axisA(0);
    btScalar t = twist(r.fa.getBasis(), r.fb.getBasis()), limit = angleOf(j->cos_half_twist_limit_angle);
    if (j->twistOption == MdtSkeletalTwistOptionFixed)
        r.lock(r.angular(axis), t, j->twist_stiffness, j->twist_damping);
    else if (j->twistOption == MdtSkeletalTwistOptionLimited && btFabs(t) > limit)
        oneSided(r, r.angular(axis), t > 0 ? t - limit : t + limit, j->twist_stiffness, j->twist_damping, t > 0);
}

const MdtConstraintOps ops = mdtJointOps<MdtSkeletal>(rows);

} // namespace

extern "C" {

MdtSkeletalID MEAPI MdtSkeletalCreate(const MdtWorldID w)
{
    MdtSkeletalID j = mdtJointCreate<MdtSkeletal>(w, MdtBclSKELETAL, ops, MdtBclGETMAXROWSSKELETAL);
    MdtSkeletalReset(j);
    return j;
}

void MEAPI MdtSkeletalReset(const MdtSkeletalID j)
{
    mdtConstraintResetFrames(j->head);
    j->combinedLimits = 0;
    j->coneOption = MdtSkeletalConeOptionCone;
    j->twistOption = MdtSkeletalTwistOptionFixed;
    j->cos_half_twist_limit_angle = j->cos_half_cone_limit_angle_1 = j->cos_half_cone_limit_angle_2 =
        cosHalfOf(ME_PI / 4);
    j->twist_stiffness = j->twist_damping = j->cone_stiffness = j->cone_damping = MEINFINITY;
}

MdtConstraintID MEAPI MdtSkeletalQuaConstraint(const MdtSkeletalID j)
{
    return mdtQua(j);
}

MdtSkeletalID MEAPI MdtConstraintDCastSkeletal(const MdtConstraintID c)
{
    return mdtDCast<MdtSkeletal>(c, MdtBclSKELETAL);
}

MdtSkeletalConeOption MEAPI MdtSkeletalGetConeOption(const MdtSkeletalID j)
{
    return j->coneOption;
}

MeReal MEAPI MdtSkeletalGetConePrimaryLimitAngle(const MdtSkeletalID j)
{
    return angleOf(j->cos_half_cone_limit_angle_1);
}

MeReal MEAPI MdtSkeletalGetConeSecondaryLimitAngle(const MdtSkeletalID j)
{
    return angleOf(j->cos_half_cone_limit_angle_2);
}

MeReal MEAPI MdtSkeletalGetConeStiffness(const MdtSkeletalID j)
{
    return j->cone_stiffness;
}

MeReal MEAPI MdtSkeletalGetConeDamping(const MdtSkeletalID j)
{
    return j->cone_damping;
}

MdtSkeletalTwistOption MEAPI MdtSkeletalGetTwistOption(const MdtSkeletalID j)
{
    return j->twistOption;
}

MeReal MEAPI MdtSkeletalGetTwistLimitAngle(const MdtSkeletalID j)
{
    return angleOf(j->cos_half_twist_limit_angle);
}

MeReal MEAPI MdtSkeletalGetTwistStiffness(const MdtSkeletalID j)
{
    return j->twist_stiffness;
}

MeReal MEAPI MdtSkeletalGetTwistDamping(const MdtSkeletalID j)
{
    return j->twist_damping;
}

// Turns body 2's frame about its primary axis so the current twist reads zero.
void MEAPI MdtSkeletalZeroTwist(const MdtSkeletalID j)
{
    MdtConstraintID c = mdtQua(j);
    btMatrix3x3 ra = mdtFrameWorld(c, 0).getBasis(), rb = mdtFrameWorld(c, 1).getBasis();
    btTransform ref = mdtRef(c, 1);
    ref.setBasis(ref.getBasis() * btMatrix3x3(btQuaternion(btVector3(1, 0, 0), twist(ra, rb))));
    mdtSetRef(c, 1, ref);
}

void MEAPI MdtSkeletalSetConeOption(const MdtSkeletalID j, MdtSkeletalConeOption co)
{
    j->coneOption = co;
}

void MEAPI MdtSkeletalSetConePrimaryLimitAngle(const MdtSkeletalID j, const MeReal theta)
{
    j->cos_half_cone_limit_angle_1 = cosHalfOf(theta);
}

void MEAPI MdtSkeletalSetConeSecondaryLimitAngle(const MdtSkeletalID j, const MeReal theta)
{
    j->cos_half_cone_limit_angle_2 = cosHalfOf(theta);
}

void MEAPI MdtSkeletalSetConeStiffness(const MdtSkeletalID j, const MeReal kp)
{
    j->cone_stiffness = kp;
}

void MEAPI MdtSkeletalSetConeDamping(const MdtSkeletalID j, const MeReal kd)
{
    j->cone_damping = kd;
}

void MEAPI MdtSkeletalSetTwistOption(const MdtSkeletalID j, MdtSkeletalTwistOption to)
{
    j->twistOption = to;
}

void MEAPI MdtSkeletalSetTwistLimitAngle(const MdtSkeletalID j, const MeReal theta)
{
    j->cos_half_twist_limit_angle = cosHalfOf(theta);
}

void MEAPI MdtSkeletalSetTwistStiffness(const MdtSkeletalID j, const MeReal kp)
{
    j->twist_stiffness = kp;
}

void MEAPI MdtSkeletalSetTwistDamping(const MdtSkeletalID j, const MeReal kd)
{
    j->twist_damping = kd;
}

} // extern "C"
