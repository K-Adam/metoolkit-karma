#include "MdtImpl.h"

namespace {

// Body 1 is the chassis and body 2 the wheel. The steering axis is the chassis frame's primary axis and the
// hinge axis the wheel frame's orthogonal axis.
struct Geometry
{
    btVector3 steer, hinge, attach, wheel;
    btScalar steerAngle, hingeAngle, height;
};

Geometry geometry(const btTransform& fa, const btTransform& fb)
{
    Geometry g;
    g.steer = fa.getBasis().getColumn(0);
    g.hinge = fb.getBasis().getColumn(1);
    g.attach = fa.getOrigin();
    g.wheel = fb.getOrigin();
    btVector3 h0 = fa.getBasis().getColumn(1), s1 = fb.getBasis().getColumn(0);
    g.steerAngle = btAtan2(h0.cross(g.hinge).dot(g.steer), h0.dot(g.hinge));
    g.hingeAngle = btAtan2(g.steer.cross(s1).dot(g.hinge), g.steer.dot(s1));
    g.height = (g.wheel - g.attach).dot(g.steer);
    return g;
}

void rows(MdtConstraintID c, MdtJointRows& r)
{
    MdtCarWheelID j = MdtConstraintDCastCarWheel(c);
    Geometry g = geometry(r.fa, r.fb);
    btVector3 d = g.attach - g.wheel;
    for (int k = 1; k < 3; ++k)
        r.lock(r.linear(r.axisA(k), g.wheel, g.wheel), d.dot(r.axisA(k)));
    r.lock(r.angular(g.steer.cross(g.hinge)), g.steer.dot(g.hinge));

    // Steering and hinge rows measure the wheel relative to the chassis.
    MdtRow steer = r.angular(-g.steer);
    if (j->slock)
        r.lock(steer, g.steerAngle);
    else if (j->sfmax > 0)
        r.motor(steer, j->svel, j->sfmax);
    if (j->hfmax > 0)
        r.motor(r.angular(-g.hinge), j->hvel, j->hfmax);

    MdtRow lift = r.linear(-g.steer, g.wheel, g.wheel);
    if (g.height > j->shi || g.height < j->slo)
    {
        bool high = g.height > j->shi;
        MdtRow* row = r.lock(lift, g.height - (high ? j->shi : j->slo));
        row->cfm = j->slsoft / r.effectiveMass(*row);
        r.oneSided(*row, high);
    }
    else
        r.lock(lift, g.height - j->sref, j->skp, j->skd);
}

const MdtConstraintOps ops = mdtJointOps<MdtCarWheel>(rows);

Geometry current(MdtCarWheelID j)
{
    return geometry(mdtFrameWorld(mdtQua(j), 0), mdtFrameWorld(mdtQua(j), 1));
}

btVector3 angularVelocity(MdtBodyID b)
{
    return b ? toBt(b->keaBody.velrot) : btVector3(0, 0, 0);
}

btVector3 pointVelocity(MdtBodyID b, const btVector3& p)
{
    if (!b)
        return btVector3(0, 0, 0);
    MeVector3 q = {p.x(), p.y(), p.z()}, v;
    MdtBodyGetVelocityAtPoint(b, q, v);
    return toBt(v);
}

btVector3 relativeAngularVelocity(MdtCarWheelID j)
{
    return angularVelocity(j->head.mdtbody[1]) - angularVelocity(j->head.mdtbody[0]);
}

} // namespace

extern "C" {

MdtCarWheelID MEAPI MdtCarWheelCreate(const MdtWorldID w)
{
    MdtCarWheelID j = mdtJointCreate<MdtCarWheel>(w, MdtBclCARWHEEL, ops, MdtBclGETMAXROWSCARWHEEL);
    MdtCarWheelReset(j);
    return j;
}

void MEAPI MdtCarWheelReset(const MdtCarWheelID j)
{
    mdtConstraintResetFrames(j->head);
    j->skp = j->skd = 0;
    j->svel = j->hvel = j->hfmax = 0;
    j->sfmax = 1e9f;
    j->slock = 0;
    j->shi = 1e9f;
    j->slo = -1e9f;
    j->sref = j->slsoft = 0;
}

MdtConstraintID MEAPI MdtCarWheelQuaConstraint(const MdtCarWheelID j)
{
    return mdtQua(j);
}

MdtCarWheelID MEAPI MdtConstraintDCastCarWheel(const MdtConstraintID c)
{
    return mdtDCast<MdtCarWheel>(c, MdtBclCARWHEEL);
}

void MEAPI MdtCarWheelGetSteeringAxis(const MdtCarWheelID j, MeVector3 v)
{
    fromBt(current(j).steer, v);
}

MeReal MEAPI MdtCarWheelGetSteeringAngle(const MdtCarWheelID j)
{
    return current(j).steerAngle;
}

MeReal MEAPI MdtCarWheelGetSteeringAngleRate(const MdtCarWheelID j)
{
    return relativeAngularVelocity(j).dot(current(j).steer);
}

void MEAPI MdtCarWheelGetHingeAxis(const MdtCarWheelID j, MeVector3 v)
{
    fromBt(current(j).hinge, v);
}

MeReal MEAPI MdtCarWheelGetHingeAngle(const MdtCarWheelID j)
{
    btScalar a = current(j).hingeAngle;
    return a < 0 ? a + SIMD_2_PI : a;
}

MeReal MEAPI MdtCarWheelGetHingeAngleRate(const MdtCarWheelID j)
{
    return relativeAngularVelocity(j).dot(current(j).hinge);
}

MeReal MEAPI MdtCarWheelGetSuspensionHeight(const MdtCarWheelID j)
{
    return current(j).height;
}

MeReal MEAPI MdtCarWheelGetSuspensionRate(const MdtCarWheelID j)
{
    Geometry g = current(j);
    btVector3 v = pointVelocity(j->head.mdtbody[1], g.wheel) - pointVelocity(j->head.mdtbody[0], g.wheel);
    return v.dot(g.steer);
}

MeReal MEAPI MdtCarWheelGetSteeringMotorDesiredVelocity(const MdtCarWheelID j)
{
    return j->svel;
}

MeReal MEAPI MdtCarWheelGetSteeringMotorMaxForce(const MdtCarWheelID j)
{
    return j->sfmax;
}

MeBool MEAPI MdtCarWheelIsSteeringLocked(const MdtCarWheelID j)
{
    return j->slock;
}

MeReal MEAPI MdtCarWheelGetHingeMotorDesiredVelocity(const MdtCarWheelID j)
{
    return j->hvel;
}

MeReal MEAPI MdtCarWheelGetHingeMotorMaxForce(const MdtCarWheelID j)
{
    return j->hfmax;
}

MeReal MEAPI MdtCarWheelGetSuspensionHighLimit(const MdtCarWheelID j)
{
    return j->shi;
}

MeReal MEAPI MdtCarWheelGetSuspensionLowLimit(const MdtCarWheelID j)
{
    return j->slo;
}

MeReal MEAPI MdtCarWheelGetSuspensionLimitSoftness(const MdtCarWheelID j)
{
    return j->slsoft;
}

MeReal MEAPI MdtCarWheelGetSuspensionReference(const MdtCarWheelID j)
{
    return j->sref;
}

MeReal MEAPI MdtCarWheelGetSuspensionKp(const MdtCarWheelID j)
{
    return j->skp;
}

MeReal MEAPI MdtCarWheelGetSuspensionKd(const MdtCarWheelID j)
{
    return j->skd;
}

void MEAPI MdtCarWheelSetSteeringLimitedForceMotor(const MdtCarWheelID j, const MeReal desiredVelocity,
                                                   const MeReal forceLimit)
{
    j->svel = desiredVelocity;
    j->sfmax = forceLimit;
}

void MEAPI MdtCarWheelSetSteeringLock(const MdtCarWheelID j, const MeBool lock)
{
    j->slock = lock != 0;
}

void MEAPI MdtCarWheelSetHingeLimitedForceMotor(const MdtCarWheelID j, const MeReal desiredVelocity,
                                                const MeReal forceLimit)
{
    j->hvel = desiredVelocity;
    j->hfmax = forceLimit;
}

void MEAPI MdtCarWheelSetSuspension(const MdtCarWheelID j, const MeReal Kp, const MeReal Kd,
                                    const MeReal limit_softness, const MeReal lolimit, const MeReal hilimit,
                                    const MeReal reference)
{
    j->skp = Kp;
    j->skd = Kd;
    j->slsoft = limit_softness;
    j->slo = lolimit;
    j->shi = hilimit;
    j->sref = reference;
}

} // extern "C"
