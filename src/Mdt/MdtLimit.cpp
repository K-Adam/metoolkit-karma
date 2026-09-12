#include <algorithm>

#include "MdtImpl.h"

// Karma uses one row per limited degree of freedom: the limit row while at or beyond a stop, otherwise the
// motor or the lock.
void mdtLimitRows(MdtJointRows& r, MdtBclLimit& l, const MdtRow& dof, btScalar position)
{
    btScalar pos = position + l.offset;
    btScalar lo = l.limit[0].stop, hi = l.limit[1].stop;
    l.position = l.previous_position = pos;
    l.velocity = r.velocity(dof);
    l.bPositionInitialised = 1;
    l.overshoot = !l.bLimited ? 0 : pos > hi ? pos - hi : pos < lo ? pos - lo : 0;

    if (l.bLimited && (pos >= hi || pos <= lo))
    {
        bool upper = pos - hi >= lo - pos;
        const MdtBclSingleLimit& sl = l.limit[upper];
        bool hard = sl.stiffness >= MEINFINITY || sl.stiffness > l.damping_thresh;
        MdtRow* row = r.lock(dof, pos - sl.stop, hard ? MEINFINITY : sl.stiffness, sl.damping);
        if (!row)
            return;
        if (hard)
        {
            btScalar bounce = -sl.restitution * l.velocity;
            row->rhs = upper ? std::min(row->rhs, bounce) : std::max(row->rhs, bounce);
        }
        r.oneSided(*row, upper);
    }
    else if (l.bPowered)
        r.motor(dof, l.desired_vel, l.fmax);
    else if (l.is_locked)
    {
        MdtRow* row = r.lock(dof, pos - l.position_lock);
        row->lo = -l.fmax * r.h;
        row->hi = l.fmax * r.h;
    }
}

extern "C" {

void MEAPI MdtSingleLimitReset(const MdtSingleLimitID sl)
{
    sl->stop = 0;
    sl->stiffness = MEINFINITY;
    sl->damping = 0;
    sl->restitution = 1;
}

MeReal MEAPI MdtSingleLimitGetStop(const MdtSingleLimitID sl)
{
    return sl->stop;
}

MeReal MEAPI MdtSingleLimitGetStiffness(const MdtSingleLimitID sl)
{
    return sl->stiffness;
}

MeReal MEAPI MdtSingleLimitGetDamping(const MdtSingleLimitID sl)
{
    return sl->damping;
}

MeReal MEAPI MdtSingleLimitGetRestitution(const MdtSingleLimitID sl)
{
    return sl->restitution;
}

void MEAPI MdtSingleLimitSetStop(const MdtSingleLimitID sl, const MeReal NewStop)
{
    sl->stop = NewStop;
}

void MEAPI MdtSingleLimitSetStiffness(const MdtSingleLimitID sl, const MeReal NewStiffness)
{
    sl->stiffness = std::max(NewStiffness, (MeReal)0);
}

void MEAPI MdtSingleLimitSetDamping(const MdtSingleLimitID sl, const MeReal NewDamping)
{
    sl->damping = std::max(NewDamping, (MeReal)0);
}

void MEAPI MdtSingleLimitSetRestitution(const MdtSingleLimitID sl, const MeReal NewRestitution)
{
    sl->restitution = std::clamp(NewRestitution, (MeReal)0, (MeReal)1);
}

void MEAPI MdtLimitReset(const MdtLimitID l)
{
    *l = MdtBclLimit{};
    l->damping_thresh = MEINFINITY;
    MdtSingleLimitReset(&l->limit[0]);
    MdtSingleLimitReset(&l->limit[1]);
}

void MEAPI MdtLimitResetState(const MdtLimitID l)
{
    l->bPositionInitialised = 0;
    l->bRelaxingToLimit = 0;
    l->overshoot = 0;
    l->position = 0;
    l->previous_position = 0;
    l->velocity = 0;
}

MeBool MEAPI MdtLimitIsActive(const MdtLimitID l)
{
    return l->bLimited;
}

MeBool MEAPI MdtLimitPositionIsCalculated(const MdtLimitID l)
{
    return l->bCalculatePosition;
}

MdtSingleLimitID MEAPI MdtLimitGetLowerLimit(const MdtLimitID l)
{
    return &l->limit[0];
}

MdtSingleLimitID MEAPI MdtLimitGetUpperLimit(const MdtLimitID l)
{
    return &l->limit[1];
}

MeReal MEAPI MdtLimitGetPosition(const MdtLimitID l)
{
    return l->position;
}

MeReal MEAPI MdtLimitGetOvershoot(const MdtLimitID l)
{
    return l->overshoot;
}

MeReal MEAPI MdtLimitGetVelocity(const MdtLimitID l)
{
    return l->velocity;
}

MeReal MEAPI MdtLimitGetStiffnessThreshold(const MdtLimitID l)
{
    return l->damping_thresh;
}

MeBool MEAPI MdtLimitIsMotorized(const MdtLimitID l)
{
    return l->bPowered;
}

MeBool MEAPI MdtLimitIsLocked(const MdtLimitID l)
{
    return l->is_locked;
}

MeReal MEAPI MdtLimitGetMotorDesiredVelocity(const MdtLimitID l)
{
    return l->desired_vel;
}

MeReal MEAPI MdtLimitGetMotorMaxForce(const MdtLimitID l)
{
    return l->fmax;
}

void MEAPI MdtLimitSetLowerLimit(const MdtLimitID l, const MdtSingleLimitID sl)
{
    l->limit[0] = *sl;
    if (l->limit[1].stop < sl->stop)
        l->limit[1].stop = sl->stop;
}

void MEAPI MdtLimitSetUpperLimit(const MdtLimitID l, const MdtSingleLimitID sl)
{
    l->limit[1] = *sl;
    if (l->limit[0].stop > sl->stop)
        l->limit[0].stop = sl->stop;
}

void MEAPI MdtLimitSetPosition(MdtLimitID l, const MeReal NewPosition)
{
    l->offset += NewPosition - l->position;
    l->position = NewPosition;
}

void MEAPI MdtLimitActivateLimits(const MdtLimitID l, const MeBool NewActivationState)
{
    l->bLimited = NewActivationState != 0;
    if (l->bLimited)
        l->bCalculatePosition = 1;
}

void MEAPI MdtLimitCalculatePosition(const MdtLimitID l, const MeBool NewState)
{
    l->bCalculatePosition = NewState || l->bLimited || l->bPowered;
}

void MEAPI MdtLimitSetStiffnessThreshold(const MdtLimitID l, const MeReal NewStiffnessThreshold)
{
    l->damping_thresh = std::max(NewStiffnessThreshold, (MeReal)0);
}

void MEAPI MdtLimitActivateMotor(const MdtLimitID l, const MeBool NewActivationState)
{
    l->bPowered = NewActivationState != 0;
    if (l->bPowered)
    {
        l->bCalculatePosition = 1;
        l->is_locked = 0;
    }
}

void MEAPI MdtLimitSetLimitedForceMotor(const MdtLimitID l, const MeReal desiredVelocity, const MeReal forceLimit)
{
    l->desired_vel = desiredVelocity;
    l->fmax = std::max(forceLimit, (MeReal)0);
    MdtLimitActivateMotor(l, l->fmax > 0);
}

void MEAPI MdtLimitActivateLock(const MdtLimitID l, const MeBool NewActivationState)
{
    l->is_locked = NewActivationState != 0;
    if (l->is_locked)
    {
        l->bPowered = 0;
        l->bCalculatePosition = 1;
    }
}

void MEAPI MdtLimitSetLock(const MdtLimitID l, const MeReal lock_value, const MeReal forceLimit)
{
    l->position_lock = lock_value;
    l->fmax = std::max(forceLimit, (MeReal)0);
    MdtLimitActivateLock(l, 1);
}

} // extern "C"
