#include <cmath>
#include <limits>

#include <Mdt.h>

#include "check.h"

namespace {

struct Reported
{
    int calls = 0;
    MdtKeaBody* first = nullptr;
    int nBodies = 0;
};

void MEAPI onSimError(MdtKeaConstraints*, MdtKeaBody** kb, int nBodies, void* data)
{
    auto* r = static_cast<Reported*>(data);
    ++r->calls;
    r->first = kb[0];
    r->nBodies = nBodies;
}

MeReal movedInOneStep(MdtWorldID w, MdtBodyID b, MeReal step)
{
    MdtBodySetPosition(b, 0, 0, 0);
    MdtBodySetLinearVelocity(b, 1, 0, 0);
    MdtWorldStepSafeTime(w, step);
    MeVector3 p;
    MdtBodyGetPosition(b, p);
    return p[0];
}

// A sleeping chain hit at one end wakes as a whole in a single step, whatever the order of the constraint set.
void wakeChain()
{
    const int n = 10;
    MdtWorldID w = MdtWorldCreate(n, n, 1, 1);
    MdtWorldSetGravity(w, 0, 0, 0);
    MdtBodyID bodies[n];
    for (int i = 0; i < n; ++i)
    {
        bodies[i] = MdtBodyCreate(w);
        MdtBodySetPosition(bodies[i], (MeReal)i, 0, 0);
    }
    for (int i = 1; i < n; ++i)
    {
        MdtHingeID h = MdtHingeCreate(w);
        MdtHingeSetBodies(h, bodies[i - 1], bodies[i]);
        MdtHingeSetPosition(h, (MeReal)i - (MeReal)0.5, 0, 0);
        MdtHingeEnable(h);
    }
    MdtBodyEnable(bodies[0]);
    MdtBodySetLinearVelocity(bodies[0], 0, 1, 0);
    MdtWorldStep(w, (MeReal)0.01);
    CHECK(MdtWorldGetEnabledBodies(w) == n);
    MdtWorldDestroy(w);
}

MdtBodyID wokenByCallback = nullptr;

void MEAPI enableOther(MdtBodyID)
{
    MdtBodyEnable(wokenByCallback);
}

// A zero step changes nothing, and a disable callback may enable another body.
void stepEdges()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, 0, 0);
    MdtBodyID b = MdtBodyCreate(w);
    MdtBodyEnable(b);
    MdtBodySetLinearVelocity(b, 1, 0, 0);
    MdtBodyAddImpulse(b, 1, 0, 0);
    MdtWorldStep(w, 0);
    MeVector3 p, a;
    MdtBodyGetPosition(b, p);
    MdtBodyGetLinearAcceleration(b, a);
    CHECK(p[0] == 0 && std::isfinite(a[0]));
    MdtWorldStep(w, (MeReal)0.01);
    MdtBodyGetPosition(b, p);
    CHECK(p[0] > (MeReal)0.01);

    MdtBodySetLinearVelocity(b, 0, 0, 0);
    wokenByCallback = MdtBodyCreate(w);
    w->bodyDisableCallback = enableOther;
    for (int i = 0; i < 500 && MdtBodyIsEnabled(b); ++i)
        MdtWorldStep(w, (MeReal)0.01);
    CHECK(!MdtBodyIsEnabled(b) && MdtBodyIsEnabled(wokenByCallback));
    w->bodyDisableCallback = nullptr;
    MdtWorldDestroy(w);
}

// Rotation is not capped at pi/4 per step, so a fast spinning body turns by its angular velocity.
void fastSpin()
{
    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, 0, 0);
    MdtBodyID b = MdtBodyCreate(w);
    MdtBodyEnable(b);
    MdtBodySetAngularVelocity(b, 0, 0, 100);
    MdtWorldStep(w, (MeReal)0.01);
    MeVector4 q;
    MdtBodyGetQuaternion(b, q);
    CHECK_NEAR(2 * std::acos(q[0]), 1, 1e-4);
    MdtWorldDestroy(w);
}

} // namespace

int main()
{
    wakeChain();
    stepEdges();
    fastSpin();

    MdtWorldID w = MdtWorldCreate(10, 10, 1, 1);
    MdtWorldSetGravity(w, 0, 0, 0);
    MdtWorldSetAutoDisable(w, 0);
    MdtBodyID b = MdtBodyCreate(w);
    MdtBodyEnable(b);

    // A body is stepped by its safe time, but never by less than the world's minimum.
    CHECK_NEAR(movedInOneStep(w, b, 1), 1, 1e-5);
    MdtBodySetSafeTime(b, (MeReal)0.25);
    CHECK_NEAR(movedInOneStep(w, b, 1), 0.25, 1e-5);
    CHECK_NEAR(movedInOneStep(w, b, (MeReal)0.1), 0.1, 1e-5);
    MdtWorldSetMinSafeTime(w, (MeReal)0.5);
    CHECK_NEAR(movedInOneStep(w, b, 1), 0.5, 1e-5);

    // Non-finite state reaches the error callback only while checking is on.
    CHECK(MdtWorldGetSimErrorCB(w) == MdtDefaultSimErrorCallBack);
    CHECK(!MdtWorldGetCheckSim(w));
    Reported reported;
    MdtWorldSetSimErrorCB(w, onSimError, &reported);
    CHECK(MdtWorldGetSimErrorCB(w) == onSimError);
    MdtBodyID healthy = MdtBodyCreate(w);
    MdtBodyEnable(healthy);
    MdtWorldStep(w, (MeReal)0.01);
    MdtBodySetLinearVelocity(b, std::numeric_limits<MeReal>::quiet_NaN(), 0, 0);
    MdtWorldStep(w, (MeReal)0.01);
    CHECK(reported.calls == 0);
    MdtWorldSetCheckSim(w, 1);
    CHECK(MdtWorldGetCheckSim(w));
    MdtWorldStep(w, (MeReal)0.01);
    CHECK(reported.calls == 1 && reported.nBodies == 1 && reported.first == &b->keaBody);
    MdtWorldSetSimErrorCB(w, MdtDefaultSimErrorCallBack, nullptr);
    MdtWorldStep(w, (MeReal)0.01);

    // The chunks callers stepping partitions themselves draw from.
    CHECK(MdtWorldGetKeaPoolChunk(w) == &w->keaPool);
    CHECK(MdtWorldGetPartitionOutputChunk(w) == &w->partOutChunk);
    CHECK(MdtWorldGetKeaTMChunk(w) == &w->keaTMChunk && MdtWorldGetKeaConstraintsChunk(w) == &w->keaConstraintsChunk);
    CHECK(MdtWorldGetMaxMemoryPoolUsed(w) == 0);
    MdtKeaDebugDataRequest request{};
    MdtWorldSetKeaDebugRequest(w, request);
    MdtWorldSetDebugDrawing(w, MdtDebugDrawOptions(0));
    int log[8] = {};
    MdtWorldSetMatrixSizeLog(w, log, 8);

    // Recreating empties the world and restores its defaults.
    MdtBodyID other = MdtBodyCreate(w);
    MdtConstraintID joint = MdtBSJointQuaConstraint(MdtBSJointCreate(w));
    MdtConstraintSetBodies(joint, other, nullptr);
    MdtConstraintEnable(joint);
    MdtWorldReCreate(w);
    CHECK(MdtWorldGetTotalBodies(w) == 0 && MdtWorldGetTotalConstraints(w) == 0);
    CHECK(MdtWorldGetEnabledBodies(w) == 0 && MdtWorldGetEnabledConstraints(w) == 0);
    MeVector3 g;
    MdtWorldGetGravity(w, g);
    CHECK_NEAR(g[1], MDTWORLD_DEFAULT_GRAVITY_Y, 0);
    CHECK_NEAR(MdtWorldGetMinSafeTime(w), MDTWORLD_DEFAULT_MIN_SAFETIME, 0);
    CHECK(MdtWorldGetMaxBodies(w) == 10);
    MdtBodyID again = MdtBodyCreate(w);
    MdtBodyEnable(again);
    MdtWorldStep(w, (MeReal)0.01);
    MdtWorldDestroy(w);
    return 0;
}
