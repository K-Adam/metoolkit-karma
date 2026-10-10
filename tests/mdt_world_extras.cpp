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

} // namespace

int main()
{
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

    // Solver internals Bullet replaces.
    CHECK(MdtWorldGetKeaPoolChunk(w) == nullptr);
    CHECK(MdtWorldGetPartitionOutputChunk(w) == nullptr);
    CHECK(MdtWorldGetKeaTMChunk(w) == nullptr && MdtWorldGetKeaConstraintsChunk(w) == nullptr);
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
