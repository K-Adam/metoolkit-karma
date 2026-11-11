#include <MdtKea.h>
#include <MeMath.h>

#include "MdtImpl.h"
#include "Unsupported.h"

extern "C" {

MeCPUResources MEAPI MdtKeaQueryCPUResources(void)
{
    ME_UNSUPPORTED("MdtKeaQueryCPUResources");
    return {};
}

// Bullet's solver keeps its own working memory.
int MEAPI MdtKeaMemoryRequired(const int num_rows_exc_padding[], int num_partitions, int max_rows, int max_bodies)
{
    return 0;
}

// The bodies are the ones MdtPackPartition prepared, which start with their MdtKeaBody.
void MEAPI MdtKeaAddConstraintForces(MdtKeaConstraints constraints, MdtKeaBody * blist[],
                                     const MdtKeaTransformation tlist[], int num_bodies, MdtKeaParameters parameters)
{
    if (num_bodies == 0)
        return;
    MdtWorldImpl& w = *impl(reinterpret_cast<MdtBodyID>(blist[0])->world);
    mdtSolvePacked(w, parameters.stepsize, constraints.force);
}

// Kea's integrator: velocity from acceleration, then position, and the quaternion by its first order update. A
// fast spin axis turns the body by the exact rotation about that axis first.
void MEAPI MdtKeaIntegrateSystem(MdtKeaBody *const blist[], MdtKeaTransformation tlist[], int num_bodies,
                                 MdtKeaParameters parameters)
{
    MeReal h = parameters.stepsize;
    for (int i = 0; i < num_bodies; ++i)
    {
        MdtKeaBody& k = *blist[i];
        for (int j = 0; j < 3; ++j)
        {
            k.vel[j] += h * k.accel[j];
            k.velrot[j] += h * k.accelrot[j];
            tlist[i].pos[j] += h * k.vel[j];
        }
        btVector3 w = toBt(k.velrot);
        if (k.flags & MdtKeaBodyFlagUseFastSpin)
        {
            btVector3 axis = toBt(k.fastSpinAxis);
            MeReal spin = w.dot(axis);
            MeQuaternionFiniteRotation(k.qrot, k.fastSpinAxis, spin * h);
            w -= axis * spin;
        }
        btQuaternion q = toBtQuat(k.qrot[0], k.qrot[1], k.qrot[2], k.qrot[3]);
        q += btQuaternion(w.x(), w.y(), w.z(), 0) * q * (h / 2);
        q.normalize();
        fromBtQuat(q, k.qrot);
        btTransform t(q, toBt(tlist[i].pos));
        MeMatrix4 tm;
        fromBtTransform(t, tm);
        std::memcpy(&tlist[i], tm, sizeof(MeMatrix4));
    }
}

// Cache control for the PS2 vector units, nothing to do on other platforms.
void MdtFlushCache(int)
{
}

void MdtSyncDCache(void*, void*)
{
}

void MdtInvalidDCache(void*, void*)
{
}

} // extern "C"
