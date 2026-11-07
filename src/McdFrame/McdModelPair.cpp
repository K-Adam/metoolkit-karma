#include "McdImpl.h"

extern "C" {

McdModelPairID MEAPI McdModelPairCreate(McdModelID m1, McdModelID m2)
{
    auto* p = new McdModelPair{};
    McdModelPairReset(p, m1, m2);
    return p;
}

void MEAPI McdModelPairDestroy(McdModelPairID p)
{
    if (!p)
        return;
    // Releases the narrowphase state of a pair the application never said goodbye to.
    McdGoodbye(p);
    delete p;
}

void MEAPI McdModelPairReset(McdModelPairID p, McdModelID m1, McdModelID m2)
{
    // The old contents may be garbage, so a stale live entry is dropped by address only.
    if (m1)
        impl(m1->frame)->livePairs.erase(p);
    *p = McdModelPair{};
    p->model1 = m1;
    p->model2 = m2;
    p->phase = kMcdFFStateInactive;
    p->request = m1 ? m1->frame->defaultRequest : nullptr;
}

void MEAPI McdModelPairGetModels(McdModelPairID p, McdModelID* m1, McdModelID* m2)
{
    *m1 = p->model1;
    *m2 = p->model2;
}

McdRequest* MEAPI McdModelPairGetRequestPtr(const McdModelPairID p)
{
    return p->request;
}

void MEAPI McdModelPairSetRequestPtr(McdModelPairID p, McdRequest* r)
{
    p->request = r;
}

void* MEAPI McdModelPairGetUserData(McdModelPairID p)
{
    return p->userData;
}

void MEAPI McdModelPairSetUserData(McdModelPairID p, void* d)
{
    p->userData = d;
}

void MEAPI McdModelPairGetGeometryNames(McdModelPairID p, const char** stringPtr1, const char** stringPtr2)
{
    *stringPtr1 = McdGeometryGetTypeName(McdModelGetGeometry(p->model1));
    *stringPtr2 = McdGeometryGetTypeName(McdModelGetGeometry(p->model2));
}

} // extern "C"
