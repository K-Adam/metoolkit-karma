#include "McdImpl.h"

namespace {

const McdGeometryVTable* vtable(McdGeometryID g)
{
    McdFrameworkID f = g->frame;
    int type = McdGeometryGetTypeId(g);
    return McdFrameworkTypeIsRegistered(f, type) ? &f->geometryVTableTable[type] : nullptr;
}

} // namespace

extern "C" {

void MEAPI McdGeometryInit(McdGeometryID g, McdFramework* frame, MeI16 typeId)
{
    g->mRefCtAndID = (MeU32)(typeId & 255);
    g->frame = frame;
    g->prev = nullptr;
    g->next = frame->firstGeometry;
    if (g->next)
        g->next->prev = g;
    frame->firstGeometry = g;
    ++frame->geometryCount;
}

void MEAPI McdGeometryDeinit(McdGeometryID g)
{
    McdFrameworkID f = g->frame;
    if (g->prev)
        g->prev->next = g->next;
    else
        f->firstGeometry = g->next;
    if (g->next)
        g->next->prev = g->prev;
    --f->geometryCount;
}

MeBool MEAPI McdGeometryIsValid(McdGeometryID g)
{
    return g && g->frame && vtable(g) != nullptr;
}

const char* MEAPI McdGeometryGetTypeName(McdGeometryID g)
{
    return McdFrameworkGetTypeName(g->frame, McdGeometryGetTypeId(g));
}

void MEAPI McdGeometryDestroy(McdGeometryID g)
{
    if (!g)
        return;
    if (McdGeometryGetReferenceCount(g) > 0)
    {
        MeWarning(0, "McdGeometryDestroy: geometry is still used by %d model(s)", McdGeometryGetReferenceCount(g));
        return;
    }
    const McdGeometryVTable* v = vtable(g);
    if (v && v->destroy)
        v->destroy(g);
}

void MEAPI McdGeometryGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    const McdGeometryVTable* v = vtable(g);
    if (v && v->getBSphere)
        v->getBSphere(g, center, radius);
}

MeI16 MEAPI McdGeometryGetMassProperties(McdGeometryID g, MeMatrix4 relativeTransform, MeMatrix3 inertiaMatrix,
                                         MeReal* volume)
{
    const McdGeometryVTable* v = vtable(g);
    return v && v->getMassProperties ? v->getMassProperties(g, relativeTransform, inertiaMatrix, volume) : 0;
}

McdFrameworkID MEAPI McdGeometryGetFramework(McdGeometryID g)
{
    return g->frame;
}

void MEAPI McdGeometryIncrementReferenceCount(McdGeometryID g)
{
    g->mRefCtAndID += 256;
}

void MEAPI McdGeometryDecrementReferenceCount(McdGeometryID g)
{
    if (g->mRefCtAndID >= 256)
        g->mRefCtAndID -= 256;
}

void MEAPI McdGeometrySetReferenceCount(McdGeometryID g, int refCount)
{
    g->mRefCtAndID = ((MeU32)refCount << 8) | (g->mRefCtAndID & 255);
}

int MEAPI McdGeometryGetReferenceCount(McdGeometryID g)
{
    return (int)(g->mRefCtAndID >> 8);
}

} // extern "C"

btCollisionShape* mcdGeometryShape(McdGeometryID g)
{
    McdFrameworkImpl* f = impl(g->frame);
    int type = McdGeometryGetTypeId(g);
    return McdFrameworkTypeIsValid(g->frame, type) && f->shapeFns[type] ? f->shapeFns[type](g) : nullptr;
}
