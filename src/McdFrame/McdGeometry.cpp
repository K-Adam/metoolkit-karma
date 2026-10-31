#include "McdImpl.h"

namespace {

template <class F>
void withInstance(McdGeometryID g, MeMatrix4 tm, F f)
{
    McdGeometryInstance ins{};
    ins.mGeometry = g;
    ins.mTM = tm;
    McdGeometryInstanceUpdateAABB(&ins, tm, 1);
    f(&ins);
    mcdFreeChildInstances(&ins);
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
    return g && g->frame && mcdVTable(g) != nullptr;
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
    const McdGeometryVTable* v = mcdVTable(g);
    if (v && v->destroy)
        v->destroy(g);
}

void MEAPI McdGeometryGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    const McdGeometryVTable* v = mcdVTable(g);
    if (v && v->getBSphere)
        v->getBSphere(g, center, radius);
}

MeI16 MEAPI McdGeometryGetMassProperties(McdGeometryID g, MeMatrix4 relativeTransform, MeMatrix3 inertiaMatrix,
                                         MeReal* volume)
{
    const McdGeometryVTable* v = mcdVTable(g);
    return v && v->getMassProperties ? v->getMassProperties(g, relativeTransform, inertiaMatrix, volume) : 0;
}

McdLineSegIntersectFnPtr MEAPI McdGeometryGetLineSegIntersectFnPtr(McdGeometryID g)
{
    const McdGeometryVTable* v = mcdVTable(g);
    return v ? v->lineSegIntersect : nullptr;
}

void MEAPI McdGeometryGetAABB(McdGeometryID g, MeMatrix4 tm, MeVector3 minCorner, MeVector3 maxCorner)
{
    withInstance(g, tm, [&](McdGeometryInstanceID ins) { McdGeometryInstanceGetAABB(ins, minCorner, maxCorner); });
}

void MEAPI McdGeometryMaximumPoint(McdGeometryID g, MeMatrix4 tm, MeReal* const inDir, MeReal* const outPoint)
{
    withInstance(g, tm, [&](McdGeometryInstanceID ins) { McdGeometryInstanceMaximumPoint(ins, inDir, outPoint); });
}

void MEAPI McdGeometryDebugDraw(const McdGeometryID geom, const MeMatrix4Ptr tm, const MeReal colour[3])
{
    const McdGeometryVTable* v = mcdVTable(geom);
    if (v && v->debugDraw)
        v->debugDraw(geom, tm, colour);
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
    btCollisionShape* shape = McdFrameworkTypeIsValid(g->frame, type) && f->shapeFns[type] ? f->shapeFns[type](g) : nullptr;
    return shape ? shape : &f->emptyShape;
}
