#include <algorithm>

#include "McdImpl.h"

extern "C" {

McdFrameworkID MEAPI McdInit(int geoTypeMaxCount, int modelCount, int instanceCount, MeReal unitLength)
{
    (void)modelCount;
    (void)instanceCount;
    auto* f = new McdFrameworkImpl();
    // Karma returns a whole contact patch from one intersect call, so Bullet must sample several points.
    f->config.setPlaneConvexMultipointIterations();
    f->config.setConvexConvexMultipointIterations();
    // Built-in type ids are fixed by McdGeometryTypes.h, so the table always covers them.
    int types = std::max<int>(geoTypeMaxCount, kMcdGeometryBuiltInTypes);
    f->vtables.assign(types, McdGeometryVTable{});
    f->shapeFns.assign(types, nullptr);
    f->interactions.assign(types * types, McdInteractions{});

    McdFramework& p = f->pub;
    p.geometryRegisteredCountMax = types;
    p.geometryVTableTable = f->vtables.data();
    p.interactionTable = f->interactions.data();
    p.mScale = unitLength;
    p.mDefaultPadding = unitLength * (MeReal)0.005;
    p.request = &f->request;
    p.defaultRequest = &f->request;
    p.toolkitVersionString = "Karma 1.2 (Bullet)";
    return &f->pub;
}

void MEAPI McdTerm(McdFrameworkID frame)
{
    if (!frame)
        return;
    // Objects the application left behind go before the term actions, which may release what they need.
    McdFrameworkImpl* f = impl(frame);
    while (!f->spaces.empty())
        f->spaceDestroyHook(*f->spaces.begin());
    McdFrameworkDestroyAllModelsAndGeometries(frame);
    for (auto* link = frame->termActions; link;)
    {
        auto* next = static_cast<McdTermActionLink*>(link->next);
        link->action(frame);
        delete link;
        link = next;
    }
    delete impl(frame);
}

void MEAPI McdFrameworkRegisterTermAction(McdFrameworkID frame, McdTermAction action)
{
    frame->termActions = new McdTermActionLink{action, frame->termActions};
}

MeI16 MEAPI McdFrameworkGetRegisteredTypeCount(const McdFrameworkID frame)
{
    return (MeI16)frame->geometryRegisteredCount;
}

MeBool MEAPI McdFrameworkTypeIsValid(const McdFrameworkID frame, int id)
{
    return id >= 0 && id < frame->geometryRegisteredCountMax;
}

MeBool MEAPI McdFrameworkTypeIsRegistered(const McdFrameworkID frame, int typeId)
{
    return McdFrameworkTypeIsValid(frame, typeId) && frame->geometryVTableTable[typeId].registered;
}

const char* MEAPI McdFrameworkGetTypeName(const McdFrameworkID frame, int typeId)
{
    if (!McdFrameworkTypeIsRegistered(frame, typeId))
        return nullptr;
    return frame->geometryVTableTable[typeId].name;
}

void MEAPI McdFrameworkRegisterGeometryType(McdFrameworkID frame, McdGeometryType typeID, char* typeName,
                                            McdGeometryDestroyFnPtr destroy, McdGeometryGetAABBFnPtr getAABB,
                                            McdGeometryGetBSphereFnPtr getBSphere,
                                            McdGeometryMaximumPointFnPtr maximumPoint,
                                            McdGeometryGetMassPropertiesFnPtr getMassProperties,
                                            McdGeometryDebugDrawFnPtr debugDraw)
{
    if (!McdFrameworkTypeIsValid(frame, typeID))
    {
        MeWarning(0, "McdFrameworkRegisterGeometryType: type id %d out of range", typeID);
        return;
    }
    McdGeometryVTable& v = frame->geometryVTableTable[typeID];
    if (!v.registered)
        ++frame->geometryRegisteredCount;
    v.destroy = destroy;
    v.getAABB = getAABB;
    v.getBSphere = getBSphere;
    v.maximumPoint = maximumPoint;
    v.getMassProperties = getMassProperties;
    v.debugDraw = debugDraw;
    v.name = typeName;
    v.registered = 1;
}

MeReal MEAPI McdFrameworkGetDefaultContactTolerance(const McdFrameworkID frame)
{
    return frame->mDefaultPadding;
}

void MEAPI McdFrameworkSetDefaultContactTolerance(McdFrameworkID frame, MeReal tol)
{
    frame->mDefaultPadding = tol;
}

McdRequest* MEAPI McdFrameworkGetDefaultRequestPtr(const McdFrameworkID frame)
{
    return frame->defaultRequest;
}

void MEAPI McdFrameworkSetDefaultRequestPtr(McdFrameworkID frame, McdRequest* r)
{
    frame->defaultRequest = r ? r : frame->request;
}

int MEAPI McdFrameworkGetModelCount(McdFrameworkID f)
{
    return f->modelCount;
}

int MEAPI McdFrameworkGetGeometryCount(McdFrameworkID f)
{
    return f->geometryCount;
}

void MEAPI McdFrameworkSetInteractions(McdFramework* frame, int geoType1, int geoType2, McdInteractions* interactions)
{
    if (!McdFrameworkTypeIsValid(frame, geoType1) || !McdFrameworkTypeIsValid(frame, geoType2))
        return;
    frame->interactionTable[geoType1 * frame->geometryRegisteredCountMax + geoType2] = *interactions;
}

McdInteractions* MEAPI McdFrameworkGetInteractions(McdFramework* frame, int geoType1, int geoType2)
{
    if (!McdFrameworkTypeIsValid(frame, geoType1) || !McdFrameworkTypeIsValid(frame, geoType2))
        return nullptr;
    return &frame->interactionTable[geoType1 * frame->geometryRegisteredCountMax + geoType2];
}

void MEAPI McdFrameworkShowTypes(const McdFrameworkID frame)
{
    for (int t = 0; t < frame->geometryRegisteredCountMax; ++t)
        if (McdFrameworkTypeIsRegistered(frame, t))
            MeInfo(0, "geometry type %d: %s", t, frame->geometryVTableTable[t].name);
}

void MEAPI McdFrameworkDestroyAllModelsAndGeometries(McdFrameworkID f)
{
    while (McdModelID m = f->firstModel)
    {
        if (m->mSpace)
            McdSpaceRemoveModel(m);
        McdModelDestroy(m);
    }
    // Unreferenced geometry goes first, so an aggregate releases its elements before they are destroyed.
    while (McdGeometryID g = f->firstGeometry)
    {
        McdGeometryID free = g;
        while (free && McdGeometryGetReferenceCount(free) > 0)
            free = free->next;
        if (!free)
            McdGeometrySetReferenceCount(free = g, 0);
        // A type whose destroy is missing or does not unlink would otherwise stay in the list forever.
        int count = f->geometryCount;
        McdGeometryDestroy(free);
        if (f->geometryCount == count)
            McdGeometryDeinit(free);
    }
}

void MEAPI McdFrameworkSetInteractionsWarned(McdFramework* framework, int geoType1, int geoType2, int count)
{
    if (McdInteractions* i = McdFrameworkGetInteractions(framework, geoType1, geoType2))
        i->warned = (MeBool)count;
}

int MEAPI McdFrameworkGetInteractionsWarned(McdFramework* framework, int geoType1, int geoType2)
{
    McdInteractions* i = McdFrameworkGetInteractions(framework, geoType1, geoType2);
    return i ? i->warned : 0;
}

void MEAPI McdFrameworkPrintInteractionTable(McdFramework* frame)
{
    for (int a = 0; a < frame->geometryRegisteredCountMax; ++a)
    {
        char row[256];
        int n = 0;
        for (int b = 0; b < frame->geometryRegisteredCountMax && n < (int)sizeof(row) - 1; ++b)
            row[n++] = McdFrameworkGetInteractions(frame, a, b)->intersectFn ? 'X' : '.';
        row[n] = 0;
        MeInfo(0, "%2d %s", a, row);
    }
}

void MEAPI McdFrameworkSetLineSegInteraction(McdFramework* frame, int geoType, McdLineSegIntersectFnPtr isectfn)
{
    if (McdFrameworkTypeIsValid(frame, geoType))
        frame->geometryVTableTable[geoType].lineSegIntersect = isectfn;
}

McdLineSegIntersectFnPtr MEAPI McdFrameworkGetLineSegInteraction(McdFramework* frame, int geoType)
{
    return McdFrameworkTypeIsValid(frame, geoType) ? frame->geometryVTableTable[geoType].lineSegIntersect : nullptr;
}

} // extern "C"

void mcdRegisterShapeFn(McdFrameworkID frame, int typeId, McdShapeFn fn)
{
    if (McdFrameworkTypeIsValid(frame, typeId))
        impl(frame)->shapeFns[typeId] = fn;
}
