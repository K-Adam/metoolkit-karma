#include <Mst.h>

namespace {

void MEAPI clearPairResponse(const MdtContactGroupID g)
{
    if (auto* p = static_cast<McdModelPair*>(MdtContactGroupGetGenerator(g)))
        p->responseData = nullptr;
}

// A sleeping body's model stops being updated by the space, and wakes with it.
void MEAPI freezeModel(const MdtBodyID b)
{
    auto* m = static_cast<McdModelID>(b->model);
    if (m && McdModelGetSpace(m))
        McdSpaceFreezeModel(m);
}

void MEAPI unfreezeModel(const MdtBodyID b)
{
    auto* m = static_cast<McdModelID>(b->model);
    if (m && McdModelGetSpace(m))
        McdSpaceUnfreezeModel(m);
}

} // namespace

extern "C" {

void MEAPI MstSetWorldHandlers(const MdtWorldID world)
{
    world->contactGroupDestroyCallback = clearPairResponse;
    world->bodyDisableCallback = freezeModel;
    world->bodyEnableCallback = unfreezeModel;
}

void MEAPI MstAutoSetMassProperties(const MdtBodyID body, const McdModelID model, const MeReal density)
{
    MeMatrix4 relTM;
    MeMatrix3 inertia;
    MeReal volume = 0;
    McdGeometryGetMassProperties(McdModelGetGeometry(model), relTM, inertia, &volume);
    if (volume <= 0)
    {
        MeWarning(0, "MstAutoSetMassProperties: geometry has no volume");
        return;
    }
    MeReal mass = density * volume;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            inertia[r][c] *= mass;
    MdtBodySetMass(body, mass);
    MdtBodySetInertiaTensor(body, inertia);
}

McdModelID MEAPI MstModelAndBodyCreate(const MstUniverseID u, const McdGeometryID g, const MeReal density)
{
    McdModelID m = McdModelCreate(g);
    MdtBodyID b = m ? MdtBodyCreate(u->world) : nullptr;
    if (!b)
    {
        McdModelDestroy(m);
        return nullptr;
    }
    McdModelSetBody(m, b);
    MstAutoSetMassProperties(b, m, density);
    McdSpaceInsertModel(u->space, m);
    MdtBodyEnable(b);
    return m;
}

void MEAPI MstModelAndBodyDestroy(const McdModelID m)
{
    MdtBodyID b = McdModelGetBody(m);
    McdSpaceRemoveModel(m);
    McdModelDestroy(m);
    MdtBodyDestroy(b);
}

McdModelID MEAPI MstFixedModelCreate(const MstUniverseID u, const McdGeometryID g, MeMatrix4Ptr transformation)
{
    McdModelID m = McdModelCreate(g);
    if (!m)
        return nullptr;
    McdModelSetTransformPtr(m, transformation);
    McdSpaceInsertModel(u->space, m);
    McdSpaceUpdateModel(m);
    McdSpaceFreezeModel(m);
    return m;
}

void MEAPI MstFixedModelDestroy(const McdModelID m)
{
    McdSpaceRemoveModel(m);
    McdModelDestroy(m);
}

} // extern "C"
