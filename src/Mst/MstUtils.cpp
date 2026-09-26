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

// The geometry reports the inertia of unit mass.
bool unitInertia(McdModelID model, MeMatrix3 inertia, MeReal* volume)
{
    MeMatrix4 relTM;
    McdGeometryGetMassProperties(McdModelGetGeometry(model), relTM, inertia, volume);
    if (*volume > 0)
        return true;
    MeWarning(0, "Mst: geometry has no volume, mass properties left unchanged");
    return false;
}

void setScaledInertia(MdtBodyID body, MeMatrix3 inertia, MeReal mass)
{
    MeMatrix3Scale(inertia, mass);
    MdtBodySetInertiaTensor(body, inertia);
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
    MeReal volume = 0;
    MeMatrix3 inertia;
    if (!unitInertia(model, inertia, &volume))
        return;
    MeReal mass = density * volume;
    MdtBodySetMass(body, mass);
    setScaledInertia(body, inertia, mass);
}

// Keeps the body's mass and takes the shape of the inertia from the geometry.
void MEAPI MstAutoSetInertialTensor(const McdModelID model)
{
    MdtBodyID body = McdModelGetBody(model);
    MeReal volume = 0;
    MeMatrix3 inertia;
    if (body && unitInertia(model, inertia, &volume))
        setScaledInertia(body, inertia, MdtBodyGetMass(body));
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
