#include <Mst.h>

extern "C" {

const MstUniverseSizes MstUniverseDefaultSizes = {
    100,  // dynamicBodiesMaxCount
    500,  // dynamicConstraintsMaxCount
    0,    // collisionUserGeometryTypesMaxCount
    100,  // collisionModelsMaxCount
    1000, // collisionPairsMaxCount
    100,  // collisionGeometryInstancesMaxCount
    10,   // materialsMaxCount
    1,    // lengthScale
    1,    // massScale
};

MstUniverseID MEAPI MstUniverseCreate(const MstUniverseSizes* const sizes)
{
    auto* u = new MstUniverse{};
    u->sizes = *sizes;
    u->frame = McdInit(McdPrimitivesGetTypeCount() + (int)sizes->collisionUserGeometryTypesMaxCount,
                       (int)sizes->collisionModelsMaxCount, (int)sizes->collisionGeometryInstancesMaxCount,
                       sizes->lengthScale);
    McdPrimitivesRegisterTypes(u->frame);
    McdPrimitivesRegisterInteractions(u->frame);
    u->space = McdSpaceAxisSortCreate(u->frame, McdAllAxes, (int)sizes->collisionModelsMaxCount,
                                      (int)sizes->collisionPairsMaxCount);
    u->world = MdtWorldCreate(sizes->dynamicBodiesMaxCount, sizes->dynamicConstraintsMaxCount, sizes->lengthScale,
                              sizes->massScale);
    MstSetWorldHandlers(u->world);
    u->bridge = MstBridgeCreate(u->frame, sizes->materialsMaxCount);
    return u;
}

void MEAPI MstUniverseDestroy(const MstUniverseID u)
{
    // The world goes first: destroying its contact groups clears the pairs that still point at them.
    MdtWorldDestroy(u->world);
    McdSpaceDestroy(u->space);
    while (McdModelID m = u->frame->firstModel)
        McdModelDestroy(m);
    while (McdGeometryID g = u->frame->firstGeometry)
    {
        McdGeometrySetReferenceCount(g, 0);
        McdGeometryDestroy(g);
    }
    McdTerm(u->frame);
    MstBridgeDestroy(u->bridge);
    delete u;
}

void MEAPI MstUniverseStep(const MstUniverseID u, const MeReal step)
{
    McdSpaceUpdateAll(u->space);
    MstBridgeUpdateContacts(u->bridge, u->space, u->world);
    MdtWorldStep(u->world, step);
}

MdtWorldID MEAPI MstUniverseGetWorld(const MstUniverseID u)
{
    return u->world;
}

McdSpaceID MEAPI MstUniverseGetSpace(const MstUniverseID u)
{
    return u->space;
}

MstBridgeID MEAPI MstUniverseGetBridge(const MstUniverseID u)
{
    return u->bridge;
}

McdFrameworkID MEAPI MstUniverseGetFramework(const MstUniverseID u)
{
    return u->frame;
}

} // extern "C"
