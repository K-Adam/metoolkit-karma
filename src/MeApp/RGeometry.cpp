// Rendering is outside this library. These graphics helpers exist so demo code links, and does nothing.

#include <RGeometry.h>

extern "C" {

RGraphic* MEAPI RGraphicAggregateCreate(RRender *rc, McdAggregateID g, float color[4], MeMatrix4Ptr tm)
{
    return {};
}

RGraphic* MEAPI RGraphicCreateFromGeometry(RRender *rc, McdGeometryID geom, float color[4])
{
    return {};
}

RGraphic* MEAPI RGraphicCreateFromModel(RRender *rc, McdModelID m, float color[4])
{
    return {};
}

} // extern "C"
