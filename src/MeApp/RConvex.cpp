// Rendering is outside this library. These graphics helpers exist so demo code links, and does nothing.

#include <RConvex.h>

extern "C" {

RGraphic* MEAPI RGraphicConvexCreate(RRender *const rc, const McdConvexMeshID conv, const float color[4],
                                     const MeMatrix4Ptr matrix)
{
    return {};
}

} // extern "C"
