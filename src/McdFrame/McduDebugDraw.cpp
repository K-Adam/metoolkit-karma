// Rendering is outside this library. Debug drawing exists so demo code links, and does nothing.

#include <McduDebugDraw.h>

extern "C" {

void MEAPI McduDebugDrawSphere(const McdSphereID, const MeMatrix4, const MeReal[3])
{
}

void MEAPI McduDebugDrawBox(const McdBoxID, const MeMatrix4, const MeReal[3])
{
}

void MEAPI McduDebugDrawCylinder(const McdCylinderID, const MeMatrix4, const MeReal[3])
{
}

void MEAPI McduDebugDrawConvex(const McdConvexMeshID, const MeMatrix4, const MeReal[3])
{
}

void MEAPI McduDebugDrawAggregate(const McdAggregateID, const MeMatrix4, const MeReal[3])
{
}

void MEAPI McduDebugDrawAABB(const MeVector3, const MeVector3, const MeReal[3])
{
}

void MEAPI McduDebugDrawGeometry(const McdGeometryID, const MeMatrix4Ptr, const MeReal[3])
{
}

void MEAPI McduDebugDrawModel(const McdModelID, McduDebugDrawFlags, const MeReal[3])
{
}

void MEAPI McduDebugDrawSpace(const McdSpaceID, McduDebugDrawFlags, const MeReal[3])
{
}

} // extern "C"
