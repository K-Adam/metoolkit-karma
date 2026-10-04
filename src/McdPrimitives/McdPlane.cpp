#include "Primitive.h"

namespace {

struct PlaneImpl
{
    McdPlane pub;
    btStaticPlaneShape shape;
};

typedef Primitive<PlaneImpl> P;

// The plane bounds the half-space below it, so a segment starting below it hits at distance zero.
int MEAPI intersectLineSegment(const McdModelID model, MeReal* const inOrig, MeReal* const inDest,
                               McdLineSegIntersectResult* info)
{
    MeMatrix4Ptr tm = McdModelGetTransformPtr(model);
    McdGeometryID g = McdModelGetGeometry(model);
    MeReal h0 = McdPlaneGetDistanceToPoint(g, tm, inOrig), h1 = McdPlaneGetDistanceToPoint(g, tm, inDest);
    if (h0 > 0 && h1 >= 0)
        return 0;
    MeReal t = h0 > 0 ? h0 / (h0 - h1) : 0;
    btVector3 from = toBt(inOrig), to = toBt(inDest);
    info->model = model;
    fromBt(from + (to - from) * t, info->position);
    McdPlaneGetNormal(g, tm, info->normal);
    info->distance = t * (to - from).length();
    return 1;
}

} // namespace

extern "C" {

MeI16 MEAPI McdPlaneGetTypeId()
{
    return kMcdGeometryTypePlane;
}

MeI16 MEAPI McdPlaneGetMassProperties(McdGeometryID, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    mcdSetDiagonalInertia(relTM, inertia, 0, 0, 0);
    *volume = 0;
    return 0;
}

void MEAPI McdPlaneRegisterType(McdFramework* frame)
{
    P::registerType(frame, kMcdGeometryTypePlane, "Plane", McdPlaneGetMassProperties, McdPlaneMaximumPoint);
    McdFrameworkSetLineSegInteraction(frame, kMcdGeometryTypePlane, intersectLineSegment);
}

// The plane passes through the model origin with its normal along the model's z axis.
McdPlaneID MEAPI McdPlaneCreate(McdFramework* frame)
{
    PlaneImpl* p = P::create(frame, kMcdGeometryTypePlane, new PlaneImpl{McdPlane{}, btStaticPlaneShape(btVector3(0, 0, 1), 0)});
    return &p->pub.m_g;
}

void MEAPI McdPlaneGetNormal(const McdGeometry*, const MeMatrix4 tm, MeVector3 normal)
{
    normal[0] = tm[2][0];
    normal[1] = tm[2][1];
    normal[2] = tm[2][2];
}

MeReal MEAPI McdPlaneGetDistanceToPoint(const McdGeometry* g, const MeMatrix4 tm, const MeVector3 point)
{
    MeVector3 n;
    McdPlaneGetNormal(g, tm, n);
    return n[0] * (point[0] - tm[3][0]) + n[1] * (point[1] - tm[3][1]) + n[2] * (point[2] - tm[3][2]);
}

void MEAPI McdPlaneDestroy(McdGeometryID g)
{
    P::destroy(g);
}

void MEAPI McdPlaneUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool tight)
{
    P::updateAABB(ins, finalTM, tight);
}

void MEAPI McdPlaneGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    P::getBSphere(g, center, radius);
}

// The support point of the solid half-space lies at infinity unless the direction points straight out of it.
void MEAPI McdPlaneMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    worldMaximumPoint(ins, inDir, outPoint, [](MeReal* d, MeReal* out) {
        out[0] = d[0] * MEINFINITY;
        out[1] = d[1] * MEINFINITY;
        out[2] = d[2] < 0 ? d[2] * MEINFINITY : 0;
    });
}

char* MEAPI McdPlaneGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Plane");
}

void MEAPI McdPlaneDebugDraw(const McdGeometryID, const MeMatrix4, const MeReal[3])
{
}

} // extern "C"
