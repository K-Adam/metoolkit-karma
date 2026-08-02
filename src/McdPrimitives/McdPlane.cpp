#include "Primitive.h"

namespace {

struct PlaneImpl
{
    McdPlane pub;
    btStaticPlaneShape shape;
};

typedef Primitive<PlaneImpl> P;

} // namespace

extern "C" {

MeI16 MEAPI McdPlaneGetTypeId()
{
    return kMcdGeometryTypePlane;
}

MeI16 MEAPI McdPlaneGetMassProperties(McdGeometryID, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    setDiagonalInertia(relTM, inertia, 0, 0, 0);
    *volume = 0;
    return 0;
}

void MEAPI McdPlaneRegisterType(McdFramework* frame)
{
    P::registerType(frame, kMcdGeometryTypePlane, "Plane", McdPlaneGetMassProperties, false);
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

char* MEAPI McdPlaneGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Plane");
}

} // extern "C"
