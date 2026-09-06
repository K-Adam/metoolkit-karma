#include "Primitive.h"

namespace {

struct SphereImpl
{
    McdSphere pub;
    btSphereShape shape;
};

typedef Primitive<SphereImpl> P;

} // namespace

extern "C" {

MeI16 MEAPI McdSphereGetTypeId()
{
    return kMcdGeometryTypeSphere;
}

MeI16 MEAPI McdSphereGetMassProperties(McdGeometryID g, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    MeReal r = P::get(g)->pub.mRadius;
    MeReal i = (MeReal)0.4 * r * r;
    mcdSetDiagonalInertia(relTM, inertia, i, i, i);
    *volume = (MeReal)(4.0 / 3.0 * ME_PI) * r * r * r;
    return 1;
}

void MEAPI McdSphereRegisterType(McdFramework* frame)
{
    P::registerType(frame, kMcdGeometryTypeSphere, "Sphere", McdSphereGetMassProperties);
}

McdSphereID MEAPI McdSphereCreate(McdFramework* frame, MeReal r)
{
    SphereImpl* s = P::create(frame, kMcdGeometryTypeSphere, new SphereImpl{McdSphere{}, btSphereShape(r)});
    s->pub.mRadius = r;
    return &s->pub.m_g;
}

void MEAPI McdSphereSetRadius(McdSphereID g, MeReal r)
{
    P::get(g)->pub.mRadius = r;
    P::get(g)->shape.setUnscaledRadius(r);
}

MeReal MEAPI McdSphereGetRadius(McdSphereID g)
{
    return P::get(g)->pub.mRadius;
}

void MEAPI McdSphereDestroy(McdGeometryID g)
{
    P::destroy(g);
}

void MEAPI McdSphereUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool tight)
{
    P::updateAABB(ins, finalTM, tight);
}

void MEAPI McdSphereGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    P::getBSphere(g, center, radius);
}

void MEAPI McdSphereMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    P::maximumPoint(ins, inDir, outPoint);
}

char* MEAPI McdSphereGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Sphere");
}

} // extern "C"
