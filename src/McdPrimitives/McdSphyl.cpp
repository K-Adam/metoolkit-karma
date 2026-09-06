#include <cmath>
#include <new>

#include "Primitive.h"

namespace {

struct SphylImpl
{
    McdSphyl pub;
    btCapsuleShapeZ shape;
};

typedef Primitive<SphylImpl> P;

// The height is the length of the cylindrical part, as for btCapsuleShape.
void setParameters(SphylImpl* s, MeReal r, MeReal h)
{
    s->pub.mRadius = r;
    s->pub.mHalfHeight = h / 2;
    s->shape.~btCapsuleShapeZ();
    new (&s->shape) btCapsuleShapeZ(r, h);
}

} // namespace

extern "C" {

MeI16 MEAPI McdSphylGetTypeId()
{
    return kMcdGeometryTypeSphyl;
}

// Karma treats a sphyl as a cylinder of length h + sqrt(1.8) r. That length gives a bare sphere its exact
// transverse inertia 2/5 r^2, and the original library's volumes and inertias follow it.
MeI16 MEAPI McdSphylGetMassProperties(McdGeometryID g, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    const McdSphyl& s = P::get(g)->pub;
    MeReal r2 = s.mRadius * s.mRadius;
    MeReal length = 2 * s.mHalfHeight + std::sqrt((MeReal)1.8) * s.mRadius;
    MeReal side = (3 * r2 + length * length) / 12;
    mcdSetDiagonalInertia(relTM, inertia, side, side, r2 / 2);
    *volume = (MeReal)ME_PI * r2 * length;
    return 1;
}

void MEAPI McdSphylGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    const McdSphyl& s = P::get(g)->pub;
    center[0] = center[1] = center[2] = 0;
    *radius = s.mHalfHeight + s.mRadius;
}

void MEAPI McdSphylMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    P::maximumPoint(ins, inDir, outPoint);
}

void MEAPI McdSphylRegisterType(McdFramework* frame)
{
    P::registerType(frame, kMcdGeometryTypeSphyl, "Sphyl", McdSphylGetMassProperties, McdSphylMaximumPoint,
                    McdSphylGetBSphere);
}

McdSphylID MEAPI McdSphylCreate(McdFramework* frame, MeReal r, MeReal h)
{
    SphylImpl* s = P::create(frame, kMcdGeometryTypeSphyl, new SphylImpl{McdSphyl{}, btCapsuleShapeZ(r, h)});
    setParameters(s, r, h);
    return &s->pub.m_g;
}

void MEAPI McdSphylSetRadius(McdSphylID s, MeReal r)
{
    setParameters(P::get(s), r, 2 * P::get(s)->pub.mHalfHeight);
}

MeReal MEAPI McdSphylGetRadius(McdSphylID s)
{
    return P::get(s)->pub.mRadius;
}

void MEAPI McdSphylSetHeight(McdSphylID s, MeReal h)
{
    setParameters(P::get(s), P::get(s)->pub.mRadius, h);
}

MeReal MEAPI McdSphylGetHeight(McdSphylID s)
{
    return 2 * P::get(s)->pub.mHalfHeight;
}

void MEAPI McdSphylDestroy(McdGeometryID g)
{
    P::destroy(g);
}

void MEAPI McdSphylUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool tight)
{
    P::updateAABB(ins, finalTM, tight);
}

char* MEAPI McdSphylGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Sphyl");
}

} // extern "C"
