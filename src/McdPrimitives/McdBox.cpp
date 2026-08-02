#include <cmath>
#include <new>

#include "Primitive.h"

namespace {

struct BoxImpl
{
    McdBox pub;
    btBoxShape shape;
};

typedef Primitive<BoxImpl> P;

void setRadii(McdBox& b, MeReal dx, MeReal dy, MeReal dz)
{
    b.mR[0] = dx / 2;
    b.mR[1] = dy / 2;
    b.mR[2] = dz / 2;
    b.mRadius = std::sqrt(b.mR[0] * b.mR[0] + b.mR[1] * b.mR[1] + b.mR[2] * b.mR[2]);
}

} // namespace

extern "C" {

MeI16 MEAPI McdBoxGetTypeId()
{
    return kMcdGeometryTypeBox;
}

MeI16 MEAPI McdBoxGetMassProperties(McdGeometryID g, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    const MeReal* r = P::get(g)->pub.mR;
    MeReal x2 = 4 * r[0] * r[0], y2 = 4 * r[1] * r[1], z2 = 4 * r[2] * r[2];
    setDiagonalInertia(relTM, inertia, (y2 + z2) / 12, (x2 + z2) / 12, (x2 + y2) / 12);
    *volume = 8 * r[0] * r[1] * r[2];
    return 1;
}

void MEAPI McdBoxRegisterType(McdFramework* frame)
{
    P::registerType(frame, kMcdGeometryTypeBox, "Box", McdBoxGetMassProperties);
}

McdBoxID MEAPI McdBoxCreate(McdFramework* frame, MeReal dx, MeReal dy, MeReal dz)
{
    McdBox pub{};
    setRadii(pub, dx, dy, dz);
    BoxImpl* b = P::create(frame, kMcdGeometryTypeBox, new BoxImpl{pub, btBoxShape(toBt(pub.mR))});
    return &b->pub.m_g;
}

void MEAPI McdBoxSetDimensions(McdGeometryID g, MeReal dx, MeReal dy, MeReal dz)
{
    BoxImpl* b = P::get(g);
    setRadii(b->pub, dx, dy, dz);
    // btBoxShape has no extent setter. Rebuilding in place keeps the shape address models hold.
    b->shape.~btBoxShape();
    new (&b->shape) btBoxShape(toBt(b->pub.mR));
}

void MEAPI McdBoxGetDimensions(McdGeometryID g, MeReal* dx, MeReal* dy, MeReal* dz)
{
    const MeReal* r = P::get(g)->pub.mR;
    *dx = 2 * r[0];
    *dy = 2 * r[1];
    *dz = 2 * r[2];
}

MeReal* MEAPI McdBoxGetRadii(McdBoxID g)
{
    return P::get(g)->pub.mR;
}

void MEAPI McdBoxDestroy(McdGeometryID g)
{
    P::destroy(g);
}

void MEAPI McdBoxUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool tight)
{
    P::updateAABB(ins, finalTM, tight);
}

void MEAPI McdBoxGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    P::getBSphere(g, center, radius);
}

void MEAPI McdBoxMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    P::maximumPoint(ins, inDir, outPoint);
}

char* MEAPI McdBoxGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Box");
}

} // extern "C"
