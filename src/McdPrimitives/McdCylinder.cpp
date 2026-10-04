#include <cmath>
#include <new>

#include "Primitive.h"

namespace {

struct CylinderImpl
{
    McdCylinder pub;
    btCylinderShapeZ shape;
};

typedef Primitive<CylinderImpl> P;

void setParameters(CylinderImpl* c, MeReal r, MeReal h)
{
    c->pub.mR = r;
    c->pub.mRz = h / 2;
    c->pub.mSphereRadius = std::sqrt(r * r + c->pub.mRz * c->pub.mRz);
    // btCylinderShape has no extent setter. Rebuilding in place keeps the shape address models hold.
    c->shape.~btCylinderShapeZ();
    new (&c->shape) btCylinderShapeZ(btVector3(r, r, h / 2));
}

} // namespace

extern "C" {

MeI16 MEAPI McdCylinderGetTypeId()
{
    return kMcdGeometryTypeCylinder;
}

MeI16 MEAPI McdCylinderGetMassProperties(McdGeometryID g, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    const McdCylinder& c = P::get(g)->pub;
    MeReal r2 = c.mR * c.mR, h = 2 * c.mRz;
    MeReal side = (3 * r2 + h * h) / 12;
    mcdSetDiagonalInertia(relTM, inertia, side, side, r2 / 2);
    *volume = (MeReal)ME_PI * r2 * h;
    return 1;
}

void MEAPI McdCylinderGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    center[0] = center[1] = center[2] = 0;
    *radius = P::get(g)->pub.mSphereRadius;
}

void MEAPI McdCylinderMaximumPointLocal(McdGeometry* g, MeReal* const inDir, MeReal* const outPoint)
{
    const McdCylinder& c = P::get(g)->pub;
    MeReal xy = std::sqrt(inDir[0] * inDir[0] + inDir[1] * inDir[1]);
    outPoint[0] = xy > 0 ? c.mR * inDir[0] / xy : c.mR;
    outPoint[1] = xy > 0 ? c.mR * inDir[1] / xy : 0;
    outPoint[2] = inDir[2] < 0 ? -c.mRz : c.mRz;
}

void MEAPI McdCylinderMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    worldMaximumPoint(ins, inDir, outPoint,
                      [ins](MeReal* dir, MeReal* out) { McdCylinderMaximumPointLocal(ins->mGeometry, dir, out); });
}

void MEAPI McdCylinderRegisterType(McdFramework* frame)
{
    P::registerType(frame, kMcdGeometryTypeCylinder, "Cylinder", McdCylinderGetMassProperties,
                    McdCylinderMaximumPoint, McdCylinderGetBSphere);
}

McdCylinderID MEAPI McdCylinderCreate(McdFramework* frame, MeReal r, MeReal h)
{
    CylinderImpl* c = P::create(frame, kMcdGeometryTypeCylinder,
                                new CylinderImpl{McdCylinder{}, btCylinderShapeZ(btVector3(r, r, h / 2))});
    setParameters(c, r, h);
    return &c->pub.m_g;
}

void MEAPI McdCylinderSetGeometricalParameters(McdCylinderID g, MeReal inRadius, MeReal inHeight)
{
    setParameters(P::get(g), inRadius, inHeight);
}

void MEAPI McdCylinderSetRadius(McdCylinderID g, MeReal r)
{
    setParameters(P::get(g), r, 2 * P::get(g)->pub.mRz);
}

void MEAPI McdCylinderSetHeight(McdCylinderID g, MeReal h)
{
    setParameters(P::get(g), P::get(g)->pub.mR, h);
}

MeReal MEAPI McdCylinderGetRadius(McdCylinderID g)
{
    return P::get(g)->pub.mR;
}

MeReal MEAPI McdCylinderGetHeight(McdCylinderID g)
{
    return 2 * P::get(g)->pub.mRz;
}

MeReal MEAPI McdCylinderGetHalfHeight(McdCylinderID g)
{
    return P::get(g)->pub.mRz;
}

// bounds is {minX, maxX, minY, maxY}, the layout the original library writes.
void MEAPI McdCylinderGetXYAABB(McdGeometry* g, MeMatrix4 tm, MeReal bounds[4])
{
    const McdCylinder& c = P::get(g)->pub;
    for (int axis = 0; axis < 2; ++axis)
    {
        MeReal a = tm[2][axis];
        MeReal half = c.mR * std::sqrt(std::fmax(MeReal(0), 1 - a * a)) + c.mRz * std::fabs(a);
        bounds[2 * axis] = tm[3][axis] - half;
        bounds[2 * axis + 1] = tm[3][axis] + half;
    }
}

void MEAPI McdCylinderDestroy(McdGeometryID g)
{
    P::destroy(g);
}

void MEAPI McdCylinderUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool tight)
{
    P::updateAABB(ins, finalTM, tight);
}

char* MEAPI McdCylinderGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Cylinder");
}

void MEAPI McdCylinderDebugDraw(const McdGeometryID, const MeMatrix4, const MeReal[3])
{
}

} // extern "C"
