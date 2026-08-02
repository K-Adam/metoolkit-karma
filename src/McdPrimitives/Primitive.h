#pragma once

#include "McdImpl.h"

// Geometry vtable entries shared by every primitive whose public struct is followed by a Bullet shape.
template <class Impl>
struct Primitive
{
    static Impl* get(McdGeometryID g) { return reinterpret_cast<Impl*>(g); }

    static btCollisionShape* shape(McdGeometryID g) { return &get(g)->shape; }

    static Impl* create(McdFramework* frame, int typeId, Impl* p)
    {
        McdGeometryInit(&p->pub.m_g, frame, (MeI16)typeId);
        return p;
    }

    static void MEAPI destroy(McdGeometryID g)
    {
        McdGeometryDeinit(g);
        delete get(g);
    }

    static void MEAPI updateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool tight)
    {
        (void)tight;
        btVector3 lo, hi;
        shape(ins->mGeometry)->getAabb(toBtTransform(&finalTM[0][0]), lo, hi);
        fromBt(lo, ins->min);
        fromBt(hi, ins->max);
    }

    static void MEAPI getBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
    {
        btVector3 c;
        btScalar r;
        shape(g)->getBoundingSphere(c, r);
        fromBt(c, center);
        *radius = r;
    }

    static void MEAPI maximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
    {
        btTransform t = toBtTransform(ins->mTM);
        auto* convex = static_cast<btConvexShape*>(shape(ins->mGeometry));
        btVector3 local = convex->localGetSupportingVertex(t.getBasis().transpose() * toBt(inDir));
        fromBt(t * local, outPoint);
    }

    static void registerType(McdFramework* frame, int typeId, const char* name,
                             McdGeometryGetMassPropertiesFnPtr massProperties, bool convex = true)
    {
        McdFrameworkRegisterGeometryType(frame, (McdGeometryType)typeId, const_cast<char*>(name), destroy, updateAABB,
                                         getBSphere, convex ? maximumPoint : nullptr, massProperties, nullptr);
        mcdRegisterShapeFn(frame, typeId, shape);
    }
};

// Mass properties are reported for unit density: inertia is per unit mass and volume is returned separately.
inline void setDiagonalInertia(MeMatrix4 relTM, MeMatrix3 inertia, MeReal ix, MeReal iy, MeReal iz)
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            relTM[r][c] = r == c ? (MeReal)1 : (MeReal)0;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            inertia[r][c] = 0;
    inertia[0][0] = ix;
    inertia[1][1] = iy;
    inertia[2][2] = iz;
}
