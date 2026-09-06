#pragma once

#include <McdPrimitives.h>

#include "McdImpl.h"

// Maps a local support point of a geometry instance to world space.
template <class LocalFn>
void worldMaximumPoint(McdGeometryInstanceID ins, const MeReal* inDir, MeReal* outPoint, LocalFn local)
{
    btTransform t = toBtTransform(ins->mTM);
    MeVector3 dir, out;
    fromBt(t.getBasis().transpose() * toBt(inDir), dir);
    local(dir, out);
    fromBt(t * toBt(out), outPoint);
}

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
        auto* convex = static_cast<btConvexShape*>(shape(ins->mGeometry));
        worldMaximumPoint(ins, inDir, outPoint,
                          [convex](MeReal* dir, MeReal* out) { fromBt(convex->localGetSupportingVertex(toBt(dir)), out); });
    }

    static void registerType(McdFramework* frame, int typeId, const char* name,
                             McdGeometryGetMassPropertiesFnPtr massProperties,
                             McdGeometryMaximumPointFnPtr maxPoint = maximumPoint,
                             McdGeometryGetBSphereFnPtr bsphere = getBSphere)
    {
        McdFrameworkRegisterGeometryType(frame, (McdGeometryType)typeId, const_cast<char*>(name), destroy, updateAABB,
                                         bsphere, maxPoint, massProperties, nullptr);
        mcdRegisterShapeFn(frame, typeId, shape);
    }
};
