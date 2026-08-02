#pragma once

#include <btBulletCollisionCommon.h>

#include <map>
#include <vector>

#include <Mcd.h>
#include <McdModelPair.h>

#include "BulletConvert.h"

// Built-in geometry types collide through Bullet. A type without a shape function relies on the
// McdInteractions table instead, which is how user geometry plug-ins work in the original.
typedef btCollisionShape* (*McdShapeFn)(McdGeometryID);

struct McdFrameworkImpl
{
    McdFramework pub{};
    btDefaultCollisionConfiguration config;
    btCollisionDispatcher dispatcher{&config};
    std::vector<McdGeometryVTable> vtables;
    std::vector<McdShapeFn> shapeFns;
    std::vector<McdInteractions> interactions;
    McdRequest request{};
};

inline McdFrameworkImpl* impl(McdFrameworkID f) { return reinterpret_cast<McdFrameworkImpl*>(f); }

void mcdRegisterShapeFn(McdFrameworkID frame, int typeId, McdShapeFn fn);
btCollisionShape* mcdGeometryShape(McdGeometryID g);

struct McdModelImpl
{
    McdModel pub{};
    btCollisionObject object;
    MeMatrix4 ownTM;
    bool frozen = false;
};

inline McdModelImpl* impl(McdModelID m) { return reinterpret_cast<McdModelImpl*>(m); }

void mcdModelSync(McdModelID m);

inline int mcdContactLimit(McdModelPair* p, McdIntersectResult* r)
{
    int limit = r->contactMaxCount;
    if (p->request && p->request->contactMaxCount > 0 && p->request->contactMaxCount < limit)
        limit = p->request->contactMaxCount;
    return limit;
}

inline void mcdAddContact(McdIntersectResult* r, const btVector3& position, const btVector3& normal, MeReal separation)
{
    McdContact& c = r->contacts[r->contactCount++];
    fromBt(position, c.position);
    fromBt(normal, c.normal);
    c.separation = separation;
    c.dims = 0;
    c.element1.ptr = c.element2.ptr = nullptr;
}

struct _McdSpace
{
    McdFrameworkImpl* frame;
    btDbvtBroadphase broadphase;
    btCollisionWorld world;
    std::map<std::pair<McdModelID, McdModelID>, McdModelPair*> pairs;
    std::vector<McdModelPair*> goodbye, hello, staying;
    std::vector<McdModelPair*> removed;
    int nextModelId = 0;
    int changing = 0;
    void* userData = nullptr;

    explicit _McdSpace(McdFrameworkImpl* f) : frame(f), world(&f->dispatcher, &broadphase, &f->config) {}
};
