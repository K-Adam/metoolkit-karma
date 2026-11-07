#include <algorithm>
#include <vector>

#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include "McdImpl.h"

// An aggregate is a btCompoundShape over its elements' shapes, so pairs without a registered interaction collide
// through Bullet. The registered interactions instead test each element as a model of its own, which gives the
// elements the same contacts as standalone primitives.

namespace {

struct AggregateImpl
{
    McdAggregate pub;
    std::vector<McdAggregateElement> elements;
    btCompoundShape shape{false};
};

AggregateImpl* get(McdGeometryID g) { return reinterpret_cast<AggregateImpl*>(g); }

btCollisionShape* shapeOf(McdGeometryID g) { return &get(g)->shape; }

void rebuildShape(AggregateImpl* a)
{
    while (a->shape.getNumChildShapes() > 0)
        a->shape.removeChildShapeByIndex(a->shape.getNumChildShapes() - 1);
    for (McdAggregateElement& e : a->elements)
        if (e.mGeometry)
            a->shape.addChildShape(toBtTransform(e.mRelTM), mcdGeometryShape(e.mGeometry));
}

// Element transforms may be edited through McdAggregateGetElementTransformPtr, so they are reread here.
void syncChildren(McdGeometryInstanceID ins, const btTransform& tm)
{
    AggregateImpl* a = get(ins->mGeometry);
    McdGeometryInstance** link = &ins->child;
    McdGeometryInstance* prev = nullptr;
    int shapeIndex = 0;
    for (McdAggregateElement& e : a->elements)
    {
        if (!e.mGeometry)
            continue;
        btTransform rel = toBtTransform(e.mRelTM);
        a->shape.updateChildTransform(shapeIndex++, rel, false);
        if (!*link)
            *link = &(new McdChildInstance())->ins;
        auto* child = reinterpret_cast<McdChildInstance*>(*link);
        if (child->ins.mGeometry != e.mGeometry)
            mcdFreeChildInstances(&child->ins);
        child->ins.mGeometry = e.mGeometry;
        child->ins.mTM = child->tm;
        child->ins.mMaterial = ins->mMaterial;
        child->ins.parent = ins;
        child->ins.prev = prev;
        fromBtTransform(tm * rel, child->tm);
        McdGeometryInstanceUpdateAABB(&child->ins, child->tm, 1);
        prev = &child->ins;
        link = &child->ins.next;
    }
    McdGeometryInstance holder{};
    holder.child = *link;
    mcdFreeChildInstances(&holder);
    *link = nullptr;
    a->shape.recalculateLocalAabb();
}

// A stand-in model for one element of an aggregate model, valid while the aggregate model is unchanged.
struct ElementModel
{
    McdModelImpl m;

    ElementModel(McdModelID aggregate, McdGeometryInstance* child)
    {
        McdModel& pub = m.pub;
        pub.frame = aggregate->frame;
        pub.mSpace = aggregate->mSpace;
        pub.linearVelocity = aggregate->linearVelocity;
        pub.angularVelocity = aggregate->angularVelocity;
        pub.mPadding = aggregate->mPadding;
        pub.mBody = aggregate->mBody;
        pub.mData = aggregate->mData;
        pub.mInstance = *child;
        m.object.setCollisionShape(mcdGeometryShape(child->mGeometry));
        m.object.setWorldTransform(toBtTransform(child->mTM));
        m.object.setUserPointer(&pub);
    }
};

int MEAPI intersectAggregate(McdModelPair* p, McdIntersectResult* r)
{
    bool aggregateFirst = McdModelGetGeometryType(p->model1) == kMcdGeometryTypeAggregate;
    McdModelID aggregate = aggregateFirst ? p->model1 : p->model2;
    McdModelID other = aggregateFirst ? p->model2 : p->model1;
    int limit = mcdContactLimit(p, r);
    btVector3 sum(0, 0, 0);
    for (McdGeometryInstance* child = aggregate->mInstance.child; child && r->contactCount < limit; child = child->next)
    {
        if (!McdGeometryInstanceOverlap(child, &other->mInstance))
            continue;
        ElementModel element(aggregate, child);
        McdModelPair sub{};
        sub.model1 = aggregateFirst ? &element.m.pub : other;
        sub.model2 = aggregateFirst ? other : &element.m.pub;
        sub.phase = kMcdFFStateStaying;
        sub.request = p->request;
        McdIntersectResult part{};
        part.contacts = r->contacts + r->contactCount;
        part.contactMaxCount = limit - r->contactCount;
        if (mcdIntersectOnce(&sub, &part))
        {
            r->contactCount += part.contactCount;
            sum += toBt(part.normal) * (btScalar)part.contactCount;
        }
    }
    mcdSetMeanNormal(r, sum);
    return r->contactCount > 0;
}

// At most one hit per aggregate, where the segment first enters any element.
int MEAPI intersectAggregateLineSegment(const McdModelID model, MeReal* const inOrig, MeReal* const inDest,
                                        McdLineSegIntersectResult* info)
{
    int found = 0;
    for (McdGeometryInstance* child = model->mInstance.child; child; child = child->next)
    {
        ElementModel element(model, child);
        McdLineSegIntersectResult hit;
        if (McdLineSegIntersect(&element.m.pub, inOrig, inDest, &hit) && (!found || hit.distance < info->distance))
        {
            *info = hit;
            found = 1;
        }
    }
    if (found)
        info->model = model;
    return found;
}

} // namespace

extern "C" {

MeI16 MEAPI McdAggregateGetTypeId()
{
    return kMcdGeometryTypeAggregate;
}

void MEAPI McdAggregateRegisterType(McdFramework* frame)
{
    McdFrameworkRegisterGeometryType(frame, kMcdGeometryTypeAggregate, const_cast<char*>("Aggregate"),
                                     McdAggregateDestroy, McdAggregateUpdateAABB, McdAggregateGetBSphere,
                                     McdAggregateMaximumPoint, McdAggregateGetMassProperties, nullptr);
    mcdRegisterShapeFn(frame, kMcdGeometryTypeAggregate, shapeOf);
}

McdAggregateID MEAPI McdAggregateCreate(McdFramework* frame, int maxChildren)
{
    auto* a = new AggregateImpl{};
    McdGeometryInit(&a->pub.m_g, frame, kMcdGeometryTypeAggregate);
    a->elements.assign((size_t)std::max(maxChildren, 0), McdAggregateElement{});
    a->pub.elementTable = a->elements.data();
    a->pub.elementCountMax = maxChildren;
    return &a->pub.m_g;
}

void MEAPI McdAggregateDestroy(McdGeometryID g)
{
    AggregateImpl* a = get(g);
    for (McdAggregateElement& e : a->elements)
        if (e.mGeometry)
            McdGeometryDecrementReferenceCount(e.mGeometry);
    McdGeometryDeinit(g);
    delete a;
}

int MEAPI McdAggregateAddElement(McdAggregateID g, McdGeometryID element, MeMatrix4 relTM)
{
    if (!element)
        return -1;
    AggregateImpl* a = get(g);
    for (int i = 0; i < (int)a->elements.size(); ++i)
    {
        McdAggregateElement& e = a->elements[i];
        if (e.mGeometry)
            continue;
        e.mGeometry = element;
        std::copy(&relTM[0][0], &relTM[0][0] + 16, &e.mRelTM[0][0]);
        McdGeometryIncrementReferenceCount(element);
        ++a->pub.elementCount;
        rebuildShape(a);
        return i;
    }
    MeWarning(0, "McdAggregateAddElement: aggregate is full");
    return -1;
}

void MEAPI McdAggregateRemoveElement(McdAggregateID g, int i)
{
    AggregateImpl* a = get(g);
    if (i < 0 || i >= (int)a->elements.size() || !a->elements[i].mGeometry)
        return;
    McdGeometryDecrementReferenceCount(a->elements[i].mGeometry);
    a->elements[i].mGeometry = nullptr;
    --a->pub.elementCount;
    rebuildShape(a);
}

int MEAPI McdAggregateGetElementCount(McdAggregateID g)
{
    return get(g)->pub.elementCount;
}

int MEAPI McdAggregateGetElementCountMax(McdAggregateID g)
{
    return get(g)->pub.elementCountMax;
}

McdGeometryID MEAPI McdAggregateGetElementGeometry(McdAggregateID g, int i)
{
    AggregateImpl* a = get(g);
    return i >= 0 && i < (int)a->elements.size() ? a->elements[i].mGeometry : nullptr;
}

MeMatrix4Ptr MEAPI McdAggregateGetElementTransformPtr(McdAggregateID g, int i)
{
    AggregateImpl* a = get(g);
    return i >= 0 && i < (int)a->elements.size() && a->elements[i].mGeometry ? a->elements[i].mRelTM : nullptr;
}

void MEAPI McdAggregateUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool)
{
    btTransform tm = toBtTransform(&finalTM[0][0]);
    syncChildren(ins, tm);
    if (!ins->child)
    {
        btVector3 o = tm.getOrigin();
        fromBt(o, ins->min);
        fromBt(o, ins->max);
        return;
    }
    for (int k = 0; k < 3; ++k)
    {
        ins->min[k] = MEINFINITY;
        ins->max[k] = -MEINFINITY;
    }
    for (McdGeometryInstance* c = ins->child; c; c = c->next)
        for (int k = 0; k < 3; ++k)
        {
            ins->min[k] = std::min(ins->min[k], c->min[k]);
            ins->max[k] = std::max(ins->max[k], c->max[k]);
        }
}

void MEAPI McdAggregateGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    btVector3 c;
    btScalar r = 0;
    AggregateImpl* a = get(g);
    a->shape.recalculateLocalAabb();
    a->shape.getBoundingSphere(c, r);
    fromBt(c, center);
    *radius = r;
}

void MEAPI McdAggregateMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    btVector3 dir = toBt(inDir);
    btScalar best = -MEINFINITY;
    // An empty aggregate is a point at its origin.
    std::copy(ins->mTM[3], ins->mTM[3] + 3, outPoint);
    for (McdGeometryInstance* c = ins->child; c; c = c->next)
    {
        MeVector3 p;
        McdGeometryInstanceMaximumPoint(c, inDir, p);
        if (dir.dot(toBt(p)) > best)
        {
            best = dir.dot(toBt(p));
            std::copy(p, p + 3, outPoint);
        }
    }
}

// Elements are combined at unit density: the inertia is about the combined centre of mass, per unit mass.
MeI16 MEAPI McdAggregateGetMassProperties(McdGeometryID g, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    struct Part
    {
        btMatrix3x3 inertia;
        btVector3 com;
        btScalar volume;
    };
    std::vector<Part> parts;
    btScalar total = 0;
    btVector3 com(0, 0, 0);
    for (McdAggregateElement& e : get(g)->elements)
    {
        if (!e.mGeometry)
            continue;
        MeMatrix4 childRel;
        MeMatrix3 childI;
        MeReal v = 0;
        if (!McdGeometryGetMassProperties(e.mGeometry, childRel, childI, &v) || v <= 0)
            continue;
        btTransform t = toBtTransform(e.mRelTM) * toBtTransform(childRel);
        btMatrix3x3 local(childI[0][0], childI[0][1], childI[0][2], childI[1][0], childI[1][1], childI[1][2],
                          childI[2][0], childI[2][1], childI[2][2]);
        parts.push_back({t.getBasis() * local * t.getBasis().transpose(), t.getOrigin(), v});
        total += v;
        com += t.getOrigin() * v;
    }
    mcdSetDiagonalInertia(relTM, inertia, 0, 0, 0);
    btMatrix3x3 sum(0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (total > 0)
    {
        com /= total;
        for (const Part& part : parts)
        {
            btVector3 d = part.com - com;
            btMatrix3x3 shift(d.dot(d) - d.x() * d.x(), -d.x() * d.y(), -d.x() * d.z(), -d.y() * d.x(),
                              d.dot(d) - d.y() * d.y(), -d.y() * d.z(), -d.z() * d.x(), -d.z() * d.y(),
                              d.dot(d) - d.z() * d.z());
            btScalar w = part.volume / total;
            for (int r = 0; r < 3; ++r)
                sum[r] += (part.inertia[r] + shift[r]) * w;
        }
        fromBt(com, relTM[3]);
    }
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            inertia[r][c] = sum[r][c];
    *volume = total;
    return total > 0;
}

MeBool MEAPI McdAggregateGenericRegisterInteractions(McdFrameworkID frame)
{
    for (int t = 0; t < frame->geometryRegisteredCountMax; ++t)
        mcdSetInteractionsBoth(frame, kMcdGeometryTypeAggregate, t, mcdIntersectInteraction(intersectAggregate));
    return 1;
}

void MEAPI McdAggregateRegisterInteractions(McdFrameworkID frame)
{
    McdAggregateGenericRegisterInteractions(frame);
    McdFrameworkSetLineSegInteraction(frame, kMcdGeometryTypeAggregate, intersectAggregateLineSegment);
}

char* MEAPI McdAggregateGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Aggregate");
}

void MEAPI McdAggregateDebugDraw(const McdGeometryID, const MeMatrix4, const MeReal[3])
{
}

} // extern "C"
