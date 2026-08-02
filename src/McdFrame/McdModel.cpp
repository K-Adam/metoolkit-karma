#include <cstring>

#include "McdImpl.h"

namespace {

const MeReal kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

} // namespace

void mcdModelSync(McdModelID m)
{
    McdModelImpl* mi = impl(m);
    btTransform t = toBtTransform(m->mInstance.mTM);
    mi->object.setWorldTransform(t);
    btVector3 lo, hi;
    mi->object.getCollisionShape()->getAabb(t, lo, hi);
    btVector3 pad(m->mPadding, m->mPadding, m->mPadding);
    fromBt(lo - pad, m->mInstance.min);
    fromBt(hi + pad, m->mInstance.max);
}

extern "C" {

McdModelID MEAPI McdModelCreate(McdGeometryID g)
{
    btCollisionShape* shape = g ? mcdGeometryShape(g) : nullptr;
    if (!shape)
    {
        // User geometry plug-ins need a btCollisionShape adapter over their vtable before they can be models.
        MeWarning(0, "McdModelCreate: geometry type has no collision backend");
        return nullptr;
    }
    auto* mi = new McdModelImpl();
    McdModel& m = mi->pub;
    McdFrameworkID f = g->frame;
    std::memcpy(mi->ownTM, kIdentity, sizeof(kIdentity));
    m.frame = f;
    m.mPadding = f->mDefaultPadding;
    m.mSpaceID = -1;
    m.mInstance.mGeometry = g;
    m.mInstance.mTM = mi->ownTM;
    mi->object.setCollisionShape(shape);
    // Bullet defaults collision objects to static, which only matters to its dynamics, not to Mcd queries.
    mi->object.setCollisionFlags(0);
    mi->object.setUserPointer(&m);
    McdGeometryIncrementReferenceCount(g);

    m.next = f->firstModel;
    if (m.next)
        m.next->prev = &m;
    f->firstModel = &m;
    ++f->modelCount;
    mcdModelSync(&m);
    return &m;
}

void MEAPI McdModelDestroy(McdModelID cm)
{
    if (!cm)
        return;
    if (cm->mSpace)
    {
        MeWarning(0, "McdModelDestroy: remove the model from its space first");
        return;
    }
    McdFrameworkID f = cm->frame;
    if (cm->prev)
        cm->prev->next = cm->next;
    else
        f->firstModel = cm->next;
    if (cm->next)
        cm->next->prev = cm->prev;
    --f->modelCount;
    McdGeometryDecrementReferenceCount(cm->mInstance.mGeometry);
    delete impl(cm);
}

McdSpaceID MEAPI McdModelGetSpace(McdModelID cm)
{
    return cm->mSpace;
}

McdGeometryID MEAPI McdModelGetGeometry(McdModelID cm)
{
    return cm->mInstance.mGeometry;
}

McdGeometryInstanceID MEAPI McdModelGetGeometryInstance(McdModelID cm)
{
    return &cm->mInstance;
}

McdGeometryType MEAPI McdModelGetGeometryType(McdModelID cm)
{
    return (McdGeometryType)McdGeometryGetTypeId(cm->mInstance.mGeometry);
}

void MEAPI McdModelSetTransformPtr(const McdModelID cm, const MeMatrix4Ptr geometryTM)
{
    cm->mInstance.mTM = geometryTM ? geometryTM : impl(cm)->ownTM;
}

MeMatrix4Ptr MEAPI McdModelGetTransformPtr(McdModelID cm)
{
    return cm->mInstance.mTM;
}

void MEAPI McdModelUpdate(McdModelID cm)
{
    if (cm->mTransformUpdateFn)
        cm->mTransformUpdateFn(cm);
    mcdModelSync(cm);
}

void MEAPI McdModelSetUpdateCallback(McdModelID cm, McdModelUpdateFnPtr f)
{
    cm->mTransformUpdateFn = f;
}

McdModelUpdateFnPtr MEAPI McdModelGetUpdateCallback(McdModelID cm)
{
    return cm->mTransformUpdateFn;
}

void MEAPI McdModelGetAABB(McdModelID cm, MeVector3 minCorner, MeVector3 maxCorner)
{
    std::memcpy(minCorner, cm->mInstance.min, sizeof(MeVector3));
    std::memcpy(maxCorner, cm->mInstance.max, sizeof(MeVector3));
}

void MEAPI McdModelSetLinearVelocityPtr(McdModelID cm, MeReal* v)
{
    cm->linearVelocity = v;
}

MeReal* MEAPI McdModelGetLinearVelocityPtr(McdModelID cm)
{
    return cm->linearVelocity;
}

void MEAPI McdModelSetAngularVelocityPtr(McdModelID cm, MeReal* v)
{
    cm->angularVelocity = v;
}

MeReal* MEAPI McdModelGetAngularVelocityPtr(McdModelID cm)
{
    return cm->angularVelocity;
}

void MEAPI McdModelSetContactTolerance(McdModelID cm, MeReal tol)
{
    cm->mPadding = tol;
}

MeReal MEAPI McdModelGetContactTolerance(McdModelID cm)
{
    return cm->mPadding;
}

void MEAPI McdModelSetUserData(McdModelID cm, void* data)
{
    cm->mData = data;
}

void* MEAPI McdModelGetUserData(McdModelID cm)
{
    return cm->mData;
}

void* MEAPI McdModelGetBodyData(McdModelID cm)
{
    return cm->mBody;
}

void MEAPI McdModelSetBodyData(McdModelID cm, void* body)
{
    cm->mBody = body;
}

void MEAPI McdModelSetMaterial(McdModelID cm, int material)
{
    cm->mInstance.mMaterial = material;
}

unsigned int MEAPI McdModelGetMaterial(McdModelID cm)
{
    return (unsigned int)cm->mInstance.mMaterial;
}

void MEAPI McdModelSetRequestID(McdModelID cm, int requestId)
{
    cm->mRequestID = requestId;
}

int MEAPI McdModelGetRequestID(McdModelID cm)
{
    return cm->mRequestID;
}

MeI16 MEAPI McdModelGetSortKey(McdModelID cm)
{
    return cm->sortKey;
}

void MEAPI McdModelSetSortKey(McdModelID cm, MeI16 key)
{
    cm->sortKey = key;
}

void MEAPI McdModelSetIntersectCallback(McdModelID cm, McdModelIntersectFnPtr fn)
{
    cm->mIntersectFn = fn;
}

McdModelIntersectFnPtr MEAPI McdModelGetIntersectCallback(McdModelID cm)
{
    return cm->mIntersectFn;
}

} // extern "C"
