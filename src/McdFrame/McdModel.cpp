#include <cstring>
#include <iterator>

#include "McdImpl.h"

namespace {

const MeReal kIdentity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

} // namespace

void mcdModelSync(McdModelID m)
{
    if (m->mRelTM && m->mRefTM)
        McdModelCompoundTransforms(m);
    McdModelImpl* mi = impl(m);
    McdGeometryInstance& ins = m->mInstance;
    mi->object.setWorldTransform(toBtTransform(ins.mTM));
    McdGeometryInstanceUpdateAABB(&ins, ins.mTM, 1);
    for (int k = 0; k < 3; ++k)
    {
        ins.min[k] -= m->mPadding;
        ins.max[k] += m->mPadding;
    }
}

void mcdFreeChildInstances(McdGeometryInstance* ins)
{
    for (McdGeometryInstance* c = ins->child; c;)
    {
        McdGeometryInstance* next = c->next;
        mcdFreeChildInstances(c);
        delete reinterpret_cast<McdChildInstance*>(c);
        c = next;
    }
    ins->child = nullptr;
}

extern "C" {

McdModelID MEAPI McdModelCreate(McdGeometryID g)
{
    if (!g)
        return nullptr;
    auto* mi = new McdModelImpl();
    McdModel& m = mi->pub;
    McdFrameworkID f = g->frame;
    std::memcpy(mi->ownTM, kIdentity, sizeof(kIdentity));
    m.frame = f;
    m.mPadding = f->mDefaultPadding;
    m.mSpaceID = -1;
    m.mInstance.mGeometry = g;
    m.mInstance.mTM = mi->ownTM;
    mi->object.setCollisionShape(mcdGeometryShape(g));
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
    mcdFreeChildInstances(&cm->mInstance);
    std::set<McdModelKey>& disabled = impl(f)->disabledPairs;
    for (auto it = disabled.begin(); it != disabled.end();)
        it = it->first == cm || it->second == cm ? disabled.erase(it) : std::next(it);
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
    MeMatrix4Ptr tm = geometryTM ? geometryTM : impl(cm)->ownTM;
    // A model with a relative transform follows its reference frame, so the new matrix becomes that frame.
    if (cm->mRelTM)
        cm->mRefTM = tm;
    else
        cm->mInstance.mTM = tm;
}

void MEAPI McdModelReset(McdModelID cm)
{
    if (cm->mSpace)
    {
        MeWarning(0, "McdModelReset: remove the model from its space first");
        return;
    }
    McdModelImpl* mi = impl(cm);
    McdModel& m = mi->pub;
    std::memcpy(mi->ownTM, kIdentity, sizeof(kIdentity));
    m.linearVelocity = m.angularVelocity = nullptr;
    m.mPadding = m.frame->mDefaultPadding;
    m.sortKey = 0;
    m.mBody = nullptr;
    m.mRequestID = 0;
    m.mTransformUpdateFn = nullptr;
    m.mRelTM = m.mRefTM = nullptr;
    m.mIntersectFn = nullptr;
    m.flags = 0;
    m.mInstance.mTM = mi->ownTM;
    m.mInstance.mMaterial = 0;
    m.mData = nullptr;
    mi->cullTable = nullptr;
    mcdModelSync(cm);
}

void MEAPI McdModelSetGeometry(McdModelID cm, McdGeometryID g)
{
    // Pairs in a space cache narrowphase state built for the old shape.
    if (cm->mSpace || !g)
    {
        MeWarning(0, "McdModelSetGeometry: the model must not be in a space");
        return;
    }
    McdGeometryDecrementReferenceCount(cm->mInstance.mGeometry);
    mcdFreeChildInstances(&cm->mInstance);
    McdGeometryIncrementReferenceCount(g);
    cm->mInstance.mGeometry = g;
    impl(cm)->object.setCollisionShape(mcdGeometryShape(g));
    mcdModelSync(cm);
}

void MEAPI McdModelSetRelativeTransform(McdModelID cm, MeMatrix4 relTM)
{
    McdModelImpl* mi = impl(cm);
    std::memcpy(mi->ownRelTM, relTM, sizeof(MeMatrix4));
    MeMatrix4Ptr ref = cm->mRelTM ? cm->mRefTM : cm->mInstance.mTM;
    McdModelSetRelativeTransformPtrs(cm, mi->ownRelTM, ref, mi->compoundTM, 1);
}

MeMatrix4Ptr MEAPI McdModelGetRelativeTransform(McdModelID cm)
{
    return cm->mRelTM;
}

void MEAPI McdModelSetRelativeTransformPtrs(McdModelID cm, MeMatrix4 relTM, MeMatrix4 refTM, MeMatrix4 compoundTM,
                                            MeBool own)
{
    if (!relTM || !refTM)
    {
        MeMatrix4Ptr ref = cm->mRefTM;
        cm->mRelTM = cm->mRefTM = nullptr;
        cm->flags &= ~kMcdModelOwnRelativeTransforms;
        cm->mInstance.mTM = ref ? ref : impl(cm)->ownTM;
        return;
    }
    cm->mRelTM = relTM;
    cm->mRefTM = refTM;
    cm->mInstance.mTM = compoundTM ? compoundTM : impl(cm)->compoundTM;
    cm->flags = own ? cm->flags | kMcdModelOwnRelativeTransforms : cm->flags & ~kMcdModelOwnRelativeTransforms;
    McdModelCompoundTransforms(cm);
}

void MEAPI McdModelGetRelativeTransformPtrs(McdModelID cm, MeMatrix4Ptr* relTM, MeMatrix4Ptr* refTM)
{
    *relTM = cm->mRelTM;
    *refTM = cm->mRefTM;
}

// Row vectors: a point maps through relTM first, then through refTM.
void MEAPI McdModelCompoundTransforms(McdModelID cm)
{
    if (cm->mRelTM && cm->mRefTM)
        fromBtTransform(toBtTransform(cm->mRefTM) * toBtTransform(cm->mRelTM), cm->mInstance.mTM);
}

void MEAPI McdModelUpdatePath(McdModelID cm, MeReal motionDuration)
{
    McdModelUpdate(cm);
    if (!cm->linearVelocity)
        return;
    for (int k = 0; k < 3; ++k)
    {
        MeReal d = cm->linearVelocity[k] * motionDuration;
        (d < 0 ? cm->mInstance.min[k] : cm->mInstance.max[k]) += d;
    }
}

void MEAPI McdModelUpdatePathCompatible(McdModelID cm, MeReal)
{
    McdModelUpdate(cm);
}

void MEAPI McdModelGetBSphere(McdModelID cm, MeVector3 center, MeReal* radius)
{
    McdGeometryInstanceGetBSphere(&cm->mInstance, center, radius);
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
