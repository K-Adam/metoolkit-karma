#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <MeAssetFactory.h>
#include <MeMath.h>

#include "StringCopy.h"

namespace {

const MeMatrix4 kIdentity = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

void MEAPI freeKey(void* const key)
{
    std::free(key);
}

// Names are copied so an instance outlives the asset it was created from.
MeHash* createNameTable()
{
    MeHash* h = MeHashCreate(64);
    MeHashSetKeyFreeFunc(h, freeKey);
    return h;
}

void insertName(MeHash* h, const char* name, void* datum)
{
    if (name && datum)
        MeHashInsert(duplicateString(name), datum, h);
}

// The instance owns what its name tables list, so an object created under a name already taken would leak.
bool nameTaken(MeHash* h, const char* name)
{
    if (!name || !MeHashLookup(name, h))
        return false;
    MeWarning(0, "MeAssetInstanceCreate: duplicate name '%s' skipped", name);
    return true;
}

void worldTransform(MeMatrix4 out, const MeFAssetPart* part, const MeMatrix4Ptr assetTM)
{
    MeMatrix4TMCompound(out, part ? part->tm : kIdentity, assetTM ? assetTM : kIdentity);
}

McdGeometryID createPrimitive(McdFrameworkID fwk, MeFPrimitive* p)
{
    const MeReal* d = p->dims;
    switch (p->type)
    {
    case kMeFPrimitiveTypeSphere: return McdSphereCreate(fwk, d[0]);
    case kMeFPrimitiveTypeBox: return McdBoxCreate(fwk, d[0], d[1], d[2]);
    case kMeFPrimitiveTypeCylinder: return McdCylinderCreate(fwk, d[0], d[1]);
    case kMeFPrimitiveTypeSphyl: return McdSphylCreate(fwk, d[0], d[1]);
    case kMeFPrimitiveTypePlane: return McdPlaneCreate(fwk);
    case kMeFPrimitiveTypeConvex: return McdConvexMeshCreateHull(fwk, p->vertices, p->nVertices, 0);
    default: return nullptr;
    }
}

void destroyGeometry(McdGeometryID g)
{
    std::vector<McdGeometryID> elements;
    if (McdGeometryGetTypeId(g) == kMcdGeometryTypeAggregate)
        for (int i = 0; i < McdAggregateGetElementCountMax(g); ++i)
            if (McdGeometryID e = McdAggregateGetElementGeometry(g, i))
                elements.push_back(e);
    McdGeometryDestroy(g);
    for (McdGeometryID e : elements)
        McdGeometryDestroy(e);
}

void setFrame(MdtConstraintID c, unsigned i, const MeFJoint* j, int part, McdModelID m, const MeMatrix4Ptr assetTM,
              bool swapAxes)
{
    const MeReal *p = j->pos[part], *a = j->pax[part], *o = j->oax[part];
    if (swapAxes)
        std::swap(a, o);
    if (m && McdModelGetBody(m))
    {
        MdtConstraintBodySetPositionRel(c, i, p[0], p[1], p[2]);
        MdtConstraintBodySetAxesRel(c, i, a[0], a[1], a[2], o[0], o[1], o[2]);
        return;
    }
    // Without a body the joint attaches to the world, so the part frame is resolved now.
    MeMatrix4 tm;
    worldTransform(tm, MeFJointGetPart(j, part), assetTM);
    MeVector3 wp, wa, wo;
    MeMatrix4TMTransform(wp, tm, p);
    MeMatrix4TMRotate(wa, tm, a);
    MeMatrix4TMRotate(wo, tm, o);
    MdtConstraintBodySetPositionRel(c, i, wp[0], wp[1], wp[2]);
    MdtConstraintBodySetAxesRel(c, i, wa[0], wa[1], wa[2], wo[0], wo[1], wo[2]);
}

// Swapped bodies negate the joint coordinate, so the stops trade places and the motor reverses.
void setLimit(MdtLimitID l, const MeFJoint& j, bool swapped)
{
    int hi = swapped ? 1 : 0, lo = 1 - hi;
    MeReal sign = swapped ? -1 : 1;
    MdtSingleLimitSetStop(MdtLimitGetUpperLimit(l), sign * j.stop[hi]);
    MdtSingleLimitSetStiffness(MdtLimitGetUpperLimit(l), j.stiff[hi]);
    MdtSingleLimitSetStop(MdtLimitGetLowerLimit(l), sign * j.stop[lo]);
    MdtSingleLimitSetStiffness(MdtLimitGetLowerLimit(l), j.stiff[lo]);
    MdtLimitActivateLimits(l, j.bLimited[0]);
    MdtLimitSetLimitedForceMotor(l, sign * j.desVel[0], j.strength[0]);
    MdtLimitActivateMotor(l, j.bMotorized[0]);
}

struct JointKind
{
    MdtConstraintID (*create)(MdtWorldID);
    void (*setup)(MdtConstraintID, const MeFJoint&, bool swapped);
};

// Only the properties the .ka format can carry are applied, other types keep their Mdt defaults.
const JointKind kJointKinds[] = {
    /* Unknown */ {nullptr, nullptr},
    /* Carwheel */
    {[](MdtWorldID w) { return MdtCarWheelQuaConstraint(MdtCarWheelCreate(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool) {
         MdtCarWheelID cw = MdtConstraintDCastCarWheel(c);
         MdtCarWheelSetHingeLimitedForceMotor(cw, j.desVel[0], j.strength[0]);
         MdtCarWheelSetSteeringLimitedForceMotor(cw, j.desVel[1], j.strength[1]);
         MdtCarWheelSetSteeringLock(cw, j.special_b[0]);
         MdtCarWheelSetSuspension(cw, j.stiff[0], j.damp[0], j.stiff[2], j.stop[1], j.stop[0], j.special_f[0]);
     }},
    /* Hinge */
    {[](MdtWorldID w) { return MdtHingeQuaConstraint(MdtHingeCreate(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool swapped) {
         setLimit(MdtHingeGetLimit(MdtConstraintDCastHinge(c)), j, swapped);
     }},
    /* BallAndSocket */ {[](MdtWorldID w) { return MdtBSJointQuaConstraint(MdtBSJointCreate(w)); }, nullptr},
    /* ConeLimit */
    {[](MdtWorldID w) { return MdtConeLimitQuaConstraint(MdtConeLimitCreate(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool) {
         MdtConeLimitID cl = MdtConstraintDCastConeLimit(c);
         MdtConeLimitSetConeHalfAngle(cl, j.stop[0]);
         MdtConeLimitSetStiffness(cl, j.stiff[0]);
         MdtConeLimitSetDamping(cl, j.damp[0]);
     }},
    /* Universal */ {[](MdtWorldID w) { return MdtUniversalQuaConstraint(MdtUniversalCreate(w)); }, nullptr},
    /* Rpro */
    {[](MdtWorldID w) { return MdtRPROJointQuaConstraint(MdtRPROJointCreate(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool) {
         MdtRPROJointID r = MdtConstraintDCastRPROJoint(c);
         MdtRPROJointSetLinearStrength(r, j.strength[0], j.strength[1], j.strength[2]);
         MdtRPROJointSetAngularStrength(r, j.strength[3], j.strength[4], j.strength[5]);
     }},
    /* Prismatic */
    {[](MdtWorldID w) { return MdtPrismaticQuaConstraint(MdtPrismaticCreate(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool swapped) {
         setLimit(MdtPrismaticGetLimit(MdtConstraintDCastPrismatic(c)), j, swapped);
     }},
    /* Skeletal */
    {[](MdtWorldID w) { return MdtSkeletalQuaConstraint(MdtSkeletalCreate(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool) {
         MdtSkeletalID s = MdtConstraintDCastSkeletal(c);
         MdtSkeletalSetConeOption(s, (MdtSkeletalConeOption)j.special_i[1]);
         MdtSkeletalSetConePrimaryLimitAngle(s, j.stop[0]);
         MdtSkeletalSetConeSecondaryLimitAngle(s, j.stop[1]);
         MdtSkeletalSetConeStiffness(s, j.stiff[0]);
         MdtSkeletalSetConeDamping(s, j.damp[0]);
         MdtSkeletalSetTwistOption(s, (MdtSkeletalTwistOption)j.special_i[0]);
         MdtSkeletalSetTwistLimitAngle(s, j.stop[2]);
         MdtSkeletalSetTwistStiffness(s, j.stiff[1]);
         MdtSkeletalSetTwistDamping(s, j.damp[1]);
     }},
    /* Angular3 */
    {[](MdtWorldID w) { return MdtAngular3QuaConstraint(MdtAngular3Create(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool) {
         MdtAngular3ID a = MdtConstraintDCastAngular3(c);
         MdtAngular3EnableRotation(a, j.special_b[0]);
         MdtAngular3SetStiffness(a, j.stiff[0]);
         MdtAngular3SetDamping(a, j.damp[0]);
     }},
    /* Spring6 */
    {[](MdtWorldID w) { return MdtSpring6QuaConstraint(MdtSpring6Create(w)); },
     [](MdtConstraintID c, const MeFJoint& j, bool) {
         MdtSpring6ID s = MdtConstraintDCastSpring6(c);
         for (int k = 0; k < 3; ++k)
         {
             MdtSpring6SetLinearStiffness(s, k, j.stiff[k]);
             MdtSpring6SetAngularStiffness(s, k, j.stiff[3 + k]);
             MdtSpring6SetLinearDamping(s, k, j.damp[k]);
             MdtSpring6SetAngularDamping(s, k, j.damp[3 + k]);
         }
     }},
};

void setDynamics(MeAssetInstance* ins, bool enable)
{
    MeAIModelIt mit;
    MeAssetInstanceInitModelIterator(ins, &mit);
    while (McdModelID m = MeAssetInstanceGetNextModel(&mit))
        if (MdtBodyID b = McdModelGetBody(m))
            enable ? MdtBodyEnable(b) : MdtBodyDisable(b);
    MeAIJointIt jit;
    MeAssetInstanceInitJointIterator(ins, &jit);
    while (MdtConstraintID c = MeAssetInstanceGetNextJoint(&jit))
        enable ? MdtConstraintEnable(c) : MdtConstraintDisable(c);
}

} // namespace

extern "C" {

McdModelID MEAPI McdModelCreateFromMeFAssetPart(MeFAssetPart* part, McdGeometryID g, MdtWorldID world,
                                                MeMatrix4Ptr assetTM)
{
    MeFModel* fm = MeFAssetPartGetModel(part);
    McdModelID m = fm ? McdModelCreate(g) : nullptr;
    if (!m)
        return nullptr;
    MeMatrix4 tm;
    worldTransform(tm, part, assetTM);
    if (fm->type == kMeFModelTypeGeometryOnly || !world)
    {
        MeMatrix4Copy(McdModelGetTransformPtr(m), tm);
        return m;
    }

    MdtBodyID b = MdtBodyCreate(world);
    if (!b)
    {
        McdModelDestroy(m);
        return nullptr;
    }
    MdtBodySetTransform(b, tm);
    McdModelSetBody(m, b);
    MdtBodySetCenterOfMassRelativePosition(b, fm->mass_offset);
    MeMatrix3 inertia;
    MeFModelGetInertiaTensor(fm, inertia);
    if (fm->mass <= 0)
        MstAutoSetMassProperties(b, m, fm->density);
    else
    {
        MdtBodySetMass(b, fm->mass);
        if (inertia[0][0] > 0 && inertia[1][1] > 0 && inertia[2][2] > 0)
            MdtBodySetInertiaTensor(b, inertia);
        else
            MstAutoSetInertialTensor(m);
    }
    MdtBodySetLinearVelocityDamping(b, fm->linearDamp);
    MdtBodySetAngularVelocityDamping(b, fm->angularDamp);
    if (fm->useFastSpin)
        MdtBodySetFastSpinAxis(b, fm->fastSpin[0], fm->fastSpin[1], fm->fastSpin[2]);
    MdtBodyEnable(b);
    return m;
}

MdtConstraintID MEAPI MdtConstraintCreateFromMeFJoint(MeFJoint* joint, MdtWorldID world, McdModelID m1,
                                                      McdModelID m2, MeMatrix4Ptr assetTM)
{
    int type = MeFJointGetType(joint);
    if (type <= kMeFJointTypeUnknown || type >= (int)(sizeof(kJointKinds) / sizeof(kJointKinds[0])))
    {
        MeWarning(0, "MdtConstraintCreateFromMeFJoint: joint '%s' has unsupported type %d", MeFJointGetName(joint),
                  type);
        return nullptr;
    }
    // Mdt needs a first body, so a joint whose first part has none attaches the parts the other way round.
    int first = m1 && McdModelGetBody(m1) ? 0 : 1;
    if (first && type == kMeFJointTypeCarwheel)
    {
        MeWarning(0, "MdtConstraintCreateFromMeFJoint: car wheel '%s' needs a chassis body", MeFJointGetName(joint));
        return nullptr;
    }
    const JointKind& kind = kJointKinds[type];
    MdtConstraintID c = kind.create(world);
    if (!c)
        return nullptr;
    McdModelID m[2] = {first ? m2 : m1, first ? m1 : m2};
    MdtConstraintSetBodies(c, m[0] ? McdModelGetBody(m[0]) : nullptr, m[1] ? McdModelGetBody(m[1]) : nullptr);
    // A universal joint keeps body 1's primary axis normal to body 2's orthogonal one, so swapped bodies swap axes.
    bool swapAxes = first && type == kMeFJointTypeUniversal;
    setFrame(c, 0, joint, first, m[0], assetTM, swapAxes);
    setFrame(c, 1, joint, 1 - first, m[1], assetTM, swapAxes);
    if (first && type == kMeFJointTypeSkeletal &&
        (joint->special_i[1] == MdtSkeletalConeOptionSlot || joint->stop[0] != joint->stop[1]))
        MeWarning(0, "MdtConstraintCreateFromMeFJoint: skeletal '%s' is measured from its second part",
                  MeFJointGetName(joint));
    if (kind.setup)
        kind.setup(c, *joint, first != 0);
    MdtConstraintEnable(c);
    return c;
}

McdGeomMan* MEAPI McdGMCreate(McdFrameworkID fwk)
{
    // Assets may hold aggregates and convex hulls, which the caller need not have registered.
    if (!McdFrameworkTypeIsRegistered(fwk, kMcdGeometryTypeNull))
        McdNullRegisterType(fwk);
    if (!McdFrameworkTypeIsRegistered(fwk, kMcdGeometryTypeConvexMesh))
    {
        McdConvexMeshRegisterType(fwk);
        McdConvexMeshPrimitivesRegisterInteractions(fwk);
    }
    if (!McdFrameworkTypeIsRegistered(fwk, kMcdGeometryTypeAggregate))
    {
        McdAggregateRegisterType(fwk);
        McdAggregateRegisterInteractions(fwk);
    }
    auto* gm = new McdGeomMan{};
    gm->fwk = fwk;
    gm->name2geom = createNameTable();
    gm->geom2name = MeHashCreate(64);
    MeHashSetHashFunc(gm->geom2name, MeHashInt);
    MeHashSetKeyCompareFunc(gm->geom2name, MeHashIntCompare);
    gm->nullGeom = McdNullCreate(fwk);
    return gm;
}

// Geometry still listed belongs to instances the application owns, so it stays with the framework. The null
// geometry stays too, because the framework may already have been destroyed and freed it.
void MEAPI McdGMDestroy(McdGeomMan* gm)
{
    MeHashDestroy(gm->name2geom);
    MeHashDestroy(gm->geom2name);
    delete gm;
}

int MEAPI McdGMGetGeomCount(McdGeomMan* gm)
{
    return MeHashPopulation(gm->name2geom);
}

McdFrameworkID MEAPI McdGMGetFramework(McdGeomMan* gm)
{
    return gm->fwk;
}

// Instances of one asset share its geometry, and the manager holds one reference per instance. Keying by the
// MeFGeometry rather than by name keeps assets that share names apart.
McdGeometryID MEAPI McdGMCreateGeometry(McdGeomMan* gm, const MeFGeometry* fg, const char*)
{
    std::string name = std::to_string(reinterpret_cast<uintptr_t>(fg));
    auto* g = static_cast<McdGeometryID>(MeHashLookup(name.c_str(), gm->name2geom));
    if (g)
    {
        McdGeometryIncrementReferenceCount(g);
        return g;
    }

    MeFPrimitiveIt pit;
    MeFGeometryInitPrimitiveIterator(fg, &pit);
    MeFPrimitive* first = MeFGeometryGetPrimitive(&pit);
    if (!first)
        return nullptr;
    if (MeFGeometryGetPrimitiveCount(fg) == 1 && MeMatrix4IsIdentity(first->tm, (MeReal)1e-6))
        g = createPrimitive(gm->fwk, first);
    else
    {
        g = McdAggregateCreate(gm->fwk, MeFGeometryGetPrimitiveCount(fg));
        MeFGeometryInitPrimitiveIterator(fg, &pit);
        while (MeFPrimitive* p = MeFGeometryGetPrimitive(&pit))
            if (McdGeometryID e = createPrimitive(gm->fwk, p))
                McdAggregateAddElement(g, e, p->tm);
    }
    if (!g)
        return nullptr;
    McdGeometryIncrementReferenceCount(g);
    char* key = duplicateString(name.c_str());
    MeHashInsert(key, g, gm->name2geom);
    MeHashInsert(g, key, gm->geom2name);
    return g;
}

McdNullID MEAPI McdGMGetNullGeometry(McdGeomMan* gm)
{
    return gm->nullGeom;
}

void MEAPI McdGMDestroyGeometry(McdGeomMan* gm, McdGeometryID geom)
{
    auto* name = static_cast<char*>(MeHashLookup(geom, gm->geom2name));
    if (!name)
        return;
    McdGeometryDecrementReferenceCount(geom);
    if (McdGeometryGetReferenceCount(geom) > 0)
        return;
    MeHashDelete(geom, gm->geom2name);
    MeHashDelete(name, gm->name2geom);
    destroyGeometry(geom);
}

MeAssetFactory* MEAPI MeAssetFactoryCreate(McdFrameworkID fwk)
{
    auto* af = new MeAssetFactory{};
    af->gm = McdGMCreate(fwk);
    af->modelCreateFunc = McdModelCreateFromMeFAssetPart;
    af->jointCreateFunc = MdtConstraintCreateFromMeFJoint;
    return af;
}

void MEAPI MeAssetFactoryDestroy(MeAssetFactory* af)
{
    McdGMDestroy(af->gm);
    delete af;
}

void MEAPI MeAssetFactorySetGeometryPostCreateCB(MeAssetFactory* af, GeometryPostCreateCB cb, void* userdata)
{
    af->geometryPostCreateCB = cb;
    af->geometryPostCreateCBUserdata = userdata;
}

void MEAPI MeAssetFactorySetModelCreateFunction(MeAssetFactory* af, ModelCreateFunc func)
{
    af->modelCreateFunc = func ? func : McdModelCreateFromMeFAssetPart;
}

void MEAPI MeAssetFactorySetModelPostCreateCB(MeAssetFactory* af, ModelPostCreateCB cb, void* userdata)
{
    af->modelPostCreateCB = cb;
    af->modelPostCreateCBUserdata = userdata;
}

void MEAPI MeAssetFactorySetJointCreateFunction(MeAssetFactory* af, JointCreateFunc func)
{
    af->jointCreateFunc = func ? func : MdtConstraintCreateFromMeFJoint;
}

void MEAPI MeAssetFactorySetJointPostCreateCB(MeAssetFactory* af, JointPostCreateCB cb, void* userdata)
{
    af->jointPostCreateCB = cb;
    af->jointPostCreateCBUserdata = userdata;
}

MeAssetInstance* MEAPI MeAssetInstanceCreate(MeAssetFactory* af, const MeFAsset* asset, MeMatrix4Ptr tm,
                                             MeBool owner, MdtWorldID world, McdSpaceID space)
{
    auto* ins = new MeAssetInstance{};
    ins->af = af;
    ins->world = world;
    ins->space = space;
    ins->asset = const_cast<MeFAsset*>(asset);
    ins->owner = owner;
    ins->nameToGeometry = createNameTable();
    ins->nameToModel = createNameTable();
    ins->nameToJoint = createNameTable();

    MeFGeometryIt git;
    MeFAssetInitGeometryIterator(asset, &git);
    while (MeFGeometry* fg = MeFAssetGetGeometry(&git))
    {
        if (nameTaken(ins->nameToGeometry, MeFGeometryGetName(fg)))
            continue;
        int cached = McdGMGetGeomCount(af->gm);
        McdGeometryID g = McdGMCreateGeometry(af->gm, fg, MeFAssetGetName(asset));
        if (g && af->geometryPostCreateCB && McdGMGetGeomCount(af->gm) > cached)
            af->geometryPostCreateCB(g, fg, af->geometryPostCreateCBUserdata);
        insertName(ins->nameToGeometry, MeFGeometryGetName(fg), g);
    }

    std::vector<MeFAssetPart*> parts(MeFAssetGetPartCount(asset));
    MeFAssetGetPartsSortedByName(asset, parts.data());
    std::vector<McdModelID> colliding(parts.size());
    for (size_t i = 0; i < parts.size(); ++i)
    {
        MeFAssetPart* part = parts[i];
        MeFModel* fm = MeFAssetPartGetModel(part);
        if (!fm || nameTaken(ins->nameToModel, MeFAssetPartGetName(part)))
            continue;
        McdGeometryID g = fm->type == kMeFModelTypeDynamicsOnly
                              ? nullptr
                              : MeAssetInstanceGetGeometry(ins, MeFModelGetGeometryName(fm));
        McdModelID m = af->modelCreateFunc(part, g ? g : McdGMGetNullGeometry(af->gm), world, tm);
        if (!m)
            continue;
        if (space && g)
        {
            McdSpaceInsertModel(space, m);
            colliding[i] = m;
            for (size_t k = 0; k < i; ++k)
                if (colliding[k] && !MeFAssetPartIsCollisionEnabled(part, parts[k]))
                    McdSpaceDisablePair(m, colliding[k]);
        }
        insertName(ins->nameToModel, MeFAssetPartGetName(part), m);
        if (af->modelPostCreateCB)
            af->modelPostCreateCB(m, part, af->modelPostCreateCBUserdata);
    }

    MeFJointIt jit;
    MeFAssetInitJointIterator(asset, &jit);
    while (MeFJoint* fj = world ? MeFAssetGetJoint(&jit) : nullptr)
    {
        if (nameTaken(ins->nameToJoint, MeFJointGetName(fj)))
            continue;
        MdtConstraintID c = af->jointCreateFunc(fj, world, MeAssetInstanceGetModel(ins, MeFJointGetPartName(fj, 0)),
                                                MeAssetInstanceGetModel(ins, MeFJointGetPartName(fj, 1)), tm);
        if (!c)
            continue;
        insertName(ins->nameToJoint, MeFJointGetName(fj), c);
        if (af->jointPostCreateCB)
            af->jointPostCreateCB(c, fj, af->jointPostCreateCBUserdata);
    }
    return ins;
}

void MEAPI MeAssetInstanceDestroy(MeAssetInstance* ins)
{
    if (ins->owner)
    {
        MeAIJointIt jit;
        MeAssetInstanceInitJointIterator(ins, &jit);
        while (MdtConstraintID c = MeAssetInstanceGetNextJoint(&jit))
            MdtConstraintDestroy(c);
        MeAIModelIt mit;
        MeAssetInstanceInitModelIterator(ins, &mit);
        while (McdModelID m = MeAssetInstanceGetNextModel(&mit))
        {
            MdtBodyID b = McdModelGetBody(m);
            if (McdModelGetSpace(m))
                McdSpaceRemoveModel(m);
            // The body goes first: its disable callback still reads the model.
            MdtBodyDestroy(b);
            McdModelDestroy(m);
        }
        MeAIGeomIt git;
        MeAssetInstanceInitGeometryIterator(ins, &git);
        while (McdGeometryID g = MeAssetInstanceGetNextGeometry(&git))
            McdGMDestroyGeometry(ins->af->gm, g);
    }
    MeHashDestroy(ins->nameToGeometry);
    MeHashDestroy(ins->nameToModel);
    MeHashDestroy(ins->nameToJoint);
    delete ins;
}

McdGeometryID MEAPI MeAssetInstanceGetGeometry(MeAssetInstance* ins, char* name)
{
    return name ? static_cast<McdGeometryID>(MeHashLookup(name, ins->nameToGeometry)) : nullptr;
}

McdModelID MEAPI MeAssetInstanceGetModel(MeAssetInstance* ins, char* name)
{
    return name ? static_cast<McdModelID>(MeHashLookup(name, ins->nameToModel)) : nullptr;
}

MdtConstraintID MEAPI MeAssetInstanceGetJoint(MeAssetInstance* ins, char* name)
{
    return name ? static_cast<MdtConstraintID>(MeHashLookup(name, ins->nameToJoint)) : nullptr;
}

void MEAPI MeAssetInstanceInitGeometryIterator(MeAssetInstance* ins, MeAIGeomIt* it)
{
    MeHashInitIterator(&it->hashIt, ins->nameToGeometry);
}

McdGeometryID MEAPI MeAssetInstanceGetNextGeometry(MeAIGeomIt* it)
{
    return static_cast<McdGeometryID>(MeHashGetDatum(&it->hashIt));
}

void MEAPI MeAssetInstanceInitModelIterator(MeAssetInstance* ins, MeAIModelIt* it)
{
    MeHashInitIterator(&it->hashIt, ins->nameToModel);
}

McdModelID MEAPI MeAssetInstanceGetNextModel(MeAIModelIt* it)
{
    return static_cast<McdModelID>(MeHashGetDatum(&it->hashIt));
}

void MEAPI MeAssetInstanceInitJointIterator(MeAssetInstance* ins, MeAIJointIt* it)
{
    MeHashInitIterator(&it->hashIt, ins->nameToJoint);
}

MdtConstraintID MEAPI MeAssetInstanceGetNextJoint(MeAIJointIt* it)
{
    return static_cast<MdtConstraintID>(MeHashGetDatum(&it->hashIt));
}

void MEAPI MeAssetInstanceEnableDynamics(MeAssetInstance* ins)
{
    setDynamics(ins, true);
}

void MEAPI MeAssetInstanceDisableDynamics(MeAssetInstance* ins)
{
    setDynamics(ins, false);
}

void* MEAPI MeAssetInstanceGetUserData(MeAssetInstance* ins)
{
    return ins->userdata;
}

void MEAPI MeAssetInstanceSetUserData(MeAssetInstance* ins, void* userdata)
{
    ins->userdata = userdata;
}

} // extern "C"
