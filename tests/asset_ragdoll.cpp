#include <MeAssetFactory.h>
#include <Mst.h>

#include "asset_load.h"

static MdtBodyID bodyOf(MeAssetInstance* ins, char* part)
{
    McdModelID m = MeAssetInstanceGetModel(ins, part);
    CHECK(m && McdModelGetBody(m));
    return McdModelGetBody(m);
}

// The joint anchors as carried by each of its two bodies, which coincide while the joint holds.
static MeReal worstJointGap(MeAssetInstance* ins, MeFAsset* asset)
{
    MeReal worst = 0;
    MeFJointIt it;
    MeFAssetInitJointIterator(asset, &it);
    while (MeFJoint* j = MeFAssetGetJoint(&it))
    {
        MeVector3 a[2];
        for (int i = 0; i < 2; ++i)
            MeMatrix4TMTransform(a[i], MdtBodyGetTransformPtr(bodyOf(ins, MeFJointGetPartName(j, i))), j->pos[i]);
        MeReal gap = MeSqrt(MeVector3DistanceSqr(a[0], a[1]));
        worst = gap > worst ? gap : worst;
    }
    return worst;
}

int main()
{
    MeAssetDB* db = load(kRagDolls);
    if (!db)
    {
        printf("skipped: %s not found\n", kRagDolls);
        return 77;
    }
    MeFAsset* asset = MeAssetDBLookupAssetByName(db, "soldier2");
    CHECK(asset);

    MstUniverseSizes sizes = MstUniverseDefaultSizes;
    MstUniverseID universe = MstUniverseCreate(&sizes);
    McdFrameworkID frame = MstUniverseGetFramework(universe);
    MdtWorldID world = MstUniverseGetWorld(universe);
    MdtWorldSetGravity(world, 0, 0, -9.81f);
    // The dynamics only lower_back weighs a thousandth of its neighbours, and an iterative solver needs many
    // passes to carry the spine's load through it.
    MdtWorldSetMaxLCPIterations(world, 100);
    MdtContactParamsID contact = MstBridgeGetContactParams(MstUniverseGetBridge(universe), 0, 0);
    MdtContactParamsSetType(contact, MdtContactTypeFriction2D);
    MdtContactParamsSetFriction(contact, 2);

    MeMatrix4 groundTM = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    McdModelID ground = MstFixedModelCreate(universe, McdPlaneCreate(frame), groundTM);

    // The asset stands along its Y axis, so with the plane normal along Z it lies flat, turned about Z and
    // lifted clear of the plane.
    const MeReal lift = (MeReal)0.6;
    MeMatrix4 assetTM = {{0, 1, 0, 0}, {-1, 0, 0, 0}, {0, 0, 1, 0}, {0, 0, lift, 1}};
    MeAssetFactory* factory = MeAssetFactoryCreate(frame);
    MeAssetInstance* ins = MeAssetInstanceCreate(factory, asset, assetTM, 1, world, MstUniverseGetSpace(universe));
    CHECK(ins);

    int models = 0, joints = 0;
    MeAIModelIt mit;
    MeAssetInstanceInitModelIterator(ins, &mit);
    while (MeAssetInstanceGetNextModel(&mit))
        ++models;
    MeAIJointIt jit;
    MeAssetInstanceInitJointIterator(ins, &jit);
    while (MeAssetInstanceGetNextJoint(&jit))
        ++joints;
    CHECK(models == MeFAssetGetPartCount(asset));
    CHECK(joints == MeFAssetGetJointCount(asset));
    CHECK(McdGMGetGeomCount(factory->gm) == MeFAssetGetGeometryCount(asset));

    // The pelvis part sits at (0, 2.1, 0) in the asset.
    char pelvisName[] = "pelvis";
    MdtBodyID pelvis = bodyOf(ins, pelvisName);
    CHECK(MdtBodyIsEnabled(pelvis));
    MeVector3 start;
    MdtBodyGetPosition(pelvis, start);
    CHECK_NEAR(start[0], -2.1, 1e-4);
    CHECK_NEAR(start[2], lift, 1e-4);
    CHECK(worstJointGap(ins, asset) < 1e-4);

    MeReal worstGap = 0;
    for (int i = 0; i < 200; ++i)
    {
        MstUniverseStep(universe, (MeReal)0.01);
        MeReal gap = worstJointGap(ins, asset);
        worstGap = gap > worstGap ? gap : worstGap;
    }
    printf("worst joint gap %g\n", worstGap);
    CHECK(worstGap < 0.05);

    MeVector3 end;
    MdtBodyGetPosition(pelvis, end);
    CHECK(end[2] < start[2] - 0.1);
    MeAssetInstanceInitModelIterator(ins, &mit);
    while (McdModelID m = MeAssetInstanceGetNextModel(&mit))
    {
        MdtBodyID b = McdModelGetBody(m);
        MeVector3 p, v, w;
        MdtBodyGetPosition(b, p);
        MdtBodyGetLinearVelocity(b, v);
        MdtBodyGetAngularVelocity(b, w);
        for (int k = 0; k < 3; ++k)
            CHECK(p[k] == p[k] && v[k] == v[k] && w[k] == w[k]);
        CHECK(p[2] > 0 && p[2] < lift);
        CHECK(MeVector3Magnitude(v) < 0.1);
    }

    MeAssetInstanceDestroy(ins);
    CHECK(McdGMGetGeomCount(factory->gm) == 0);
    MeAssetFactoryDestroy(factory);
    MstFixedModelDestroy(ground);
    MstUniverseDestroy(universe);
    MeAssetDBDestroy(db);
    return 0;
}
