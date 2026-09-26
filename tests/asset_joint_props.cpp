#include <MeAssetFactory.h>
#include <Mst.h>

#include "check.h"

static MeFJoint* addJoint(MeFAsset* asset, const char* name, MeFJointType type, const char* p1 = "a",
                          const char* p2 = "b")
{
    MeFJoint* j = MeFJointCreate(name, type);
    MeFJointSetPartByName(j, p1, 0);
    MeFJointSetPartByName(j, p2, 1);
    MeFAssetInsertJoint(asset, j);
    return j;
}

static MeFAsset* sphereAsset(MeReal radius)
{
    MeFAsset* asset = MeFAssetCreate("ball", 0);
    MeFGeometry* g = MeFGeometryCreate("shape");
    MeFPrimitive* p = MeFPrimitiveCreate("sphere", kMeFPrimitiveTypeSphere);
    MeFPrimitiveSetRadius(p, radius);
    MeFGeometryInsertPrimitive(g, p);
    MeFAssetInsertGeometry(asset, g);
    MeFModel* m = MeFModelCreate("model", kMeFModelTypeGeometryOnly);
    MeFModelSetGeometry(m, g);
    MeFAssetInsertModel(asset, m);
    MeMatrix4 tm = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    MeFAssetInsertPart(asset, MeFAssetPartCreate("part", m, tm));
    return asset;
}

static int geometryCreated;

static void MEAPI countGeometry(McdGeometryID, MeFGeometry*, void*)
{
    ++geometryCreated;
}

int main()
{
    // Fresh joints default to Mdt's values, so instantiating one without properties keeps Mdt's behaviour.
    MeFJoint* fresh = MeFJointCreate("fresh", kMeFJointTypeCarwheel);
    CHECK(fresh->stop[0] == (MeReal)MDTCARWHEEL_DEFAULT_SUSP_HI && fresh->stop[1] == (MeReal)MDTCARWHEEL_DEFAULT_SUSP_LO);
    CHECK(fresh->strength[1] == (MeReal)MDTCARWHEEL_DEFAULT_STEER_MAXFORCE);
    MeFJointSetType(fresh, kMeFJointTypeRpro);
    MeFJointSetPerTypeDefaults(fresh);
    CHECK(fresh->strength[0] == MEINFINITY && fresh->strength[5] == MEINFINITY);
    MeFJointSetType(fresh, kMeFJointTypeHinge);
    CHECK(fresh->stiff[0] == MEINFINITY && fresh->stiff[1] == MEINFINITY);
    MeFJointSetType(fresh, kMeFJointTypeSkeletal);
    CHECK(fresh->stiff[0] == MEINFINITY && fresh->damp[1] == MEINFINITY);
    CHECK(fresh->special_i[0] == MdtSkeletalTwistOptionFixed && fresh->special_i[1] == MdtSkeletalConeOptionCone);
    MeFJointDestroy(fresh);

    MeFAsset* asset = MeFAssetCreate("pair", 0);
    MeFModel* model = MeFModelCreate("body", kMeFModelTypeDynamicsOnly);
    MeFModelSetMass(model, 1);
    MeFAssetInsertModel(asset, model);
    MeMatrix4 tm = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    MeFAssetInsertPart(asset, MeFAssetPartCreate("a", model, tm));
    tm[3][2] = -1;
    MeFAssetInsertPart(asset, MeFAssetPartCreate("b", model, tm));

    MeFJoint* wheel = addJoint(asset, "wheel", kMeFJointTypeCarwheel);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyDesiredVelocity1, 3);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStrength1, 40);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyDesiredVelocity2, -0.5f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStrength2, 70);
    MeFJointSetProperty1b(wheel, kMeFJointPropertySpecialBool1, 1);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStop1, 0.2f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStop2, -0.3f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertySpecialFloat1, 0.05f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStiffness1, 5000);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyDamping1, 60);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStiffness3, 0.01f);

    addJoint(asset, "cone", kMeFJointTypeConeLimit);
    addJoint(asset, "angular", kMeFJointTypeAngular3);

    // Part "w" has no model, so joints to it attach to the world and their first part is swapped.
    tm[3][2] = 5;
    MeFAssetInsertPart(asset, MeFAssetPartCreate("w", nullptr, tm));
    MeFJoint* hinge = addJoint(asset, "hinge", kMeFJointTypeHinge, "w", "a");
    MeFJointSetPosition(hinge, 0, 0, 0, -5);
    hinge->stop[0] = 0.2f;
    hinge->stop[1] = -0.5f;
    hinge->stiff[0] = 100;
    hinge->stiff[1] = 200;
    hinge->bLimited[0] = 1;
    hinge->desVel[0] = 2;
    addJoint(asset, "chassisless", kMeFJointTypeCarwheel, "w", "a");
    MeFJoint* universal = addJoint(asset, "universal", kMeFJointTypeUniversal, "w", "a");
    MeFJointSetPrimaryAxis(universal, 0, 1, 0, 0);
    MeFJointSetOrthogonalAxis(universal, 0, 0, 1, 0);
    MeFJointSetPrimaryAxis(universal, 1, 1, 0, 0);
    MeFJointSetOrthogonalAxis(universal, 1, 0, 1, 0);

    MeFJoint* spring = addJoint(asset, "spring", kMeFJointTypeSpring6);
    for (int k = 0; k < 6; ++k)
    {
        spring->stiff[k] = (MeReal)(100 + k);
        spring->damp[k] = (MeReal)(10 + k);
    }

    MstUniverseSizes sizes = MstUniverseDefaultSizes;
    MstUniverseID universe = MstUniverseCreate(&sizes);
    MeAssetFactory* factory = MeAssetFactoryCreate(MstUniverseGetFramework(universe));
    MeMatrix4 identity = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    MeAssetInstance* ins = MeAssetInstanceCreate(factory, asset, identity, 1, MstUniverseGetWorld(universe),
                                                 MstUniverseGetSpace(universe));
    CHECK(ins);

    MdtCarWheelID cw = MdtConstraintDCastCarWheel(MeAssetInstanceGetJoint(ins, (char*)"wheel"));
    CHECK(cw);
    CHECK_NEAR(MdtCarWheelGetHingeMotorDesiredVelocity(cw), 3, 1e-6);
    CHECK_NEAR(MdtCarWheelGetHingeMotorMaxForce(cw), 40, 1e-6);
    CHECK_NEAR(MdtCarWheelGetSteeringMotorDesiredVelocity(cw), -0.5, 1e-6);
    CHECK_NEAR(MdtCarWheelGetSteeringMotorMaxForce(cw), 70, 1e-6);
    CHECK(MdtCarWheelIsSteeringLocked(cw));
    CHECK_NEAR(MdtCarWheelGetSuspensionHighLimit(cw), 0.2, 1e-6);
    CHECK_NEAR(MdtCarWheelGetSuspensionLowLimit(cw), -0.3, 1e-6);
    CHECK_NEAR(MdtCarWheelGetSuspensionReference(cw), 0.05, 1e-6);
    CHECK_NEAR(MdtCarWheelGetSuspensionKp(cw), 5000, 1e-3);
    CHECK_NEAR(MdtCarWheelGetSuspensionKd(cw), 60, 1e-6);
    CHECK_NEAR(MdtCarWheelGetSuspensionLimitSoftness(cw), 0.01, 1e-6);

    MdtSpring6ID s6 = MdtConstraintDCastSpring6(MeAssetInstanceGetJoint(ins, (char*)"spring"));
    CHECK(s6);
    for (int k = 0; k < 3; ++k)
    {
        CHECK_NEAR(MdtSpring6GetLinearStiffness(s6, k), 100 + k, 1e-6);
        CHECK_NEAR(MdtSpring6GetAngularStiffness(s6, k), 103 + k, 1e-6);
        CHECK_NEAR(MdtSpring6GetLinearDamping(s6, k), 10 + k, 1e-6);
        CHECK_NEAR(MdtSpring6GetAngularDamping(s6, k), 13 + k, 1e-6);
    }

    MdtConeLimitID cone = MdtConstraintDCastConeLimit(MeAssetInstanceGetJoint(ins, (char*)"cone"));
    CHECK(cone && MdtConeLimitGetStiffness(cone) == MEINFINITY && MdtConeLimitGetDamping(cone) == MEINFINITY);
    CHECK_NEAR(MdtConeLimitGetConeHalfAngle(cone), ME_PI, 1e-6);
    MdtAngular3ID angular = MdtConstraintDCastAngular3(MeAssetInstanceGetJoint(ins, (char*)"angular"));
    CHECK(angular && MdtAngular3GetStiffness(angular) == MEINFINITY);

    MdtConstraintID hc = MeAssetInstanceGetJoint(ins, (char*)"hinge");
    CHECK(hc && MdtConstraintGetBody(hc, 0) == McdModelGetBody(MeAssetInstanceGetModel(ins, (char*)"a")));
    CHECK(!MdtConstraintGetBody(hc, 1));
    MeVector3 anchor;
    MdtConstraintBodyGetPosition(hc, 1, anchor);
    CHECK_NEAR(anchor[2], 0, 1e-5);
    MdtLimitID hl = MdtHingeGetLimit(MdtConstraintDCastHinge(hc));
    CHECK_NEAR(MdtSingleLimitGetStop(MdtLimitGetUpperLimit(hl)), 0.5, 1e-6);
    CHECK_NEAR(MdtSingleLimitGetStop(MdtLimitGetLowerLimit(hl)), -0.2, 1e-6);
    CHECK_NEAR(MdtSingleLimitGetStiffness(MdtLimitGetUpperLimit(hl)), 200, 1e-6);
    CHECK_NEAR(MdtSingleLimitGetStiffness(MdtLimitGetLowerLimit(hl)), 100, 1e-6);
    CHECK_NEAR(MdtLimitGetMotorDesiredVelocity(hl), -2, 1e-6);
    CHECK(!MeAssetInstanceGetJoint(ins, (char*)"chassisless"));
    MeVector3 primary, orthogonal;
    MdtConstraintBodyGetAxesRel(MeAssetInstanceGetJoint(ins, (char*)"universal"), 0, primary, orthogonal);
    CHECK_NEAR(primary[1], 1, 1e-6);
    CHECK_NEAR(orthogonal[0], 1, 1e-6);

    MeAssetInstanceDestroy(ins);

    // Assets that share names still get their own geometry, and the callback only sees new geometry.
    MeFAsset* small = sphereAsset(1);
    MeFAsset* large = sphereAsset(2);
    MeAssetFactorySetGeometryPostCreateCB(factory, countGeometry, nullptr);
    MeAssetInstance* s1 = MeAssetInstanceCreate(factory, small, identity, 1, nullptr, nullptr);
    MeAssetInstance* s2 = MeAssetInstanceCreate(factory, small, identity, 1, nullptr, nullptr);
    MeAssetInstance* l1 = MeAssetInstanceCreate(factory, large, identity, 1, nullptr, nullptr);
    CHECK(geometryCreated == 2);
    CHECK(MeAssetInstanceGetGeometry(s1, (char*)"shape") == MeAssetInstanceGetGeometry(s2, (char*)"shape"));
    CHECK_NEAR(McdSphereGetRadius(MeAssetInstanceGetGeometry(l1, (char*)"shape")), 2, 1e-6);
    MeAssetInstanceDestroy(s1);
    MeAssetInstanceDestroy(s2);
    MeAssetInstanceDestroy(l1);
    MeFAssetDestroy(small);
    MeFAssetDestroy(large);

    // The universe may go first, taking the factory's null geometry with it.
    MstUniverseDestroy(universe);
    MeAssetFactoryDestroy(factory);
    MeFAssetDestroy(asset);
    return 0;
}
