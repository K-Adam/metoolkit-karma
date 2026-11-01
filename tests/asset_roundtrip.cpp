#include <clocale>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "asset_load.h"

static bool sameString(const char* a, const char* b)
{
    return (!a && !b) || (a && b && std::strcmp(a, b) == 0);
}

// Saved numbers are the shortest round-trip text, so every value must come back bit for bit.
static void checkReals(const MeReal* a, const MeReal* b, int n)
{
    for (int i = 0; i < n; ++i)
        CHECK(a[i] == b[i]);
}

static void compare(MeFAsset* a, MeFAsset* b)
{
    CHECK(MeFAssetGetGeometryCount(a) == MeFAssetGetGeometryCount(b));
    CHECK(MeFAssetGetModelCount(a) == MeFAssetGetModelCount(b));
    CHECK(MeFAssetGetPartCount(a) == MeFAssetGetPartCount(b));
    CHECK(MeFAssetGetJointCount(a) == MeFAssetGetJointCount(b));
    CHECK(sameString(MeFAssetGetGraphicHint(a), MeFAssetGetGraphicHint(b)));
    CHECK(MeFAssetGetGraphicScale(a) == MeFAssetGetGraphicScale(b));

    MeFGeometryIt git;
    MeFAssetInitGeometryIterator(a, &git);
    while (MeFGeometry* g = MeFAssetGetGeometry(&git))
    {
        MeFGeometry* h = MeFAssetLookupGeometry(b, MeFGeometryGetName(g));
        CHECK(h && MeFGeometryGetPrimitiveCount(g) == MeFGeometryGetPrimitiveCount(h));
        CHECK(sameString(MeFGeometryGetGraphicHint(g), MeFGeometryGetGraphicHint(h)));
        CHECK(MeFGeometryGetGraphicScale(g) == MeFGeometryGetGraphicScale(h));
        MeFPrimitiveIt pit;
        MeFGeometryInitPrimitiveIterator(g, &pit);
        while (MeFPrimitive* p = MeFGeometryGetPrimitive(&pit))
        {
            MeFPrimitive* q = MeFGeometryLookupPrimitive(h, MeFPrimitiveGetName(p));
            CHECK(q && MeFPrimitiveGetType(p) == MeFPrimitiveGetType(q));
            CHECK(MeFPrimitiveGetType(p) == kMeFPrimitiveTypeSphere || MeFPrimitiveGetType(p) == kMeFPrimitiveTypeSphyl);
            checkReals(p->dims, q->dims, 3);
            checkReals(&p->tm[0][0], &q->tm[0][0], 16);
        }
    }

    MeFModelIt mit;
    MeFAssetInitModelIterator(a, &mit);
    while (MeFModel* m = MeFAssetGetModel(&mit))
    {
        MeFModel* n = MeFAssetLookupModel(b, MeFModelGetName(m));
        CHECK(n && MeFModelGetType(m) == MeFModelGetType(n));
        CHECK(sameString(MeFModelGetGeometryName(m), MeFModelGetGeometryName(n)));
        CHECK(!MeFModelGetGeometryName(m) || MeFModelGetGeometry(m));
        CHECK(MeFModelGetMass(m) == MeFModelGetMass(n));
        CHECK(MeFModelGetDensity(m) == MeFModelGetDensity(n));
        checkReals(m->mass_offset, n->mass_offset, 3);
        checkReals(m->inertia, n->inertia, 6);
        checkReals(m->fastSpin, n->fastSpin, 3);
        CHECK(MeFModelIsFastSpinAxisEnabled(m) == MeFModelIsFastSpinAxisEnabled(n));
    }

    MeFAssetPartIt pit;
    MeFAssetInitPartIterator(a, &pit);
    while (MeFAssetPart* p = MeFAssetGetPart(&pit))
    {
        MeFAssetPart* q = MeFAssetLookupPart(b, MeFAssetPartGetName(p));
        CHECK(q && sameString(MeFAssetPartGetModelName(p), MeFAssetPartGetModelName(q)));
        CHECK(sameString(MeFAssetPartGetGraphicHint(p), MeFAssetPartGetGraphicHint(q)));
        CHECK(MeFAssetPartGetGraphicScale(p) == MeFAssetPartGetGraphicScale(q));
        CHECK(sameString(MeFAssetPartGetParentPartName(p), MeFAssetPartGetParentPartName(q)));
        CHECK(MeFAssetPartGetModel(p) != NULL);
        checkReals(&p->tm[0][0], &q->tm[0][0], 16);
        MeFAssetPartIt other;
        MeFAssetInitPartIterator(a, &other);
        while (MeFAssetPart* r = MeFAssetGetPart(&other))
            CHECK(MeFAssetPartIsCollisionEnabled(p, r) ==
                  MeFAssetPartIsCollisionEnabled(q, MeFAssetLookupPart(b, MeFAssetPartGetName(r))));
    }

    MeFJointIt jit;
    MeFAssetInitJointIterator(a, &jit);
    while (MeFJoint* j = MeFAssetGetJoint(&jit))
    {
        MeFJoint* k = MeFAssetLookupJoint(b, MeFJointGetName(j));
        CHECK(k && MeFJointGetType(j) == MeFJointGetType(k));
        CHECK(MeFJointGetPart(j, 0) && MeFJointGetPart(j, 1));
        CHECK(sameString(MeFJointGetPartName(j, 0), MeFJointGetPartName(k, 0)));
        CHECK(sameString(MeFJointGetPartName(j, 1), MeFJointGetPartName(k, 1)));
        checkReals(&j->pos[0][0], &k->pos[0][0], 6);
        checkReals(&j->pax[0][0], &k->pax[0][0], 6);
        checkReals(&j->oax[0][0], &k->oax[0][0], 6);
        checkReals(j->stop, k->stop, 3);
        checkReals(j->stiff, k->stiff, 6);
        checkReals(j->damp, k->damp, 6);
        checkReals(j->strength, k->strength, 6);
        checkReals(j->desVel, k->desVel, 2);
        CHECK(j->bLimited[0] == k->bLimited[0] && j->bMotorized[0] == k->bMotorized[0]);
        CHECK(j->special_i[0] == k->special_i[0] && j->special_i[1] == k->special_i[1]);
        checkReals(j->special_f, k->special_f, 1);
        CHECK(j->special_b[0] == k->special_b[0]);
    }
}

// The name table must stay valid when items share a name or are renamed onto each other.
static void checkDuplicateNames()
{
    MeFAsset* a = MeFAssetCreate("dup", 0);
    char x[] = "x", z[] = "z";
    MeFGeometry* g1 = MeFGeometryCreate("x");
    MeFGeometry* g2 = MeFGeometryCreate("x");
    MeFGeometry* g3 = MeFGeometryCreate("y");
    MeFAssetInsertGeometry(a, g1);
    MeFAssetInsertGeometry(a, g2);
    MeFAssetInsertGeometry(a, g3);
    CHECK(MeFAssetLookupGeometry(a, x) == g2);
    MeFGeometryDestroy(g2);
    CHECK(MeFAssetLookupGeometry(a, x) == g1);
    MeFGeometryRename(g3, "x");
    CHECK(MeFAssetLookupGeometry(a, x) == g3);
    MeFGeometryDestroy(g3);
    CHECK(MeFAssetLookupGeometry(a, x) == g1);
    MeFGeometryRename(g1, "z");
    CHECK(!MeFAssetLookupGeometry(a, x) && MeFAssetLookupGeometry(a, z) == g1);
    MeFAssetDestroy(a);
}

// The collision matrix stops growing at its cap, and parts past it always collide.
static void checkManyParts()
{
    MeFAsset* a = MeFAssetCreate("many", 0);
    std::vector<MeFAssetPart*> parts;
    for (int i = 0; i < 5000; ++i)
    {
        parts.push_back(MeFAssetPartCreate(("p" + std::to_string(i)).c_str(), NULL, NULL));
        MeFAssetInsertPart(a, parts.back());
    }
    MeFAssetPartEnableCollision(parts[0], parts[1], 0);
    MeFAssetPartEnableCollision(parts[0], parts[4999], 0);
    CHECK(!MeFAssetPartIsCollisionEnabled(parts[0], parts[1]));
    CHECK(MeFAssetPartIsCollisionEnabled(parts[0], parts[4999]));
    MeFAssetDestroy(a);
}

// Files use '.' for decimals even when the host program sets a locale that uses ','.
static void checkLocale()
{
    if (!std::setlocale(LC_NUMERIC, "de_DE.UTF-8") && !std::setlocale(LC_NUMERIC, "German"))
        return;
    MeAssetDB* db = MeAssetDBCreate();
    MeFAsset* a = MeFAssetCreate("loc", 0);
    MeFAssetSetGraphicScale(a, 1.5f);
    MeFGeometry* g = MeFGeometryCreate("g");
    MeFPrimitive* p = MeFPrimitiveCreate("ball", kMeFPrimitiveTypeSphere);
    MeFPrimitiveSetRadius(p, 0.25f);
    MeFGeometryInsertPrimitive(g, p);
    MeFAssetInsertGeometry(a, g);
    MeAssetDBInsertAsset(db, a);

    MeStream s = MeStreamOpenAsMemBuffer(256);
    MeAssetDBXMLOutput* output = MeAssetDBXMLOutputCreate(db);
    MeAssetDBXMLOutputWrite(output, s);
    MeAssetDBXMLOutputDestroy(output);
    MeStreamRewind(s);
    MeAssetDB* back = MeAssetDBCreate();
    MeAssetDBXMLInput* input = MeAssetDBXMLInputCreate(back, NULL);
    CHECK(MeAssetDBXMLInputRead(input, s));
    MeAssetDBXMLInputDestroy(input);
    MeStreamClose(s);

    MeFAsset* b = MeAssetDBLookupAssetByName(back, "loc");
    CHECK(b && MeFAssetGetGraphicScale(b) == 1.5f);
    MeFPrimitive* q = MeFGeometryLookupPrimitive(MeFAssetLookupGeometry(b, (char*)"g"), (char*)"ball");
    CHECK(q && MeFPrimitiveGetRadius(q) == 0.25f);
    MeAssetDBDestroy(back);
    MeAssetDBDestroy(db);
    std::setlocale(LC_NUMERIC, "C");
}

static void checkCombine(MeFAsset* soldier)
{
    MeFAsset* both = MeFAssetCreateCopy(soldier, 1);
    MeFAssetCombine(both, soldier);
    CHECK(MeFAssetGetGeometryCount(both) == 2 * MeFAssetGetGeometryCount(soldier));
    CHECK(MeFAssetGetModelCount(both) == 2 * MeFAssetGetModelCount(soldier));
    CHECK(MeFAssetGetPartCount(both) == 2 * MeFAssetGetPartCount(soldier));
    CHECK(MeFAssetGetJointCount(both) == 2 * MeFAssetGetJointCount(soldier));
    MeFAssetPartIt pit;
    MeFAssetInitPartIterator(both, &pit);
    while (MeFAssetPart* p = MeFAssetGetPart(&pit))
        CHECK(MeFAssetLookupPart(both, MeFAssetPartGetName(p)) == p && MeFAssetPartGetGeometry(p));
    MeFJointIt jit;
    MeFAssetInitJointIterator(both, &jit);
    while (MeFJoint* j = MeFAssetGetJoint(&jit))
        CHECK(MeFAssetLookupJoint(both, MeFJointGetName(j)) == j && MeFJointGetPart(j, 0) && MeFJointGetPart(j, 1));
    MeFAssetDestroy(both);
}

int main()
{
    checkDuplicateNames();
    checkManyParts();
    checkLocale();

    MeAssetDB* original = load(kRagDolls);
    if (!original)
        return 77;
    CHECK(MeAssetDBGetAssetCount(original) == 2);
    MeFAsset* soldier = MeAssetDBLookupAssetByName(original, "soldier2");
    CHECK(soldier);
    CHECK(MeFAssetGetGeometryCount(soldier) == 10 && MeFAssetGetModelCount(soldier) == 12);
    CHECK(MeFAssetGetPartCount(soldier) == 12 && MeFAssetGetJointCount(soldier) == 11);

    MeFJoint* elbow = MeFAssetLookupJoint(soldier, (char*)"l_elbow");
    MeReal high;
    CHECK(MeFJointGetProperty1f(elbow, kMeFJointPropertyStop1, &high));
    CHECK_NEAR(high, 1.272712, 1e-6);
    MeFAssetPart* hip = MeFAssetLookupPart(soldier, (char*)"l_hip");
    CHECK(!MeFAssetPartIsCollisionEnabled(hip, MeFAssetLookupPart(soldier, (char*)"pelvis")));
    CHECK(MeFAssetPartIsCollisionEnabled(hip, MeFAssetLookupPart(soldier, (char*)"neck")));

    checkCombine(soldier);

    MeFGeometrySetGraphicHint(MeFAssetLookupGeometry(soldier, (char*)"neck"), "head.ase");
    MeFAssetPartSetGraphicScale(hip, 2);

    MeFJoint* wheel = MeFJointCreate("wheel", kMeFJointTypeCarwheel);
    MeFJointSetPartByName(wheel, "pelvis", 0);
    MeFJointSetPartByName(wheel, "l_hip", 1);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStop1, 0.1f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStop2, -1.0f / 3);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStiffness1, 50000.7f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStiffness3, 0.001f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyDamping1, 0.4f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStrength1, 2.0f / 7);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyStrength2, 9.81f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyDesiredVelocity1, 3.14159265f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertyDesiredVelocity2, -0.7f);
    MeFJointSetProperty1f(wheel, kMeFJointPropertySpecialFloat1, 0.05f);
    MeFJointSetProperty1b(wheel, kMeFJointPropertySpecialBool1, 1);
    MeFAssetInsertJoint(soldier, wheel);

    // A fresh cone limit is hard, so its MEINFINITY stiffness and damping must survive the file.
    MeFJoint* cone = MeFJointCreate("cone", kMeFJointTypeConeLimit);
    MeFJointSetPartByName(cone, "pelvis", 0);
    MeFJointSetPartByName(cone, "l_hip", 1);
    MeFAssetInsertJoint(soldier, cone);

    const char* saved = "asset_roundtrip.ka";
    MeStream out = MeStreamOpen(saved, kMeOpenModeWRONLY);
    CHECK(out);
    MeAssetDBXMLOutput* output = MeAssetDBXMLOutputCreate(original);
    MeAssetDBXMLOutputSetFileHeader(output, (char*)"written by %s", "asset_roundtrip");
    MeAssetDBXMLOutputWrite(output, out);
    MeAssetDBXMLOutputDestroy(output);
    MeStreamClose(out);

    MeAssetDB* reloaded = load(saved);
    CHECK(reloaded);
    CHECK(MeAssetDBGetAssetCount(reloaded) == MeAssetDBGetAssetCount(original));
    MeFAssetIt it;
    MeAssetDBInitAssetIterator(original, &it);
    while (MeFAsset* a = MeAssetDBGetAsset(&it))
    {
        MeFAsset* b = MeAssetDBLookupAssetByName(reloaded, MeFAssetGetName(a));
        CHECK(b);
        compare(a, b);
    }

    MeAssetDB* copy = MeAssetDBCreateCopy(original);
    MeAssetDBInitAssetIterator(original, &it);
    while (MeFAsset* a = MeAssetDBGetAsset(&it))
        compare(a, MeAssetDBLookupAssetByName(copy, MeFAssetGetName(a)));

    MeAssetDBDestroy(copy);
    MeAssetDBDestroy(reloaded);
    MeAssetDBDestroy(original);
    std::remove(saved);
    return 0;
}
