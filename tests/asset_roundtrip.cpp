#include <cstdio>
#include <cstring>

#include <MeAssetDBXMLIO.h>

#include "check.h"

static const char* kRagDolls = KARMA_REFERENCE_DIR "/KarmaData/RagDolls.ka";

static MeAssetDB* load(const char* path)
{
    MeStream stream = MeStreamOpen(path, kMeOpenModeRDONLY);
    CHECK(stream);
    MeAssetDB* db = MeAssetDBCreate();
    MeAssetDBXMLInput* input = MeAssetDBXMLInputCreate(db, NULL);
    CHECK(MeAssetDBXMLInputRead(input, stream));
    MeAssetDBXMLInputDestroy(input);
    MeStreamClose(stream);
    return db;
}

static bool sameString(const char* a, const char* b)
{
    return (!a && !b) || (a && b && std::strcmp(a, b) == 0);
}

static void checkReals(const MeReal* a, const MeReal* b, int n)
{
    for (int i = 0; i < n; ++i)
        CHECK_NEAR(a[i], b[i], 1e-6);
}

static void compare(MeFAsset* a, MeFAsset* b)
{
    CHECK(MeFAssetGetGeometryCount(a) == MeFAssetGetGeometryCount(b));
    CHECK(MeFAssetGetModelCount(a) == MeFAssetGetModelCount(b));
    CHECK(MeFAssetGetPartCount(a) == MeFAssetGetPartCount(b));
    CHECK(MeFAssetGetJointCount(a) == MeFAssetGetJointCount(b));
    CHECK(sameString(MeFAssetGetGraphicHint(a), MeFAssetGetGraphicHint(b)));
    CHECK_NEAR(MeFAssetGetGraphicScale(a), MeFAssetGetGraphicScale(b), 1e-6);

    MeFGeometryIt git;
    MeFAssetInitGeometryIterator(a, &git);
    while (MeFGeometry* g = MeFAssetGetGeometry(&git))
    {
        MeFGeometry* h = MeFAssetLookupGeometry(b, MeFGeometryGetName(g));
        CHECK(h && MeFGeometryGetPrimitiveCount(g) == MeFGeometryGetPrimitiveCount(h));
        CHECK(sameString(MeFGeometryGetGraphicHint(g), MeFGeometryGetGraphicHint(h)));
        CHECK_NEAR(MeFGeometryGetGraphicScale(g), MeFGeometryGetGraphicScale(h), 1e-6);
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
        CHECK_NEAR(MeFModelGetMass(m), MeFModelGetMass(n), 1e-9);
        CHECK_NEAR(MeFModelGetDensity(m), MeFModelGetDensity(n), 1e-9);
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
        CHECK_NEAR(MeFAssetPartGetGraphicScale(p), MeFAssetPartGetGraphicScale(q), 1e-6);
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
        CHECK(MeFJointGetType(j) == kMeFJointTypeHinge || MeFJointGetType(j) == kMeFJointTypeSkeletal);
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
    }
}

int main()
{
    if (std::FILE* f = std::fopen(kRagDolls, "r"))
        std::fclose(f);
    else
        return 77;

    MeAssetDB* original = load(kRagDolls);
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

    MeFGeometrySetGraphicHint(MeFAssetLookupGeometry(soldier, (char*)"neck"), "head.ase");
    MeFAssetPartSetGraphicScale(hip, 2);

    const char* saved = "asset_roundtrip.ka";
    MeStream out = MeStreamOpen(saved, kMeOpenModeWRONLY);
    CHECK(out);
    MeAssetDBXMLOutput* output = MeAssetDBXMLOutputCreate(original);
    MeAssetDBXMLOutputSetFileHeader(output, (char*)"written by %s", "asset_roundtrip");
    MeAssetDBXMLOutputWrite(output, out);
    MeAssetDBXMLOutputDestroy(output);
    MeStreamClose(out);

    MeAssetDB* reloaded = load(saved);
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
