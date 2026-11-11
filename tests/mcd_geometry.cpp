#include <Mcd.h>
#include <McdContact.h>
#include <McdModelPairManager.h>
#include <McdPrimitives.h>

#include "check.h"

namespace {

void place(MeMatrix4 tm, MeReal x, MeReal y, MeReal z)
{
    MeMatrix4TMMakeIdentity(tm);
    MeMatrix4TMSetPosition(tm, x, y, z);
}

int intersect(McdModelID a, McdModelID b, McdContact* contacts, int max = 16)
{
    McdModelPairID pair = McdModelPairCreate(a, b);
    McdIntersectResult r{};
    r.contacts = contacts;
    r.contactMaxCount = max;
    McdHello(pair);
    McdIntersect(pair, &r);
    McdGoodbye(pair);
    McdModelPairDestroy(pair);
    return r.contactCount;
}

// A 20 x 20 ground square at z = 0, as two upward-facing triangles.
MeVector3 groundVertices[4] = {{-10, -10, 0}, {10, -10, 0}, {10, 10, 0}, {-10, 10, 0}};
MeVector3 up = {0, 0, 1};
int generatorCalls = 0;

int MEAPI ground(McdModelPair*, McdUserTriangle* t, MeVector3, MeReal, int max)
{
    ++generatorCalls;
    CHECK(max >= 2);
    int corners[2][3] = {{0, 1, 2}, {0, 2, 3}};
    for (int i = 0; i < 2; ++i)
    {
        for (int k = 0; k < 3; ++k)
            t[i].vertices[k] = &groundVertices[corners[i][k]];
        t[i].normal = &up;
        t[i].flags = kMcdTriangleStandard;
        t[i].triangleData.tag = i;
    }
    return 2;
}

void pairCounts(McdSpaceID space, McdModelPairContainer* c, bool transitions, int& hello, int& staying)
{
    McdSpaceUpdateAll(space);
    McdSpacePairIterator it;
    McdSpacePairIteratorBegin(space, &it);
    if (transitions)
        McdSpaceGetTransitions(space, &it, c);
    else
        McdSpaceGetPairs(space, &it, c);
    McdGoodbyeEach(c);
    McdHelloEach(c);
    hello = McdModelPairContainerGetHelloCount(c);
    staying = McdModelPairContainerGetStayingCount(c);
}

} // namespace

int main()
{
    McdFrameworkID frame = McdInit(McdPrimitivesGetTypeCount(), 100, 100, 1);
    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    McdAggregateRegisterType(frame);
    McdAggregateRegisterInteractions(frame);
    McdNullRegisterType(frame);
    McdConvexMeshRegisterType(frame);
    McdConvexMeshPrimitivesRegisterInteractions(frame);

    MeMatrix4 groundTM, tmA, tmB;
    place(groundTM, 0, 0, 0);
    McdModelID plane = McdModelCreate(McdPlaneCreate(frame));
    McdModelSetTransformPtr(plane, groundTM);
    McdModelUpdate(plane);
    McdContact contacts[16];

    // Cylinder and sphyl parameters and Karma's mass properties.
    McdGeometryID cylinder = McdCylinderCreate(frame, 0.5f, 2);
    CHECK_NEAR(McdCylinderGetHalfHeight(cylinder), 1, 1e-6);
    McdCylinderSetRadius(cylinder, 0.25f);
    CHECK_NEAR(McdCylinderGetRadius(cylinder), 0.25, 1e-6);
    CHECK_NEAR(McdCylinderGetHeight(cylinder), 2, 1e-6);
    MeMatrix4 rel;
    MeMatrix3 inertia;
    MeReal volume;
    McdGeometryID sphyl = McdSphylCreate(frame, 1, 0);
    McdGeometryGetMassProperties(sphyl, rel, inertia, &volume);
    CHECK_NEAR(inertia[0][0], 0.4, 1e-5);
    CHECK_NEAR(inertia[2][2], 0.5, 1e-5);
    MeVector3 center;
    MeReal radius;
    McdSphylSetHeight(sphyl, 2);
    McdGeometryGetBSphere(sphyl, center, &radius);
    CHECK_NEAR(radius, 2, 1e-5);

    // A box resting on a triangle list gets its four bottom corners, with normals towards the box.
    MeVector3 lo = {-10, -10, -1}, hi = {10, 10, 0};
    McdModelID terrain = McdModelCreate(McdTriangleListCreate(frame, lo, hi, 8, ground));
    McdModelSetTransformPtr(terrain, groundTM);
    McdModelUpdate(terrain);
    McdModelID box = McdModelCreate(McdBoxCreate(frame, 1, 1, 1));
    place(tmA, 0.3f, 0.2f, 0.49f);
    McdModelSetTransformPtr(box, tmA);
    McdModelUpdate(box);
    int n = intersect(box, terrain, contacts);
    CHECK(generatorCalls == 1);
    CHECK(n >= 4);
    for (int i = 0; i < n; ++i)
        CHECK_NEAR(contacts[i].normal[2], 1, 1e-3);
    // Hello swaps the pair to Karma's order, box first, so the normals still point towards the box.
    n = intersect(terrain, box, contacts);
    CHECK(n >= 4 && contacts[0].normal[2] > 0.999f);

    // A cube hull: six quads, unit volume, four contacts when sunk into a plane.
    MeVector3 cube[8];
    for (int i = 0; i < 8; ++i)
        for (int k = 0; k < 3; ++k)
            cube[i][k] = (i >> k & 1) ? 0.5f : -0.5f;
    McdGeometryID hull = McdConvexMeshCreateHull(frame, cube, 8, 0);
    CHECK(McdConvexMeshGetPolygonCount(hull) == 6);
    CHECK(McdConvexMeshGetPolygonVertexCount(hull, 0) == 4);
    McdGeometryGetMassProperties(hull, rel, inertia, &volume);
    CHECK_NEAR(volume, 1, 1e-4);
    CHECK_NEAR(inertia[0][0], 1.0 / 6, 1e-4);
    McdModelID hullModel = McdModelCreate(hull);
    McdModelSetTransformPtr(hullModel, tmA);
    McdModelUpdate(hullModel);
    CHECK(intersect(hullModel, plane, contacts) == 4);

    // An aggregate of two boxes collides element by element: four corners each.
    McdGeometryID twin = McdAggregateCreate(frame, 2);
    static MeMatrix4 left, right;
    place(left, -1, 0, 0);
    place(right, 1, 0, 0);
    McdGeometryID unit = McdBoxCreate(frame, 1, 1, 1);
    CHECK(McdAggregateAddElement(twin, unit, left) == 0);
    CHECK(McdAggregateAddElement(twin, unit, right) == 1);
    McdGeometryGetMassProperties(twin, rel, inertia, &volume);
    CHECK_NEAR(volume, 2, 1e-5);
    CHECK_NEAR(inertia[1][1], 1.0 / 6 + 1, 1e-4);
    McdModelID twinModel = McdModelCreate(twin);
    McdModelSetTransformPtr(twinModel, tmA);
    McdModelUpdate(twinModel);
    CHECK(McdGeometryInstanceGetChild(McdModelGetGeometryInstance(twinModel), 1) != nullptr);
    CHECK(intersect(twinModel, plane, contacts) == 8);

    // McdContactSimplify keeps the deepest contact and spreads the rest.
    contacts[3].separation = -1;
    CHECK(McdContactSimplify(up, contacts, 8, contacts, 3, 0, 1) == 3);
    CHECK(contacts[0].separation == -1);

    // A null model lives in a space but touches nothing.
    McdModelID null = McdModelCreate(McdNullCreate(frame));
    CHECK(null && intersect(null, box, contacts) == 0);

    // Line segments: the first hit is the sphere, and a segment starting inside it hits at distance zero.
    McdSpaceID space = McdSpaceAxisSortCreate(frame, McdAllAxes, 100, 100);
    McdModelPairContainer* pairs = McdModelPairContainerCreate(100);
    McdModelID ball = McdModelCreate(McdSphereCreate(frame, 1));
    place(tmB, 0, 0, 3);
    McdModelSetTransformPtr(ball, tmB);
    McdSpaceInsertModel(space, plane);
    McdSpaceInsertModel(space, ball);
    McdSpaceUpdateAll(space);
    MeVector3 from = {0, 0, 10}, to = {0, 0, -1}, inside = {0, 0, 3};
    McdLineSegIntersectResult hit, hits[4];
    CHECK(McdSpaceGetLineSegFirstIntersection(space, from, to, &hit));
    CHECK(hit.model == ball && fabs(hit.position[2] - 4) < 1e-3 && fabs(hit.distance - 6) < 1e-3);
    CHECK_NEAR(hit.normal[2], 1, 1e-3);
    CHECK(McdSpaceGetLineSegIntersections(space, from, to, hits, 4) == 2);
    CHECK(hits[1].model == plane && fabs(hits[1].distance - 10) < 1e-3);
    CHECK(McdLineSegIntersect(ball, inside, to, &hit) && hit.distance == 0);

    // Pair enable and disable, culling tables and transitions. The infinite plane would pair with everything.
    McdSpaceRemoveModel(plane);
    McdModelID other = McdModelCreate(McdSphereCreate(frame, 1));
    place(tmA, 0, 0, 4.5f);
    McdModelSetTransformPtr(other, tmA);
    McdSpaceInsertModel(space, other);
    int hello, staying;
    McdSpaceDisablePair(ball, other);
    CHECK(!McdSpacePairIsEnabled(ball, other));
    pairCounts(space, pairs, false, hello, staying);
    CHECK(hello == 0);
    McdSpaceEnablePair(ball, other);
    pairCounts(space, pairs, false, hello, staying);
    CHECK(hello == 1);
    pairCounts(space, pairs, true, hello, staying);
    CHECK(hello == 0 && staying == 0);
    pairCounts(space, pairs, false, hello, staying);
    CHECK(staying == 1);
    McdCullingTable* table = McdCullingTableCreate(2);
    McdCullingTableSet(table, 0, 1, 1);
    McdSpaceSetModelCullingParameters(space, ball, table, 0, 7);
    McdSpaceSetModelCullingParameters(space, other, table, 1, 7);
    CHECK(!McdSpacePairIsEnabled(ball, other));
    McdSpaceModelIterator mit;
    McdSpaceModelIteratorBegin(space, &mit);
    McdModelID m;
    int models = 0;
    while (McdSpaceGetModel(space, &mit, &m))
        ++models;
    CHECK(models == 2);

    // Removing a pair from a container shifts the ranges after it.
    McdModelPairContainerReset(pairs);
    McdModelPair a{}, b{}, c{};
    McdModelPairContainerInsertGoodbyePair(pairs, &a);
    McdModelPairContainerInsertHelloPair(pairs, &b);
    McdModelPairContainerInsertStayingPair(pairs, &c);
    McdModelPairContainerRemovePair(pairs, &pairs->array[0]);
    CHECK(McdModelPairContainerGetGoodbyeCount(pairs) == 0 && pairs->array[pairs->helloFirst] == &b);

    // The pair manager reports activations as hello and deactivations as goodbye.
    McdModelPairManagerID manager = McdModelPairManagerCreate(4);
    McdSpacePairIterator pit;
    CHECK(McdModelPairManagerActivate(manager, ball, other));
    McdModelPairManagerResetIterator(manager, &pit);
    McdModelPairManagerGetTransitions(manager, &pit, pairs);
    CHECK(McdModelPairContainerGetHelloCount(pairs) == 1);
    McdHelloEach(pairs);
    McdModelPairManagerFlush(manager);
    CHECK(McdModelPairManagerDeactivate(manager, other, ball));
    McdModelPairManagerResetIterator(manager, &pit);
    McdModelPairManagerGetPairs(manager, &pit, pairs);
    CHECK(McdModelPairContainerGetGoodbyeCount(pairs) == 1 && McdModelPairContainerGetStayingCount(pairs) == 0);
    McdModelPairManagerDestroy(manager);

    McdSpaceDestroy(space);
    McdCullingTableDestroy(table);
    McdModelPairContainerDestroy(pairs);
    McdFrameworkDestroyAllModelsAndGeometries(frame);
    CHECK(McdFrameworkGetGeometryCount(frame) == 0 && McdFrameworkGetModelCount(frame) == 0);
    McdTerm(frame);
    return 0;
}
