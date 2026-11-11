#include <Mcd.h>
#include <McdConvexMesh.h>
#include <McdPrimitives.h>

#include "check.h"

namespace {

MeVector3 groundVertices[4] = {{-10, 0, -10}, {10, 0, -10}, {10, 0, 10}, {-10, 0, 10}};
MeVector3 up = {0, 1, 0};

int MEAPI ground(McdModelPair*, McdUserTriangle* t, MeVector3, MeReal, int)
{
    int corners[2][3] = {{0, 2, 1}, {0, 3, 2}};
    for (int i = 0; i < 2; ++i)
    {
        for (int k = 0; k < 3; ++k)
            t[i].vertices[k] = &groundVertices[corners[i][k]];
        t[i].normal = &up;
        t[i].flags = kMcdTriangleStandard;
    }
    return 2;
}

McdModelID at(McdGeometryID g, MeMatrix4 tm, MeReal y)
{
    MeMatrix4TMMakeIdentity(tm);
    tm[3][1] = y;
    McdModelID m = McdModelCreate(g);
    McdModelSetTransformPtr(m, tm);
    return m;
}

// Collides a stack copy of model with other the way UT2004 collides one aggregate element: hello, intersect
// and goodbye through the interaction table on a pair that lives on the stack too.
int collideCopy(McdModelID model, McdModelID other)
{
    McdModel copy = *model;
    McdModelPair pair{};
    pair.model1 = &copy;
    pair.model2 = other;
    pair.request = McdFrameworkGetDefaultRequestPtr(model->frame);
    McdInteractions* i =
        McdFrameworkGetInteractions(model->frame, McdModelGetGeometryType(&copy), McdModelGetGeometryType(other));
    CHECK(i && i->intersectFn);
    McdContact contacts[16];
    McdIntersectResult r{};
    r.pair = &pair;
    r.contacts = contacts;
    r.contactMaxCount = 16;
    if (i->helloFn)
        i->helloFn(&pair);
    i->intersectFn(&pair, &r);
    if (i->goodbyeFn)
        i->goodbyeFn(&pair);
    return r.contactCount;
}

} // namespace

int main()
{
    McdFrameworkID frame = McdInit(McdPrimitivesGetTypeCount(), 20, 20, 1);
    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    McdConvexMeshRegisterType(frame);
    McdConvexMeshPrimitivesRegisterInteractions(frame);

    // Callers dispatch on the table themselves, so every primitive pair has an entry.
    const int solids[] = {kMcdGeometryTypeSphere, kMcdGeometryTypeBox, kMcdGeometryTypeCylinder,
                          kMcdGeometryTypeSphyl, kMcdGeometryTypeConvexMesh};
    for (int a : solids)
        for (int b : solids)
        {
            McdInteractions* i = McdFrameworkGetInteractions(frame, a, b);
            bool convexPair = a == kMcdGeometryTypeConvexMesh && b == kMcdGeometryTypeConvexMesh;
            CHECK(i && (i->intersectFn != nullptr) != convexPair);
        }
    McdConvexMeshConvexMeshRegisterInteraction(frame);
    CHECK(McdFrameworkGetInteractions(frame, kMcdGeometryTypeConvexMesh, kMcdGeometryTypeConvexMesh)->intersectFn);

    MeMatrix4 tmA, tmB, tmGround;
    McdModelID box = at(McdBoxCreate(frame, 2, 2, 2), tmA, 0);
    McdModelID sphere = at(McdSphereCreate(frame, 1), tmB, (MeReal)1.5);
    McdModelID terrain =
        at(McdTriangleListCreate(frame, groundVertices[0], groundVertices[2], 8, ground), tmGround, 0);
    // The models were never updated in a space, so the queries must read their current transforms.
    tmA[3][1] = (MeReal)0.9;

    // Hello puts a pair into the order its table entry lists, here the box before the triangle list.
    McdModelPairID ordered = McdModelPairCreate(terrain, box);
    McdHello(ordered);
    CHECK(ordered->model1 == box && ordered->model2 == terrain);
    McdGoodbye(ordered);
    McdModelPairDestroy(ordered);

    CHECK(collideCopy(sphere, box) > 0);
    CHECK(collideCopy(box, terrain) > 0);
    CHECK(collideCopy(sphere, terrain) == 0);
    tmB[3][1] = (MeReal)0.9;
    CHECK(collideCopy(sphere, terrain) > 0);

    McdModelDestroy(box);
    McdModelDestroy(sphere);
    McdModelDestroy(terrain);
    McdTerm(frame);
    return 0;
}
