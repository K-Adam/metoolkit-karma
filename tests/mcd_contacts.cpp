#include <Mcd.h>
#include <McdConvexMesh.h>
#include <McdPrimitives.h>

#include "check.h"

namespace {

MeMatrix4 tms[8];
int used = 0;

McdModelID at(McdGeometryID g, MeReal x)
{
    MeVector4* tm = tms[used++];
    MeMatrix4TMMakeIdentity(tm);
    tm[3][0] = x;
    McdModelID m = McdModelCreate(g);
    McdModelSetTransformPtr(m, tm);
    McdModelUpdate(m);
    return m;
}

// Returns the contact count and the smallest separation.
int intersect(McdModelID a, McdModelID b, MeReal* separation)
{
    McdContact contacts[16];
    McdModelPairID pair = McdModelPairCreate(a, b);
    McdIntersectResult r{};
    r.contacts = contacts;
    r.contactMaxCount = 16;
    McdHello(pair);
    McdIntersect(pair, &r);
    McdModelPairDestroy(pair);
    *separation = 1e9f;
    for (int i = 0; i < r.contactCount; ++i)
        *separation = contacts[i].separation < *separation ? contacts[i].separation : *separation;
    return r.contactCount;
}

void move(McdModelID m, MeReal x)
{
    McdModelGetTransformPtr(m)[3][0] = x;
    McdModelUpdate(m);
}

MeReal turn(const McdConvexHull* h, const McdCnvEdge& e1, const McdCnvEdge& e2, const MeReal* up)
{
    MeVector3 d1, d2, n;
    MeVector3Subtract(d1, h->vertex[e1.toVert].position, h->vertex[e1.fromVert].position);
    MeVector3Subtract(d2, h->vertex[e2.toVert].position, h->vertex[e2.fromVert].position);
    MeVector3Cross(n, d1, d2);
    return MeVector3Dot(n, up);
}

void checkHullTopology(const McdConvexHull* h)
{
    CHECK(h->numVertex == 8 && h->numFace == 6 && h->numEdge == 24);
    CHECK(h->face[h->numFace].firstEdge == h->numEdge);
    CHECK(h->vertex[h->numVertex].firstEdgeIndex == h->numEdge);
    for (int f = 0; f < h->numFace; ++f)
    {
        int first = h->face[f].firstEdge, end = h->face[f + 1].firstEdge;
        CHECK(end - first == 4);
        for (int i = first; i < end; ++i)
        {
            const McdCnvEdge& e = h->edge[i];
            const McdCnvEdge& next = h->edge[i + 1 < end ? i + 1 : first];
            CHECK(e.leftFace == f && e.rightFace != f && e.rightFace >= 0);
            CHECK(e.toVert == next.fromVert);
            CHECK(turn(h, e, next, h->face[f].normal) > 0);
        }
    }
    for (int v = 0; v < h->numVertex; ++v)
    {
        int first = h->vertex[v].firstEdgeIndex, end = h->vertex[v + 1].firstEdgeIndex;
        CHECK(end - first == 3);
        for (int i = first; i < end; ++i)
        {
            const McdCnvEdge& e = h->edge[h->edgeIndex[i]];
            const McdCnvEdge& next = h->edge[h->edgeIndex[i + 1 < end ? i + 1 : first]];
            CHECK(e.fromVert == v);
            // The cube is centred on the origin, so a vertex position points outwards.
            CHECK(turn(h, e, next, h->vertex[v].position) > 0);
        }
    }
}

} // namespace

int main()
{
    McdFrameworkID frame = McdInit(McdPrimitivesGetTypeCount(), 20, 20, 1);
    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    McdConvexMeshRegisterType(frame);
    McdConvexMeshPrimitivesRegisterInteractions(frame);
    MeReal sep;

    MeVector3 cube[8];
    for (int i = 0; i < 8; ++i)
        for (int k = 0; k < 3; ++k)
            cube[i][k] = (i >> k & 1) ? 0.5f : -0.5f;
    McdGeometryID fat = McdConvexMeshCreateHull(frame, cube, 8, 0.1f);
    checkHullTopology(McdConvexMeshGetPolyhedron(fat));

    // Fatness rounds both meshes, so mesh-mesh sinks twice as deep as mesh-box at the same distance.
    McdModelID meshA = at(fat, 0), meshB = at(fat, 1.05f);
    McdModelID box = at(McdBoxCreate(frame, 1, 1, 1), 1.05f);
    CHECK(intersect(meshA, meshB, &sep) > 0);
    CHECK_NEAR(sep, -0.15, 0.01);
    CHECK(intersect(meshA, box, &sep) > 0);
    CHECK_NEAR(sep, -0.05, 0.01);

    // Contacts reach out to the summed contact tolerance, beyond Bullet's own breaking threshold, and no further.
    McdGeometryID ball = McdSphereCreate(frame, 1);
    McdModelID pairs[2][2] = {{at(ball, 0), at(ball, 0)}, {at(McdBoxCreate(frame, 1, 1, 1), 0), box}};
    MeReal sizes[2] = {2, 1};
    for (int k = 0; k < 2; ++k)
    {
        McdModelID a = pairs[k][0], b = pairs[k][1];
        McdModelSetContactTolerance(a, 0.05f);
        McdModelSetContactTolerance(b, 0.05f);
        move(b, sizes[k] + 0.08f);
        CHECK(intersect(a, b, &sep) > 0);
        CHECK_NEAR(sep, 0.08, 0.01);
        move(b, sizes[k] + 0.12f);
        CHECK(intersect(a, b, &sep) == 0);
        McdModelSetContactTolerance(a, 0);
        McdModelSetContactTolerance(b, 0);
        move(b, sizes[k] + 0.01f);
        CHECK(intersect(a, b, &sep) == 0);
    }

    McdFrameworkDestroyAllModelsAndGeometries(frame);
    McdTerm(frame);
    return 0;
}
