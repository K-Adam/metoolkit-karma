#include <algorithm>
#include <array>
#include <new>
#include <cmath>
#include <vector>

#include <LinearMath/btConvexHullComputer.h>

#include <McdConvexMesh.h>
#include <McdGjkMaximumPoint.h>

#include "McdImpl.h"

// The hull is recomputed with btConvexHullComputer, which merges coplanar triangles into polygons. Fatness
// becomes the margin of the btConvexHullShape, so Bullet rounds the hull by it just as Karma does.

namespace {

struct ConvexImpl
{
    McdConvexMesh pub;
    btConvexHullShape shape;
    // Face and vertex arrays end with the sentinel McdQHullTypes.h requires.
    std::vector<McdCnvVertex> vertices;
    std::vector<McdCnvFace> faces;
    std::vector<McdCnvEdge> edges;
    std::vector<int> edgeIndex;
};

ConvexImpl* get(McdGeometryID g) { return reinterpret_cast<ConvexImpl*>(g); }

btCollisionShape* shapeOf(McdGeometryID g) { return &get(g)->shape; }

const McdCnvEdge& faceEdge(ConvexImpl* c, int face, int i) { return c->edges[c->faces[face].firstEdge + i]; }

int faceEdgeCount(ConvexImpl* c, int face) { return c->faces[face + 1].firstEdge - c->faces[face].firstEdge; }

void build(ConvexImpl* c, const MeVector3* points, int count, MeReal fatness)
{
    btConvexHullComputer hull;
    if (count > 0)
        hull.compute(&points[0][0], (int)sizeof(MeVector3), count, 0, 0);
    const btConvexHullComputer::Edge* base = hull.edges.size() ? &hull.edges[0] : nullptr;
    auto indexOf = [base](const btConvexHullComputer::Edge* e) { return (int)(e - base); };

    // Hull faces are walked anticlockwise from outside, which orders the edge array by leftFace and ACW.
    std::vector<int> order, leftFace(hull.edges.size(), -1);
    c->faces.assign(hull.faces.size() + 1, McdCnvFace{});
    for (int f = 0; f < hull.faces.size(); ++f)
    {
        c->faces[f].firstEdge = (int)order.size();
        const btConvexHullComputer::Edge* first = &hull.edges[hull.faces[f]];
        const btConvexHullComputer::Edge* e = first;
        do
        {
            leftFace[indexOf(e)] = f;
            order.push_back(indexOf(e));
            e = e->getNextEdgeOfFace();
        } while (e != first);
    }
    c->faces.back().firstEdge = (int)order.size();
    // A degenerate hull can have edges outside every face.
    for (int i = 0; i < hull.edges.size(); ++i)
        if (leftFace[i] < 0)
            order.push_back(i);

    std::vector<int> position(order.size());
    c->edges.assign(order.size(), McdCnvEdge{});
    for (size_t i = 0; i < order.size(); ++i)
    {
        const btConvexHullComputer::Edge& e = hull.edges[order[i]];
        position[order[i]] = (int)i;
        McdCnvEdge& out = c->edges[i];
        out.fromVert = e.getSourceVertex();
        out.toVert = e.getTargetVertex();
        out.leftFace = leftFace[order[i]];
        out.rightFace = leftFace[indexOf(e.getReverseEdge())];
        out.invLength = 1 / (hull.vertices[out.toVert] - hull.vertices[out.fromVert]).length();
    }
    // A collinear hull has a two-edge face with no normal.
    for (int f = 0; f < hull.faces.size() && faceEdgeCount(c, f) >= 3; ++f)
    {
        auto corner = [&](int i) { return hull.vertices[faceEdge(c, f, i).fromVert]; };
        btVector3 n = (corner(1) - corner(0)).cross(corner(2) - corner(0));
        fromBt(n.normalized(), c->faces[f].normal);
    }

    // Bullet steps clockwise around a vertex seen from outside, so each ring is stored reversed.
    std::vector<const btConvexHullComputer::Edge*> firstOut(hull.vertices.size(), nullptr);
    for (int i = 0; i < hull.edges.size(); ++i)
        if (!firstOut[hull.edges[i].getSourceVertex()])
            firstOut[hull.edges[i].getSourceVertex()] = &hull.edges[i];
    c->vertices.assign(hull.vertices.size() + 1, McdCnvVertex{});
    c->edgeIndex.clear();
    for (int v = 0; v < hull.vertices.size(); ++v)
    {
        fromBt(hull.vertices[v], c->vertices[v].position);
        c->vertices[v].firstEdgeIndex = (int)c->edgeIndex.size();
        if (const btConvexHullComputer::Edge* first = firstOut[v])
        {
            const btConvexHullComputer::Edge* e = first;
            do
            {
                c->edgeIndex.push_back(position[indexOf(e)]);
                e = e->getNextEdgeOfVertex();
            } while (e != first);
            std::reverse(c->edgeIndex.begin() + c->vertices[v].firstEdgeIndex, c->edgeIndex.end());
        }
    }
    c->vertices.back().firstEdgeIndex = (int)c->edgeIndex.size();

    McdConvexHull& h = c->pub.mHull;
    h.vertex = c->vertices.data();
    h.face = c->faces.data();
    h.edge = c->edges.data();
    h.edgeIndex = c->edgeIndex.data();
    h.numVertex = (int)c->vertices.size() - 1;
    h.numFace = (int)c->faces.size() - 1;
    h.numEdge = (int)c->edges.size();
    c->pub.mFatness = fatness;

    btVector3 center(0, 0, 0);
    for (int i = 0; i < hull.vertices.size(); ++i)
        center += hull.vertices[i] / (btScalar)hull.vertices.size();
    btScalar radius = 0;
    for (int i = 0; i < hull.vertices.size(); ++i)
        radius = std::max(radius, (hull.vertices[i] - center).length());
    fromBt(center, c->pub.mBoundingSphereCenter);
    c->pub.mBoundingSphereRadius = radius + fatness;

    // Rebuilding in place keeps the shape address models hold.
    c->shape.~btConvexHullShape();
    new (&c->shape) btConvexHullShape(hull.vertices.size() ? &hull.vertices[0][0] : nullptr, hull.vertices.size(),
                                      sizeof(btVector3));
    // No polyhedral features: Bullet's hull clipping for them ignores the margin, so fatness would be lost.
    c->shape.setMargin(fatness);
}

McdConvexMeshID create(McdFramework* frame, const MeVector3* points, int count, MeReal fatness)
{
    auto* c = new ConvexImpl{};
    McdGeometryInit(&c->pub.m_g, frame, kMcdGeometryTypeConvexMesh);
    build(c, points, count, fatness);
    return &c->pub.m_g;
}

std::vector<std::array<MeReal, 3>> hullPoints(const McdConvexHull* poly)
{
    std::vector<std::array<MeReal, 3>> points(poly ? poly->numVertex : 0);
    for (size_t i = 0; i < points.size(); ++i)
        std::copy(poly->vertex[i].position, poly->vertex[i].position + 3, points[i].data());
    return points;
}

} // namespace

extern "C" {

MeI16 MEAPI McdConvexMeshGetTypeId()
{
    return kMcdGeometryTypeConvexMesh;
}

void MEAPI McdConvexMeshRegisterType(McdFramework* frame)
{
    McdFrameworkRegisterGeometryType(frame, kMcdGeometryTypeConvexMesh, const_cast<char*>("ConvexMesh"),
                                     McdConvexMeshDestroy, McdConvexMeshUpdateAABB, McdConvexMeshGetBSphere,
                                     McdConvexMeshMaximumPoint, McdConvexMeshGetMassProperties, nullptr);
    mcdRegisterShapeFn(frame, kMcdGeometryTypeConvexMesh, shapeOf);
}

McdConvexMeshID MEAPI McdConvexMeshCreate(McdFramework* frame, McdConvexHull* poly, MeReal fatness)
{
    std::vector<std::array<MeReal, 3>> points = hullPoints(poly);
    return create(frame, reinterpret_cast<const MeVector3*>(points.data()), (int)points.size(), fatness);
}

McdConvexMeshID MEAPI McdConvexMeshCreateHull(McdFramework* frame, const MeVector3* vertices, int vertexCount,
                                              MeReal fatnessRadius)
{
    return create(frame, vertices, vertexCount, fatnessRadius);
}

// A segment rounded by the radius. It has no polygons.
McdConvexMeshID MEAPI McdConvexMeshCreateSphyl(McdFramework* frame, MeReal radius, MeReal height)
{
    MeVector3 ends[2] = {{0, 0, -height / 2}, {0, 0, height / 2}};
    return create(frame, ends, 2, radius);
}

void MEAPI McdConvexMeshDestroy(McdConvexMeshID g)
{
    McdGeometryDeinit(g);
    delete get(g);
}

void MEAPI McdConvexMeshSetPolyhedron(McdConvexMeshID g, McdConvexHull* poly, MeReal fatness)
{
    std::vector<std::array<MeReal, 3>> points = hullPoints(poly);
    build(get(g), reinterpret_cast<const MeVector3*>(points.data()), (int)points.size(), fatness);
}

const McdConvexHull* MEAPI McdConvexMeshGetPolyhedron(McdConvexMeshID g)
{
    return &get(g)->pub.mHull;
}

MeReal MEAPI McdConvexMeshGetFatness(McdConvexMeshID mesh)
{
    return get(mesh)->pub.mFatness;
}

int MEAPI McdConvexMeshGetPolygonCount(McdConvexMeshID g)
{
    return get(g)->pub.mHull.numFace;
}

int MEAPI McdConvexMeshGetPolygonVertexCount(McdConvexMeshID g, int polyID)
{
    return faceEdgeCount(get(g), polyID);
}

const MeReal* MEAPI McdConvexMeshGetPolygonVertexPtr(McdConvexMeshID g, int polyID, int vertexID)
{
    ConvexImpl* c = get(g);
    return c->vertices[faceEdge(c, polyID, vertexID).fromVert].position;
}

void MEAPI McdConvexMeshGetPolygonVertex(McdConvexMeshID g, int polyID, int vertexID, MeVector3 v)
{
    const MeReal* p = McdConvexMeshGetPolygonVertexPtr(g, polyID, vertexID);
    std::copy(p, p + 3, v);
}

void MEAPI McdConvexMeshGetPolygonNormal(McdConvexMeshID g, int polyID, MeVector3 n)
{
    const MeReal* p = get(g)->faces[polyID].normal;
    std::copy(p, p + 3, n);
}

void MEAPI McdConvexMeshUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool)
{
    mcdShapeAABB(ins, finalTM);
}

void MEAPI McdConvexMeshGetBSphere(McdGeometryID g, MeVector3 center, MeReal* radius)
{
    const McdConvexMesh& c = get(g)->pub;
    std::copy(c.mBoundingSphereCenter, c.mBoundingSphereCenter + 3, center);
    *radius = c.mBoundingSphereRadius;
}

void MEAPI McdConvexMeshMaximumPoint(McdGeometryInstanceID ins, MeReal* const inDir, MeReal* const outPoint)
{
    McdConvexMeshMaximumPointNew(ins, inDir, outPoint);
}

MeReal MEAPI McdConvexMeshMaximumPointNew(McdGeometryInstanceID ins, const MeVector3 inDir, MeVector3 outPoint)
{
    btTransform t = toBtTransform(ins->mTM);
    btVector3 world = t * get(ins->mGeometry)->shape.localGetSupportingVertex(t.getBasis().transpose() * toBt(inDir));
    fromBt(world, outPoint);
    return world.dot(toBt(inDir));
}

// Mass properties of the polyhedron itself, at unit density. The volume added by fatness is not included.
MeI16 MEAPI McdConvexMeshGetMassProperties(McdConvexMeshID mesh, MeMatrix4 relTM, MeMatrix3 m, MeReal* volume)
{
    ConvexImpl* c = get(mesh);
    btVector3 ref = toBt(c->pub.mBoundingSphereCenter);
    btScalar total = 0;
    btVector3 moment(0, 0, 0);
    btMatrix3x3 cov(0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto outer = [](const btVector3& a, const btVector3& b) {
        return btMatrix3x3(a.x() * b.x(), a.x() * b.y(), a.x() * b.z(), a.y() * b.x(), a.y() * b.y(), a.y() * b.z(),
                           a.z() * b.x(), a.z() * b.y(), a.z() * b.z());
    };
    // Fan each face into tetrahedra with the reference point as apex.
    for (int f = 0; f < c->pub.mHull.numFace; ++f)
    {
        auto corner = [c, f, &ref](int i) { return toBt(c->vertices[faceEdge(c, f, i).fromVert].position) - ref; };
        btVector3 a = corner(0);
        for (int i = 1; i + 1 < faceEdgeCount(c, f); ++i)
        {
            btVector3 b = corner(i), d = corner(i + 1);
            btScalar det = a.dot(b.cross(d));
            total += det / 6;
            moment += (a + b + d) * (det / 24);
            btMatrix3x3 sum = outer(a, a) + outer(b, b) + outer(d, d) + outer(a + b + d, a + b + d);
            for (int r = 0; r < 3; ++r)
                cov[r] += sum[r] * (det / 120);
        }
    }
    mcdSetDiagonalInertia(relTM, m, 0, 0, 0);
    *volume = total;
    if (total <= 0)
        return 0;
    btVector3 com = moment / total;
    btMatrix3x3 shifted = outer(com, com);
    for (int r = 0; r < 3; ++r)
        cov[r] -= shifted[r] * total;
    btScalar trace = cov[0][0] + cov[1][1] + cov[2][2];
    for (int r = 0; r < 3; ++r)
        for (int col = 0; col < 3; ++col)
            m[r][col] = ((r == col ? trace : 0) - cov[r][col]) / total;
    fromBt(ref + com, relTM[3]);
    return 1;
}

char* MEAPI McdConvexMeshGetTypeName(McdGeometryID)
{
    return const_cast<char*>("ConvexMesh");
}

} // extern "C"
