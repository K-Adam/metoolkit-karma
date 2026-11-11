#include <Mcd.h>
#include <McdConvexMesh.h>
#include <McdPrimitives.h>
#include <MeApp.h>
#include <MeViewer.h>
#include <RMenu.h>

#include "check.h"

namespace {

void checkBounds(const MeReal* b, MeReal minX, MeReal maxX, MeReal minY, MeReal maxY)
{
    CHECK_NEAR(b[0], minX, 1e-5);
    CHECK_NEAR(b[1], maxX, 1e-5);
    CHECK_NEAR(b[2], minY, 1e-5);
    CHECK_NEAR(b[3], maxY, 1e-5);
}

} // namespace

int main()
{
    CHECK(RRenderContextCreate2(nullptr, nullptr, 0, nullptr, nullptr) == nullptr);
    CHECK(RGraphicBoxCreate(nullptr, 1, 1, 1, nullptr, nullptr) == nullptr);
    CHECK(RMenuCreate(nullptr, "menu") == nullptr);
    CHECK(RCameraGetDistance(nullptr) == 0);
    CHECK(MeAppCreate(nullptr, nullptr, nullptr) == nullptr);

    McdFrameworkID frame = McdInit(0, 100, 0, 1);
    McdPrimitivesRegisterTypes(frame);
    McdConvexMeshRegisterType(frame);
    // Translated to (10, 20, 30), local x along world y, y along z and z along x.
    MeMatrix4 tm = {{0, 1, 0, 0}, {0, 0, 1, 0}, {1, 0, 0, 0}, {10, 20, 30, 1}};
    MeReal b[4];

    // Expected values were recorded from the original library.
    McdGeometryID box = McdBoxCreate(frame, 2, 4, 6);
    McdBoxGetXYAABB(box, tm, b);
    checkBounds(b, 7, 13, 19, 21);

    McdGeometryID cyl = McdCylinderCreate(frame, 1, 4);
    McdCylinderGetXYAABB(cyl, tm, b);
    checkBounds(b, 8, 12, 19, 21);

    MeVector3 v[8];
    for (int i = 0; i < 8; ++i)
    {
        v[i][0] = (i & 1) ? 1 : -1;
        v[i][1] = (i & 2) ? 2 : -2;
        v[i][2] = (i & 4) ? 3 : -3;
    }
    McdGeometryID hull = McdConvexMeshCreateHull(frame, v, 8, 0.5f);
    McdConvexMeshGetXYAABB(hull, tm, b);
    checkBounds(b, 6.5, 13.5, 18.5, 21.5);

    McdGeometryDestroy(hull);
    McdGeometryDestroy(cyl);
    McdGeometryDestroy(box);
    McdTerm(frame);
    return 0;
}
