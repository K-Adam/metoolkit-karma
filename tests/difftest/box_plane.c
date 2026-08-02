/* A unit box sunk 0.05 into a plane: one intersect call, no dynamics. */
#include <string.h>

#include <Mcd.h>
#include <McdPrimitives.h>

#include "trace.h"

int main(void)
{
    McdFrameworkID frame = McdInit(kMcdGeometryBuiltInTypes, 10, 10, 1);
    McdSpaceID space;
    McdModelPairContainer* pairs;
    McdModelID box, plane;
    MeMatrix4 boxTM, planeTM;
    McdSpacePairIterator it;
    McdContact contacts[16];
    McdIntersectResult r;
    MeReal deepest = 0;
    int i;

    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    space = McdSpaceAxisSortCreate(frame, McdAllAxes, 10, 10);
    pairs = McdModelPairContainerCreate(10);
    memset(boxTM, 0, sizeof boxTM);
    memset(planeTM, 0, sizeof planeTM);
    for (i = 0; i < 4; ++i)
        boxTM[i][i] = planeTM[i][i] = 1;
    boxTM[3][2] = 0.45f;

    box = McdModelCreate(McdBoxCreate(frame, 1, 1, 1));
    plane = McdModelCreate(McdPlaneCreate(frame));
    McdModelSetTransformPtr(box, boxTM);
    McdModelSetTransformPtr(plane, planeTM);
    McdSpaceInsertModel(space, box);
    McdSpaceInsertModel(space, plane);
    McdSpaceBuild(space);
    McdSpaceUpdateAll(space);
    McdSpaceEndChanges(space);
    McdSpacePairIteratorBegin(space, &it);
    McdModelPairContainerReset(pairs);
    McdSpaceGetPairs(space, &it, pairs);
    McdHelloEach(pairs);
    EXACT("hello %d", McdModelPairContainerGetHelloCount(pairs));
    if (McdModelPairContainerGetHelloCount(pairs) != 1)
        return 1;

    r.contacts = contacts;
    r.contactMaxCount = 16;
    McdIntersect(pairs->array[pairs->helloFirst], &r);
    EXACT("touch %d contacts %d", r.touch, r.contactCount);
    for (i = 0; i < r.contactCount; ++i)
        if (contacts[i].separation < deepest)
            deepest = contacts[i].separation;
    APPROX(1e-3, "deepest separation %.4f", deepest);
    return 0;
}
