/* Pure collision: sphere B is scripted along x through approach, overlap and separation. */
#include <string.h>

#include <Mcd.h>
#include <McdPrimitives.h>

#include "trace.h"

static void identity(MeMatrix4 tm, MeReal x)
{
    memset(tm, 0, sizeof(MeMatrix4));
    tm[0][0] = tm[1][1] = tm[2][2] = tm[3][3] = 1;
    tm[3][0] = x;
}

int main(void)
{
    /* B's positions stay clear of the broadphase boundary (x = 2 plus padding) so margins cannot matter. */
    static const MeReal xs[] = {6, 4, 1.9f, 1.5f, 1.2f, 1.5f, 1.9f, 4, 6};
    McdFrameworkID frame = McdInit(kMcdGeometryBuiltInTypes, 10, 10, 1);
    McdSpaceID space;
    McdModelPairContainer* pairs;
    McdGeometryID sphere;
    McdModelID a, b;
    MeMatrix4 tmA, tmB;
    unsigned s;

    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    space = McdSpaceAxisSortCreate(frame, McdAllAxes, 10, 10);
    pairs = McdModelPairContainerCreate(10);
    sphere = McdSphereCreate(frame, 1);
    a = McdModelCreate(sphere);
    b = McdModelCreate(sphere);
    identity(tmA, 0);
    identity(tmB, xs[0]);
    McdModelSetTransformPtr(a, tmA);
    McdModelSetTransformPtr(b, tmB);
    McdSpaceInsertModel(space, a);
    McdSpaceInsertModel(space, b);
    McdSpaceBuild(space);

    /* The documented loop: models move inside a change block, pairs are read outside it. */
    for (s = 0; s < sizeof xs / sizeof xs[0]; ++s)
    {
        McdSpacePairIterator it;
        int i;
        tmB[3][0] = xs[s];
        McdSpaceUpdateAll(space);
        McdSpaceEndChanges(space);
        McdSpacePairIteratorBegin(space, &it);
        McdModelPairContainerReset(pairs);
        McdSpaceGetPairs(space, &it, pairs);
        McdGoodbyeEach(pairs);
        McdHelloEach(pairs);
        EXACT("x %.2f hello %d staying %d goodbye %d", xs[s], McdModelPairContainerGetHelloCount(pairs),
              McdModelPairContainerGetStayingCount(pairs), McdModelPairContainerGetGoodbyeCount(pairs));

        for (i = pairs->helloFirst; i < pairs->stayingEnd; ++i)
        {
            McdModelPair* p = pairs->array[i];
            McdContact contacts[8];
            McdIntersectResult r;
            /* Pair order is an implementation detail, so the normal is reported as if A were model1. */
            MeReal sign = p->model1 == a ? 1.0f : -1.0f;
            r.contacts = contacts;
            r.contactMaxCount = 8;
            McdIntersect(p, &r);
            EXACT("  touch %d contacts %d", r.touch, r.contactCount);
            if (r.contactCount > 0)
                APPROX(1e-4, "  normal %.5f %.5f %.5f separation %.5f", sign * contacts[0].normal[0],
                       sign * contacts[0].normal[1], sign * contacts[0].normal[2], contacts[0].separation);
        }
        McdSpaceBeginChanges(space);
    }
    return 0;
}
