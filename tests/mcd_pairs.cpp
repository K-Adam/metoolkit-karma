#include <cstring>

#include <Mcd.h>
#include <McdPrimitives.h>

#include "check.h"

namespace {

struct Phases
{
    int hello, staying, goodbye;
};

Phases update(McdSpaceID space, McdModelPairContainer* pairs, McdModelPair** current)
{
    McdSpaceUpdateAll(space);
    McdSpacePairIterator it;
    McdSpacePairIteratorBegin(space, &it);
    CHECK(McdSpaceGetPairs(space, &it, pairs) == 0);
    McdGoodbyeEach(pairs);
    McdHelloEach(pairs);
    Phases ph = {McdModelPairContainerGetHelloCount(pairs), McdModelPairContainerGetStayingCount(pairs),
                 McdModelPairContainerGetGoodbyeCount(pairs)};
    *current = pairs->stayingEnd > pairs->helloFirst ? pairs->array[pairs->helloFirst] : nullptr;
    return ph;
}

void place(MeMatrix4 tm, MeReal x)
{
    std::memset(tm, 0, sizeof(MeMatrix4));
    tm[0][0] = tm[1][1] = tm[2][2] = tm[3][3] = 1;
    tm[3][0] = x;
}

} // namespace

int main()
{
    McdFrameworkID frame = McdInit(McdPrimitivesGetTypeCount(), 10, 10, 1);
    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    McdSpaceID space = McdSpaceAxisSortCreate(frame, McdAllAxes, 10, 10);
    McdModelPairContainer* pairs = McdModelPairContainerCreate(10);

    McdGeometryID sphere = McdSphereCreate(frame, 1);
    CHECK(McdGeometryGetTypeId(sphere) == kMcdGeometryTypeSphere);
    McdModelID a = McdModelCreate(sphere), b = McdModelCreate(sphere);
    CHECK(McdGeometryGetReferenceCount(sphere) == 2);
    MeMatrix4 tmA, tmB;
    place(tmA, 0);
    place(tmB, 10);
    McdModelSetTransformPtr(a, tmA);
    McdModelSetTransformPtr(b, tmB);
    CHECK(McdSpaceInsertModel(space, a) && McdSpaceInsertModel(space, b));

    McdModelPair* pair;
    Phases ph = update(space, pairs, &pair);
    CHECK(ph.hello == 0 && ph.staying == 0 && ph.goodbye == 0);

    place(tmB, (MeReal)1.8);
    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 1 && ph.staying == 0 && ph.goodbye == 0);

    McdContact contacts[8];
    McdIntersectResult result;
    result.contacts = contacts;
    result.contactMaxCount = 8;
    CHECK(McdIntersect(pair, &result));
    CHECK(result.contactCount >= 1);
    // The normal points from model2 to model1, as in the original library, and the spheres overlap by 0.2.
    McdModelID m1, m2;
    McdModelPairGetModels(pair, &m1, &m2);
    MeReal towards1 = m1 == a ? -1 : 1;
    CHECK_NEAR(contacts[0].normal[0], towards1, 1e-4);
    CHECK_NEAR(contacts[0].separation, -0.2, 1e-3);

    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 0 && ph.staying == 1 && ph.goodbye == 0);

    // A pair whose model leaves the space must not be reported as staying any more.
    McdSpaceRemoveModel(b);
    McdSpacePairIterator it;
    McdSpacePairIteratorBegin(space, &it);
    McdSpaceGetPairs(space, &it, pairs);
    CHECK(McdModelPairContainerGetStayingCount(pairs) == 0);
    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 0 && ph.staying == 0 && ph.goodbye == 1);
    CHECK(McdSpaceInsertModel(space, b));
    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 1 && ph.staying == 0 && ph.goodbye == 0);

    place(tmB, 10);
    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 0 && ph.staying == 0 && ph.goodbye == 1);
    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 0 && ph.staying == 0 && ph.goodbye == 0);

    // A pair pending goodbye lets go of its model when the model leaves the space, as the model may be destroyed
    // before the application says goodbye.
    place(tmB, (MeReal)1.8);
    ph = update(space, pairs, &pair);
    CHECK(ph.hello == 1 && pair->m_cachedData);
    place(tmB, 10);
    McdSpaceUpdateAll(space);
    McdSpacePairIteratorBegin(space, &it);
    McdSpaceGetPairs(space, &it, pairs);
    CHECK(McdModelPairContainerGetGoodbyeCount(pairs) == 1 && pairs->array[pairs->goodbyeFirst] == pair);
    McdSpaceRemoveModel(b);
    CHECK(pair->phase == kMcdFFStateGoodbye && !pair->m_cachedData);
    McdGoodbyeEach(pairs);

    McdSpaceRemoveModel(a);
    McdSpaceRemoveModel(b);
    McdModelDestroy(a);
    McdModelDestroy(b);
    McdGeometryDestroy(sphere);
    CHECK(McdFrameworkGetGeometryCount(frame) == 0);
    McdModelPairContainerDestroy(pairs);
    McdSpaceDestroy(space);
    McdTerm(frame);
    return 0;
}
