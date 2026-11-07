#include <Mcd.h>
#include <McdCullingTable.h>
#include <McdModelPairManager.h>
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
    MeMatrix4TMMakeIdentity(tm);
    MeMatrix4TMSetPosition(tm, x, 0, 0);
}

int hellos = 0;

void MEAPI countHello(McdModelPair*)
{
    ++hellos;
}

// Objects destroyed in any order, or left to McdTerm, never leave a pair or a space reading a freed model.
void lifetime()
{
    McdFrameworkID frame = McdInit(McdPrimitivesGetTypeCount(), 10, 10, 1);
    McdPrimitivesRegisterTypes(frame);
    McdPrimitivesRegisterInteractions(frame);
    McdGeometryID sphere = McdSphereCreate(frame, 1);
    MeMatrix4 tmA, tmB, tmC;
    place(tmA, 0);
    place(tmB, (MeReal)1.5);
    place(tmC, 20);

    frame->mHelloCallbackFnPtr = countHello;
    McdModelID a = McdModelCreate(sphere), b = McdModelCreate(sphere);
    McdModelSetTransformPtr(a, tmA);
    McdModelSetTransformPtr(b, tmB);
    McdModelPairManagerID manager = McdModelPairManagerCreate(4);
    CHECK(McdModelPairManagerActivate(manager, a, b));
    McdModelPair* p = McdModelPairManagerGetPair(manager, a, b);
    McdHello(p);
    McdHello(p);
    CHECK(hellos == 1);
    frame->mHelloCallbackFnPtr = nullptr;
    McdModelDestroy(a);
    CHECK(p->phase == kMcdFFStateGoodbye && !p->m_cachedData);
    McdModelDestroy(b);
    McdModelPairManagerDestroy(manager);

    // A model that leaves the space forgets its culling table.
    McdSpaceID space = McdSpaceAxisSortCreate(frame, McdAllAxes, 10, 10);
    McdModelPairContainer* pairs = McdModelPairContainerCreate(10);
    a = McdModelCreate(sphere);
    b = McdModelCreate(sphere);
    McdModelSetTransformPtr(a, tmA);
    McdModelSetTransformPtr(b, tmB);
    McdCullingTable* table = McdCullingTableCreate(2);
    McdCullingTableSet(table, 0, 1, 1);
    CHECK(McdSpaceInsertModelWithCulling(space, a, table, 0, 7));
    CHECK(McdSpaceInsertModelWithCulling(space, b, table, 1, 7));
    McdModelPair* current;
    CHECK(update(space, pairs, &current).hello == 0);
    McdSpaceRemoveModel(a);
    McdCullingTableDestroy(table);
    CHECK(McdSpaceInsertModel(space, a));
    CHECK(update(space, pairs, &current).hello == 1);

    // Removing each model while iterating still visits every model.
    McdModelID c = McdModelCreate(sphere);
    McdModelSetTransformPtr(c, tmC);
    CHECK(McdSpaceInsertModel(space, c));
    McdSpaceModelIterator it;
    McdSpaceModelIteratorBegin(space, &it);
    McdModelID m;
    int seen = 0;
    while (McdSpaceGetModel(space, &it, &m))
    {
        McdSpaceRemoveModel(m);
        McdModelDestroy(m);
        ++seen;
    }
    CHECK(seen == 3 && McdSpaceGetModelCount(space) == 0);

    // McdTerm releases whatever is left.
    a = McdModelCreate(sphere);
    McdModelSetTransformPtr(a, tmA);
    CHECK(McdSpaceInsertModel(space, a));
    McdModelPairContainerDestroy(pairs);
    McdTerm(frame);
}

} // namespace

int main()
{
    lifetime();
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
