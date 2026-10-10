#include <cstring>

#include <Mcd.h>
#include <McdBatch.h>
#include <McdPrimitives.h>
#include <McduRequestTable.h>

#include "check.h"

namespace {

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

    // a touches b, b touches c, a and c are apart.
    McdGeometryID sphere = McdSphereCreate(frame, 1);
    McdModelID m[3];
    MeMatrix4 tm[3];
    for (int i = 0; i < 3; ++i)
    {
        m[i] = McdModelCreate(sphere);
        place(tm[i], (MeReal)(1.5 * i));
        McdModelSetTransformPtr(m[i], tm[i]);
        CHECK(McdSpaceInsertModel(space, m[i]));
    }
    McdModelSetRequestID(m[0], 0);
    McdModelSetRequestID(m[1], 1);
    McdModelSetRequestID(m[2], 1);

    McdRequest plain = {0, 0}, single = {1, 1};
    CHECK(!McduRequestTableInit(0, &plain));
    CHECK(McduRequestTableInit(2, &plain));
    McduRequestTableSetRequest(0, 1, &single);
    McduRequestTableSetRequest(5, 0, &single);
    CHECK(McduRequestTableGetRequestPtr(1, 0)->contactMaxCount == 1);
    CHECK(McduRequestTableGetRequestPtr(1, 1)->contactMaxCount == 0);
    CHECK(McduRequestTableGetRequestPtr(2, 0) == nullptr);
    frame->mHelloCallbackFnPtr = McduHelloRequestTableCB;

    McdSpaceUpdateAll(space);
    McdSpacePairIterator it;
    McdSpacePairIteratorBegin(space, &it);
    CHECK(McdSpaceGetPairs(space, &it, pairs) == 0);
    McdHelloEach(pairs);
    CHECK(McdModelPairContainerGetHelloCount(pairs) == 2);
    for (int i = pairs->helloFirst; i < pairs->helloEndStayingFirst; ++i)
    {
        McdModelPair* p = pairs->array[i];
        bool withA = p->model1 == m[0] || p->model2 == m[0];
        CHECK(McdModelPairGetRequestPtr(p) == McduRequestTableGetRequestPtr(withA ? 0 : 1, 1));
        McdRequest r = {-1, -1};
        McduRequestTableGetRequest(p, &r);
        CHECK(r.contactMaxCount == (withA ? 1 : 0));
    }

    // One result slot makes the batch stop after each touching pair and resume on the next call.
    McdBatchContext* batch = McdBatchContextCreate(frame);
    CHECK(batch->frame == frame);
    McdIntersectResult results[1];
    McdContact contacts[8];
    int resultCount = 0, contactCount = 0, total = 0, calls = 0;
    MeBool more;
    do
    {
        more = McdBatchIntersectEach(batch, pairs, results, &resultCount, 1, contacts, &contactCount, 8);
        CHECK(resultCount == 1 && results[0].touch && contactCount >= 1);
        total += resultCount;
        ++calls;
    } while (more && calls < 10);
    CHECK(total == 2 && calls == 2);

    // A finished batch starts over, and so does a reset one.
    McdBatchIntersectEach(batch, pairs, results, &resultCount, 1, contacts, &contactCount, 8);
    McdBatchContextReset(batch);
    CHECK(batch->nextFlattenPair == 0 && batch->frame == frame);
    McdIntersectResult many[4];
    CHECK(!McdBatchIntersectEach(batch, pairs, many, &resultCount, 4, contacts, &contactCount, 8));
    CHECK(resultCount == 2);
    McdBatchContextRelease(batch);
    McdBatchContextDestroy(batch);

    for (int i = pairs->helloFirst; i < pairs->stayingEnd; ++i)
        McdGoodbye(pairs->array[i]);
    McduRequestTableTerm();
    CHECK(McduRequestTableGetRequestPtr(0, 0) == nullptr);
    McdModelPairContainerDestroy(pairs);
    for (McdModelID model : m)
    {
        McdSpaceRemoveModel(model);
        McdModelDestroy(model);
    }
    McdGeometryDestroy(sphere);
    McdSpaceDestroy(space);
    McdTerm(frame);
    return 0;
}
