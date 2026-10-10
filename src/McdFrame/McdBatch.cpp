#include <McdBatch.h>
#include <McdInteractions.h>
#include <McdModelPairContainer.h>

// Karma flattened pairs into per type tables to batch its own narrowphase. Bullet tests pair by pair, so the
// context only keeps where an interrupted McdBatchIntersectEach resumes.

extern "C" {

McdBatchContext* MEAPI McdBatchContextCreate(McdFramework* frame)
{
    auto* c = new McdBatchContext{};
    c->frame = frame;
    return c;
}

void MEAPI McdBatchContextReset(McdBatchContext* context)
{
    McdFramework* frame = context->frame;
    *context = McdBatchContext{};
    context->frame = frame;
}

// No batch buffers are held, so releasing them leaves only the restart.
void MEAPI McdBatchContextRelease(McdBatchContext* context)
{
    McdBatchContextReset(context);
}

void MEAPI McdBatchContextDestroy(McdBatchContext* context)
{
    delete context;
}

// Returns 1 when the result or contact arrays filled up before every pair was tested. The next call with the
// same context and pairs continues from there.
MeBool MEAPI McdBatchIntersectEach(McdBatchContext* context, McdModelPairContainer* pairs,
                                   McdIntersectResult* resultArray, int* resultCount, int resultMaxCount,
                                   McdContact* contactArray, int* contactCount, int contactMaxCount)
{
    McdModelPairContainerIterator it;
    McdModelPairContainerIteratorInit(&it, pairs);
    it.count = context->nextFlattenPair;
    MeBool more = McdIntersectEach(pairs, &it, resultArray, resultMaxCount, resultCount, contactArray,
                                   contactMaxCount, contactCount);
    context->nextFlattenPair = more ? it.count : 0;
    return more;
}

} // extern "C"
