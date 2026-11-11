#include <MdtPartition.h>
#include <MeChunk.h>

#include "MdtImpl.h"
#include "Unsupported.h"

extern "C" {

void MEAPI MdtPartInfoAddConstraint(MdtPartitionInfo* info, MdtBaseConstraint* c)
{
    info->rowCount += c->head.maxRows;
    if (c->head.tag == MdtBclCONTACTGROUP || c->head.tag == MdtBclCONTACT)
        info->contactCount += mdtKeaConstraintCount(c);
    else
        ++info->jointCount;
}

void MEAPI MdtPartInfoReset(MdtPartitionInfo* info)
{
    *info = MdtPartitionInfo{};
}

void MEAPI MdtUpdatePartitions(MeDict *enabledBodyList, MdtPartitionOutput* po, const MdtPartitionEndCB pcb,
                               void* pcbdata)
{
    ME_UNSUPPORTED("MdtUpdatePartitions");
}

// Every partition holds at least one body, so there are at most as many partitions as bodies.
MdtPartitionOutput* MEAPI MdtPartOutCreateFromChunk(MeChunk* chunk, int maxBodies, int maxConstraints)
{
    return mdtCarve(chunk, [&](MdtChunkCarver& m) {
        MdtPartitionOutput* po = m.take<MdtPartitionOutput>(1);
        MdtBaseConstraint** constraints = m.take<MdtBaseConstraint*>(maxConstraints);
        int* constraintsStart = m.take<int>(maxBodies);
        int* constraintsSize = m.take<int>(maxBodies);
        MdtBody** bodies = m.take<MdtBody*>(maxBodies);
        int* bodiesStart = m.take<int>(maxBodies);
        int* bodiesSize = m.take<int>(maxBodies);
        MdtPartitionInfo* info = m.take<MdtPartitionInfo>(maxBodies);
        if (!po)
            return po;
        *po = MdtPartitionOutput{};
        po->constraints = constraints;
        po->constraintsStart = constraintsStart;
        po->constraintsSize = constraintsSize;
        po->maxConstraints = maxConstraints;
        po->bodies = bodies;
        po->bodiesStart = bodiesStart;
        po->bodiesSize = bodiesSize;
        po->maxBodies = maxBodies;
        po->info = info;
        po->maxPartitions = maxBodies;
        return po;
    });
}

} // extern "C"
