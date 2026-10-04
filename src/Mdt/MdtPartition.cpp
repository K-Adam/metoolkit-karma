#include <MdtPartition.h>

#include "Unsupported.h"

extern "C" {

void MEAPI MdtPartInfoAddConstraint(MdtPartitionInfo* info, MdtBaseConstraint* c)
{
    ME_UNSUPPORTED("MdtPartInfoAddConstraint");
}

void MEAPI MdtPartInfoReset(MdtPartitionInfo* info)
{
    ME_UNSUPPORTED("MdtPartInfoReset");
}

void MEAPI MdtUpdatePartitions(MeDict *enabledBodyList, MdtPartitionOutput* po, const MdtPartitionEndCB pcb,
                               void* pcbdata)
{
    ME_UNSUPPORTED("MdtUpdatePartitions");
}

MdtPartitionOutput* MEAPI MdtPartOutCreateFromChunk(MeChunk* chunk, int maxBodies, int maxConstraints)
{
    ME_UNSUPPORTED("MdtPartOutCreateFromChunk");
    return {};
}

} // extern "C"
