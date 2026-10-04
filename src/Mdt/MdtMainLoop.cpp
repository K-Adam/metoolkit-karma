#include <MdtMainLoop.h>

#include "Unsupported.h"

extern "C" {

void MEAPI MdtAutoDisableLastPartition(MdtPartitionOutput* po, void* cbdata)
{
    ME_UNSUPPORTED("MdtAutoDisableLastPartition");
}

unsigned int MEAPI MdtPackAllPartitions(const MdtPartitionOutput* po, const MeReal stepSize, MdtWorldParams* params,
                                        MdtKeaParameters* keaParams, MdtKeaTransformation* keaTMArray,
                                        MdtKeaConstraints* constraints)
{
    ME_UNSUPPORTED("MdtPackAllPartitions");
    return {};
}

unsigned int MEAPI MdtPackPartition(const MdtPartitionOutput* po, const unsigned int partitionindex,
                                    const MeReal stepSize, MdtWorldParams* params, MdtKeaParameters* keaParams,
                                    MdtKeaTransformation* keaTMArray, MdtKeaConstraints* constraints)
{
    ME_UNSUPPORTED("MdtPackPartition");
    return {};
}

void MEAPI MdtUnpackBodies(MdtKeaTransformation* keaTMArray, const unsigned int partitionindex, MdtPartitionOutput* po)
{
    ME_UNSUPPORTED("MdtUnpackBodies");
}

unsigned int MEAPI MdtUnpackForces(const MdtKeaForcePair *force, const unsigned int partitionindex,
                                   MdtPartitionOutput* po)
{
    ME_UNSUPPORTED("MdtUnpackForces");
    return {};
}

MeReal MEAPI MdtPartitionGetSafeTime(MdtPartitionOutput* po, int i)
{
    ME_UNSUPPORTED("MdtPartitionGetSafeTime");
    return {};
}

void MEAPI MdtLODLastPartition(MdtPartitionOutput* po, MdtPartitionParams* params)
{
    ME_UNSUPPORTED("MdtLODLastPartition");
}

MdtKeaConstraints* MEAPI MdtKeaConstraintsCreateFromChunk(MeChunk* chunk, int maxPartitions, int maxKeaConstraints,
                                                          int maxRows)
{
    ME_UNSUPPORTED("MdtKeaConstraintsCreateFromChunk");
    return {};
}

} // extern "C"
