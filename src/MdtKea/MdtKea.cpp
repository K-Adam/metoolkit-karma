#include <MdtKea.h>

#include "Unsupported.h"

extern "C" {

MeCPUResources MEAPI MdtKeaQueryCPUResources(void)
{
    ME_UNSUPPORTED("MdtKeaQueryCPUResources");
    return {};
}

int MEAPI MdtKeaMemoryRequired(const int num_rows_exc_padding[], int num_partitions, int max_rows, int max_bodies)
{
    ME_UNSUPPORTED("MdtKeaMemoryRequired");
    return {};
}

void MEAPI MdtKeaAddConstraintForces(MdtKeaConstraints constraints, MdtKeaBody * blist[],
                                     const MdtKeaTransformation tlist[], int num_bodies, MdtKeaParameters parameters)
{
    ME_UNSUPPORTED("MdtKeaAddConstraintForces");
}

void MEAPI MdtKeaIntegrateSystem(MdtKeaBody *const blist[], MdtKeaTransformation tlist[], int num_bodies,
                                 MdtKeaParameters parameters)
{
    ME_UNSUPPORTED("MdtKeaIntegrateSystem");
}

} // extern "C"
