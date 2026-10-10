#include <MdtBcl.h>

namespace {

// Karma adds a whole block of padding whenever a constraint leaves the padded row count off a multiple of 4.
void addPadded(int& rows, unsigned added)
{
    rows += added;
    if (rows % 4)
        rows += 4;
}

} // namespace

extern "C" {

void MEAPI MdtBclInitConstraintRowList(MdtKeaConstraints* const clist)
{
    clist->num_partitions = 0;
    clist->num_rows_inc_padding = 0;
    clist->num_rows_exc_padding = 0;
    clist->num_constraints = 0;
}

void MEAPI MdtBclStartPartition(MdtKeaConstraints* const clist)
{
    int p = clist->num_partitions;
    clist->num_rows_exc_padding_partition[p] = 0;
    clist->num_rows_inc_padding_partition[p] = 0;
    clist->num_constraints_partition[p] = 0;
}

void MEAPI MdtBclEndConstraint(MdtKeaConstraints* const clist, const unsigned rows_added)
{
    int p = clist->num_partitions, i = clist->num_constraints;
    clist->Jsize[i] = rows_added;
    clist->Jofs[i] = clist->num_rows_exc_padding_partition[p];
    ++clist->num_constraints;
    ++clist->num_constraints_partition[p];
    clist->num_rows_exc_padding_partition[p] += rows_added;
    clist->num_rows_exc_padding += rows_added;
    addPadded(clist->num_rows_inc_padding_partition[p], rows_added);
    addPadded(clist->num_rows_inc_padding, rows_added);
}

// The unpadded total is padded to a whole block at the end of each partition, the padded counts drop the
// block left over from the last constraint.
void MEAPI MdtBclEndPartition(MdtKeaConstraints* const clist)
{
    int p = clist->num_partitions++;
    clist->num_rows_inc_padding_partition[p] &= ~3;
    clist->num_rows_inc_padding &= ~3;
    clist->num_rows_exc_padding = (clist->num_rows_exc_padding + 3) & ~3;
}

} // extern "C"
