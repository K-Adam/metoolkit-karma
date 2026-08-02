/* Compiles the public headers as C and checks that the C++ implementation links with C linkage. */
#include <Mst.h>

#include "check.h"

int main(void)
{
    MstUniverseID universe = MstUniverseCreate(&MstUniverseDefaultSizes);
    McdModelID model = MstModelAndBodyCreate(universe, McdBoxCreate(MstUniverseGetFramework(universe), 1, 2, 3), 1);
    MeReal dx, dy, dz;

    CHECK(model != NULL);
    McdBoxGetDimensions(McdModelGetGeometry(model), &dx, &dy, &dz);
    CHECK(dx == 1 && dy == 2 && dz == 3);
    MstUniverseStep(universe, (MeReal)0.01);
    MstUniverseDestroy(universe);
    return 0;
}
