#include "McdImpl.h"

void mcdRegisterBoxPlane(McdFramework* frame);

extern "C" {

int MEAPI McdPrimitivesGetTypeCount()
{
    return kMcdGeometryBuiltInTypes;
}

void MEAPI McdSphereBoxPlaneRegisterTypes(McdFramework* frame)
{
    McdSphereRegisterType(frame);
    McdBoxRegisterType(frame);
    McdPlaneRegisterType(frame);
}

void MEAPI McdPrimitivesRegisterTypes(McdFramework* frame)
{
    McdSphereBoxPlaneRegisterTypes(frame);
}

// Pairs without an entry here use Bullet's narrowphase.
void MEAPI McdSphereBoxPlaneRegisterInteractions(McdFramework* frame)
{
    mcdRegisterBoxPlane(frame);
}

MeBool MEAPI McdPrimitivesRegisterInteractions(McdFramework* frame)
{
    McdSphereBoxPlaneRegisterInteractions(frame);
    return 1;
}

} // extern "C"
