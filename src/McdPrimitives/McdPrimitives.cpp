#include "Primitive.h"

void mcdRegisterBoxPlane(McdFramework* frame);
void mcdRegisterSpherePlane(McdFramework* frame);
void mcdRegisterCylinderPlane(McdFramework* frame);
void mcdRegisterSphylPlane(McdFramework* frame);

extern "C" {

unsigned int MEAPI McdPrimitivesGetTypeCount()
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
    McdCylinderRegisterType(frame);
    McdSphylRegisterType(frame);
    McdTriangleListRegisterType(frame);
}

// Pairs without an entry here use Bullet's narrowphase.
void MEAPI McdSphereBoxPlaneRegisterInteractions(McdFramework* frame)
{
    mcdRegisterBoxPlane(frame);
    mcdRegisterSpherePlane(frame);
}

MeBool MEAPI McdPrimitivesRegisterInteractions(McdFramework* frame)
{
    McdSphereBoxPlaneRegisterInteractions(frame);
    mcdRegisterCylinderPlane(frame);
    mcdRegisterSphylPlane(frame);
    for (int type : {kMcdGeometryTypeSphere, kMcdGeometryTypeBox, kMcdGeometryTypeCylinder, kMcdGeometryTypeSphyl})
        mcdRegisterTriangleListInteraction(frame, type);
    return 1;
}

} // extern "C"
