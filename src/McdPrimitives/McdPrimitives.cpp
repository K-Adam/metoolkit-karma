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

// Callers such as UT2004 dispatch on the interaction table themselves, so every supported pair needs an entry.
void MEAPI McdSphereBoxPlaneRegisterInteractions(McdFramework* frame)
{
    for (int a : {kMcdGeometryTypeSphere, kMcdGeometryTypeBox})
        for (int b : {kMcdGeometryTypeSphere, kMcdGeometryTypeBox})
            mcdSetInteractionsBoth(frame, a, b, mcdBulletInteraction());
    mcdRegisterBoxPlane(frame);
    mcdRegisterSpherePlane(frame);
}

MeBool MEAPI McdPrimitivesRegisterInteractions(McdFramework* frame)
{
    const int solids[] = {kMcdGeometryTypeSphere, kMcdGeometryTypeBox, kMcdGeometryTypeCylinder,
                          kMcdGeometryTypeSphyl};
    for (int a : solids)
        for (int b : solids)
            mcdSetInteractionsBoth(frame, a, b, mcdBulletInteraction());
    McdSphereBoxPlaneRegisterInteractions(frame);
    mcdRegisterCylinderPlane(frame);
    mcdRegisterSphylPlane(frame);
    for (int type : {kMcdGeometryTypeSphere, kMcdGeometryTypeBox, kMcdGeometryTypeCylinder, kMcdGeometryTypeSphyl})
        mcdRegisterTriangleListInteraction(frame, type);
    return 1;
}

} // extern "C"
