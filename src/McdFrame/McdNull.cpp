#include "McdImpl.h"

// A geometry that occupies only its origin and collides with nothing unless the application registers an
// interaction for it.

extern "C" {

MeI16 MEAPI McdNullGetTypeId()
{
    return kMcdGeometryTypeNull;
}

void MEAPI McdNullRegisterType(McdFramework* frame)
{
    McdFrameworkRegisterGeometryType(frame, kMcdGeometryTypeNull, const_cast<char*>("Null"), McdNullDestroy,
                                     McdNullUpdateAABB, McdNullGetBSphere, McdNullMaximumPoint,
                                     McdNullGetMassProperties, nullptr);
}

McdNullID MEAPI McdNullCreate(McdFramework* frame)
{
    auto* n = new McdNull{};
    McdGeometryInit(&n->m_g, frame, kMcdGeometryTypeNull);
    return &n->m_g;
}

void MEAPI McdNullDestroy(McdGeometryID g)
{
    McdGeometryDeinit(g);
    delete reinterpret_cast<McdNull*>(g);
}

void MEAPI McdNullUpdateAABB(McdGeometryInstanceID ins, MeMatrix4 finalTM, MeBool)
{
    for (int k = 0; k < 3; ++k)
        ins->min[k] = ins->max[k] = finalTM[3][k];
}

void MEAPI McdNullGetBSphere(McdGeometryID, MeVector3 center, MeReal* radius)
{
    center[0] = center[1] = center[2] = 0;
    *radius = 0;
}

void MEAPI McdNullMaximumPoint(McdGeometryInstanceID ins, MeReal* const, MeReal* const outPoint)
{
    for (int k = 0; k < 3; ++k)
        outPoint[k] = ins->mTM[3][k];
}

MeI16 MEAPI McdNullGetMassProperties(McdGeometryID, MeMatrix4 relTM, MeMatrix3 inertia, MeReal* volume)
{
    mcdSetDiagonalInertia(relTM, inertia, 0, 0, 0);
    *volume = 0;
    return 0;
}

char* MEAPI McdNullGetTypeName(McdGeometryID)
{
    return const_cast<char*>("Null");
}

void MEAPI McdNullDebugDraw(const McdGeometryID, const MeMatrix4, const MeReal[3])
{
}

} // extern "C"
