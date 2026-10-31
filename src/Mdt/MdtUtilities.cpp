#include <cstring>

#include "MdtImpl.h"

namespace {

void diagonal(MeMatrix3 i, MeReal x, MeReal y, MeReal z)
{
    std::memset(i, 0, sizeof(MeMatrix3));
    i[0][0] = x;
    i[1][1] = y;
    i[2][2] = z;
}

} // namespace

extern "C" {

void MEAPI MdtConvertVector(const MdtBodyID from_body, const MeVector3 f, const MdtBodyID to_body, MeVector3 t)
{
    btVector3 world = mdtBodyTransform(from_body).getBasis() * toBt(f);
    fromBt(mdtBodyTransform(to_body).getBasis().transpose() * world, t);
}

void MEAPI MdtConvertPositionVector(const MdtBodyID from_body, const MeVector3 f, const MdtBodyID to_body,
                                    MeVector3 t)
{
    fromBt(mdtBodyTransform(to_body).invXform(mdtBodyTransform(from_body) * toBt(f)), t);
}

void MEAPI MdtMakeInertiaTensorSphere(const MeReal mass, const MeReal radius, MeMatrix3 i)
{
    MeReal d = (MeReal)0.4 * mass * radius * radius;
    diagonal(i, d, d, d);
}

void MEAPI MdtMakeInertiaTensorBox(const MeReal mass, const MeReal lx, const MeReal ly, const MeReal lz, MeMatrix3 i)
{
    MeReal k = mass / 12;
    diagonal(i, k * (ly * ly + lz * lz), k * (lx * lx + lz * lz), k * (lx * lx + ly * ly));
}

} // extern "C"
