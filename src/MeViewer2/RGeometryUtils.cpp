// Rendering is outside this library. These helpers exist so demo code links.

#include <RGeometryUtils.h>

extern "C" {

void MEAPI RConvertTriStripToTriList(RGraphic* rg, RObjectVertex* strips, int* stripSize, int* stripStart,
                                     int numStrips)
{
}

void MEAPI RSetVertex(RObjectVertex* vertex, MeVector3 vert, MeVector3 norm, MeReal u, MeReal v)
{
    vertex->m_X = vert[0];
    vertex->m_Y = vert[1];
    vertex->m_Z = vert[2];
    vertex->m_NX = norm[0];
    vertex->m_NY = norm[1];
    vertex->m_NZ = norm[2];
    vertex->m_U = u;
    vertex->m_V = v;
}

} // extern "C"
