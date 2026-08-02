#pragma once

#include <LinearMath/btTransform.h>

#include <MePrecision.h>

static_assert(sizeof(MeReal) == sizeof(btScalar), "MeReal and Bullet must use the same precision");

inline btVector3 toBt(const MeReal* v) { return btVector3(v[0], v[1], v[2]); }

inline void fromBt(const btVector3& v, MeReal* out)
{
    out[0] = v.x();
    out[1] = v.y();
    out[2] = v.z();
}

// MeMatrix4 stores row vectors with the translation in row 3, the same memory order as OpenGL.
inline btTransform toBtTransform(const MeReal* tm)
{
    btTransform t;
    t.setFromOpenGLMatrix(tm);
    return t;
}

inline void fromBtTransform(const btTransform& t, MeReal* tm) { t.getOpenGLMatrix(tm); }

inline btTransform toBtTransform(const MeVector4* tm) { return toBtTransform(&tm[0][0]); }

inline void fromBtTransform(const btTransform& t, MeVector4* tm) { t.getOpenGLMatrix(&tm[0][0]); }

// Karma quaternions are (w, x, y, z), Bullet's are (x, y, z, w).
inline btQuaternion toBtQuat(MeReal w, MeReal x, MeReal y, MeReal z) { return btQuaternion(x, y, z, w); }

inline void fromBtQuat(const btQuaternion& q, MeReal* out)
{
    out[0] = q.w();
    out[1] = q.x();
    out[2] = q.y();
    out[3] = q.z();
}
