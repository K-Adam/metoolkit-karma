#include <cmath>
#include <cstdio>

#include <MeMath.h>

namespace {

bool near(MeReal a, MeReal b, MeReal tolerance)
{
    return std::fabs(a - b) <= tolerance;
}

bool isIdentity(const MeReal* a, int n, MeReal tolerance)
{
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (!near(a[i * n + j], i == j ? 1 : 0, tolerance))
                return false;
    return true;
}

// The rotation by w over h as a quaternion, world frame like MeQuaternionWtoDQ.
void deltaRotation(MeVector4 dq, const MeVector3 w, MeReal h)
{
    MeReal speed = MeVector3Magnitude(w);
    MeReal halfAngle = speed * h / 2;
    // sin(speed h / 2) / speed tends to h / 2 as speed vanishes.
    MeReal s = speed > ME_MIN_EPSILON ? std::sin(halfAngle) / speed : h / 2;
    MeQuaternionSet(dq, std::cos(halfAngle), w[0] * s, w[1] * s, w[2] * s);
}

void normalize4(MeVector4 out, const MeVector4 q)
{
    MeReal n = std::sqrt(MeVector4MagnitudeSqr(q));
    for (int i = 0; i < 4; ++i)
        out[i] = q[i] / n;
}

void updateTM(MeMatrix4 out, MeReal epsilon, MeReal h, const MeVector3 dx, const MeVector3 w, const MeMatrix4 tm)
{
    MeMatrix3 R, rotated;
    MeMatrix4TMGetRotation(R, tm);
    if (MeVector3Magnitude(w) > epsilon)
    {
        // Row vectors are transformed, v' = v R, so the world rotation goes on the right.
        MeVector4 dq;
        MeMatrix3 dR;
        deltaRotation(dq, w, h);
        MeQuaternionToR(dR, dq);
        MeMatrix3MultiplyMatrix(rotated, R, dR);
    }
    else
        MeMatrix3Copy(rotated, R);
    MeReal x = tm[3][0] + dx[0], y = tm[3][1] + dx[1], z = tm[3][2] + dx[2];
    MeMatrix4TMMakeFromRotationAndPosition(out, rotated, x, y, z);
}

} // namespace

extern "C" {

MeReal MEAPI MeVector3AreaOfTriangle(const MeVector3 v1, const MeVector3 v2)
{
    MeVector3 c;
    MeVector3Cross(c, v1, v2);
    return MeVector3Magnitude(c) / 2;
}

void MEAPI MeVector3Swap(MeVector3 a, MeVector3 b)
{
    for (int i = 0; i < 3; ++i)
        MeRealSwap(&a[i], &b[i]);
}

void MEAPI MeQuaternionFromTM(MeVector4 q, const MeMatrix4 tm)
{
    // Shepperd's method: divide by the largest of the four candidates for stability.
    MeReal trace = tm[0][0] + tm[1][1] + tm[2][2];
    if (trace > tm[0][0] && trace > tm[1][1] && trace > tm[2][2])
    {
        MeReal s = std::sqrt(1 + trace) * 2;
        q[0] = s / 4;
        q[1] = (tm[1][2] - tm[2][1]) / s;
        q[2] = (tm[2][0] - tm[0][2]) / s;
        q[3] = (tm[0][1] - tm[1][0]) / s;
    }
    else if (tm[0][0] >= tm[1][1] && tm[0][0] >= tm[2][2])
    {
        MeReal s = std::sqrt(1 + tm[0][0] - tm[1][1] - tm[2][2]) * 2;
        q[0] = (tm[1][2] - tm[2][1]) / s;
        q[1] = s / 4;
        q[2] = (tm[0][1] + tm[1][0]) / s;
        q[3] = (tm[2][0] + tm[0][2]) / s;
    }
    else if (tm[1][1] >= tm[2][2])
    {
        MeReal s = std::sqrt(1 + tm[1][1] - tm[0][0] - tm[2][2]) * 2;
        q[0] = (tm[2][0] - tm[0][2]) / s;
        q[1] = (tm[0][1] + tm[1][0]) / s;
        q[2] = s / 4;
        q[3] = (tm[1][2] + tm[2][1]) / s;
    }
    else
    {
        MeReal s = std::sqrt(1 + tm[2][2] - tm[0][0] - tm[1][1]) * 2;
        q[0] = (tm[0][1] - tm[1][0]) / s;
        q[1] = (tm[2][0] + tm[0][2]) / s;
        q[2] = (tm[1][2] + tm[2][1]) / s;
        q[3] = s / 4;
    }
}

void MEAPI MeQuaternionFiniteRotation(MeVector4 q, const MeVector3 w, const MeReal h)
{
    MeVector4 dq, r;
    deltaRotation(dq, w, h);
    MeQuaternionProduct(r, dq, q);
    normalize4(q, r);
}

void MEAPI MeQuaternionForRotation(MeVector4 q, const MeVector3 v1, const MeVector3 v2)
{
    MeVector3 a, b, axis;
    MeVector3Copy(a, v1);
    MeVector3Copy(b, v2);
    MeVector3Normalize(a);
    MeVector3Normalize(b);
    MeReal d = MeVector3Dot(a, b);
    if (d < ME_MEDIUM_EPSILON - 1)
    {
        MeVector3 unused;
        MeVector3PlaneSpace(a, axis, unused);
        MeQuaternionSet(q, 0, axis[0], axis[1], axis[2]);
        return;
    }
    // (1 + cos, sin axis) normalised is the half angle quaternion.
    MeVector3Cross(axis, a, b);
    MeQuaternionSet(q, 1 + d, axis[0], axis[1], axis[2]);
    normalize4(q, q);
}

void MEAPI MeQuaternionSlerp(MeVector4 q, const MeVector4 from, const MeVector4 to, const MeReal howFar)
{
    MeReal cosom = MeVector4Dot(from, to);
    MeReal sign = 1;
    // q and -q are the same rotation, so take the shorter arc.
    if (cosom < 0)
    {
        cosom = -cosom;
        sign = -1;
    }
    MeReal k0 = 1 - howFar, k1 = howFar;
    if (cosom < 1 - ME_MEDIUM_EPSILON)
    {
        MeReal omega = std::acos(cosom);
        MeReal sinom = std::sin(omega);
        k0 = std::sin(k0 * omega) / sinom;
        k1 = std::sin(k1 * omega) / sinom;
    }
    MeVector4 r;
    for (int i = 0; i < 4; ++i)
        r[i] = k0 * from[i] + sign * k1 * to[i];
    normalize4(q, r);
}

void MEAPI MeMatrixFPrint(FILE* const file, const MeReal* const A, const int n, const int m, const char* const format)
{
    const char* f = format ? format : "%10.5f ";
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < m; ++j)
            std::fprintf(file, f, static_cast<double>(A[i * m + j]));
        std::fputc('\n', file);
    }
}

void MEAPI MeMatrixPrint(const MeReal* const A, const int n, const int m, const char* const format)
{
    MeMatrixFPrint(stdout, A, n, m, format);
}

// The header declares this only when inlining is off, so it needs its own prototype.
MeReal MEAPI MeMatrix3Trace(MeMatrix3 m)
{
    return m[0][0] + m[1][1] + m[2][2];
}

MeBool MEAPI MeMatrix3IsIdentity(const MeMatrix3 a, const MeReal tolerance)
{
    return isIdentity(a[0], 3, tolerance);
}

void MEAPI MeMatrix3LUDecompose(MeMatrix3 L, MeMatrix3 U, const MeMatrix3 a, unsigned int* const SwappedRow,
                                MeVector3 scale)
{
    // Doolittle with implicitly scaled partial pivoting: a[SwappedRow[i]] is row i of L U.
    MeMatrix3 m;
    MeMatrix3Copy(m, a);
    for (int i = 0; i < 3; ++i)
    {
        MeReal big = std::fmax(std::fabs(m[i][0]), std::fmax(std::fabs(m[i][1]), std::fabs(m[i][2])));
        scale[i] = big > 0 ? 1 / big : 0;
        SwappedRow[i] = i;
    }
    MeMatrix3MakeIdentity(L);
    for (int k = 0; k < 3; ++k)
    {
        int pivot = k;
        for (int i = k + 1; i < 3; ++i)
            if (std::fabs(m[i][k]) * scale[SwappedRow[i]] > std::fabs(m[pivot][k]) * scale[SwappedRow[pivot]])
                pivot = i;
        if (pivot != k)
        {
            MeVector3Swap(m[k], m[pivot]);
            for (int j = 0; j < k; ++j)
                MeRealSwap(&L[k][j], &L[pivot][j]);
            unsigned int t = SwappedRow[k];
            SwappedRow[k] = SwappedRow[pivot];
            SwappedRow[pivot] = t;
        }
        for (int i = k + 1; i < 3; ++i)
        {
            MeReal f = m[k][k] != 0 ? m[i][k] / m[k][k] : 0;
            L[i][k] = f;
            for (int j = k; j < 3; ++j)
                m[i][j] -= f * m[k][j];
        }
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            U[i][j] = j >= i ? m[i][j] : 0;
}

void MEAPI MeMatrix3SwapColumns(MeMatrix3 a, const unsigned int col1, const unsigned int col2)
{
    for (int i = 0; i < 3; ++i)
        MeRealSwap(&a[i][col1], &a[i][col2]);
}

MeBool MEAPI MeMatrix3Invert(MeMatrix3 a)
{
    MeMatrix3 adj;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            int r0 = (j + 1) % 3, r1 = (j + 2) % 3, c0 = (i + 1) % 3, c1 = (i + 2) % 3;
            adj[i][j] = a[r0][c0] * a[r1][c1] - a[r0][c1] * a[r1][c0];
        }
    MeReal det = a[0][0] * adj[0][0] + a[0][1] * adj[1][0] + a[0][2] * adj[2][0];
    // Relative to the row magnitudes so that small but well conditioned matrices still invert.
    MeReal size = MeVector3Magnitude(a[0]) * MeVector3Magnitude(a[1]) * MeVector3Magnitude(a[2]);
    if (!(std::fabs(det) > ME_MIN_EPSILON * size))
        return MEFALSE;
    MeMatrix3Scale(adj, 1 / det);
    MeMatrix3Copy(a, adj);
    return 1;
}

MeBool MEAPI MeMatrix3SymmetricInvert(MeMatrix3 a)
{
    return MeMatrix3Invert(a);
}

void MEAPI MeMatrix3MakeRotationX(MeMatrix3 m, const MeReal a)
{
    MeReal c = std::cos(a), s = std::sin(a);
    MeVector3Set(m[0], 1, 0, 0);
    MeVector3Set(m[1], 0, c, s);
    MeVector3Set(m[2], 0, -s, c);
}

void MEAPI MeMatrix3MakeRotationY(MeMatrix3 m, const MeReal a)
{
    MeReal c = std::cos(a), s = std::sin(a);
    MeVector3Set(m[0], c, 0, -s);
    MeVector3Set(m[1], 0, 1, 0);
    MeVector3Set(m[2], s, 0, c);
}

void MEAPI MeMatrix3MakeRotationZ(MeMatrix3 m, const MeReal a)
{
    MeReal c = std::cos(a), s = std::sin(a);
    MeVector3Set(m[0], c, s, 0);
    MeVector3Set(m[1], -s, c, 0);
    MeVector3Set(m[2], 0, 0, 1);
}

MeBool MEAPI MeMatrix3IsValidOrientationMatrix(const MeMatrix3 rot, const MeReal tolerance)
{
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j)
            if (!near(MeVector3Dot(rot[i], rot[j]), i == j ? 1 : 0, tolerance))
                return MEFALSE;
    MeVector3 c;
    MeVector3Cross(c, rot[0], rot[1]);
    return near(MeVector3Dot(c, rot[2]), 1, tolerance);
}

void MEAPI MeMatrix3FromEulerAngles(MeMatrix3 m, const MeReal xangle, const MeReal yangle, const MeReal zangle)
{
    // Rotate about x, then y, then z, all fixed axes.
    MeMatrix3 x, y, z, xy;
    MeMatrix3MakeRotationX(x, xangle);
    MeMatrix3MakeRotationY(y, yangle);
    MeMatrix3MakeRotationZ(z, zangle);
    MeMatrix3MultiplyMatrix(xy, x, y);
    MeMatrix3MultiplyMatrix(m, xy, z);
}

MeBool MEAPI MeMatrix4IsIdentity(const MeMatrix4 a, const MeReal tolerance)
{
    return isIdentity(a[0], 4, tolerance);
}

MeBool MEAPI MeMatrix4IsTM(const MeMatrix4 tm, const MeReal tolerance)
{
    MeMatrix3 R;
    MeMatrix4TMGetRotation(R, tm);
    return near(tm[0][3], 0, tolerance) && near(tm[1][3], 0, tolerance) && near(tm[2][3], 0, tolerance) &&
           near(tm[3][3], 1, tolerance) && MeMatrix3IsValidOrientationMatrix(R, tolerance);
}

void MEAPI MeMatrix4TMCompound(MeMatrix4 A, const MeMatrix4 B, const MeMatrix4 C)
{
    // A may alias B or C.
    MeMatrix4 r;
    MeMatrix4MultiplyMatrix(r, B, C);
    MeMatrix4Copy(A, r);
}

void MEAPI MeMatrix4TMInvert(MeMatrix4 tm)
{
    MeMatrix3 R;
    MeMatrix4TMGetRotation(R, tm);
    MeMatrix3Transpose(R);
    MeVector3 t = {tm[3][0], tm[3][1], tm[3][2]}, p;
    MeMatrix3MultiplyVector(p, R, t);
    MeMatrix4TMMakeFromRotationAndPosition(tm, R, -p[0], -p[1], -p[2]);
}

void MEAPI MeMatrix4TMUpdateFromVelocities(MeMatrix4 aTransformReturn, MeReal aEpsilon, MeReal aTimeStep,
                                           const MeVector3 aVelocity, const MeVector3 aAngularVelocity,
                                           const MeMatrix4 aTransform)
{
    MeVector3 dx;
    MeVector3MultiplyScalar(dx, aVelocity, aTimeStep);
    updateTM(aTransformReturn, aEpsilon, aTimeStep, dx, aAngularVelocity, aTransform);
}

void MEAPI MeMatrix4TMUpdateFromVelocitiesAndAcceler(MeMatrix4 aTransformReturn, MeReal aEpsilon, MeReal aTimeStep,
                                                     const MeVector3 aVelocity, const MeVector3 aAcceler,
                                                     const MeVector3 aAngularVelocity,
                                                     const MeVector3 aAngularAcceler, const MeMatrix4 aTransform)
{
    // Constant acceleration over the step: the mean velocity is v + a h / 2.
    MeReal h = aTimeStep;
    MeVector3 v, w;
    MeVector3ScaleAndAdd(v, aVelocity, h / 2, aAcceler);
    MeVector3ScaleAndAdd(w, aAngularVelocity, h / 2, aAngularAcceler);
    MeVector3Scale(v, h);
    updateTM(aTransformReturn, aEpsilon, h, v, w, aTransform);
}

void MEAPI MeMatrix4TMOrthoNormalize(MeMatrix4 tm)
{
    MeVector3Normalize(tm[0]);
    MeVector3MultiplySubtract(tm[1], MeVector3Dot(tm[0], tm[1]), tm[0]);
    MeVector3Normalize(tm[1]);
    MeVector3Cross(tm[2], tm[0], tm[1]);
}

void MEAPI MeMatrix4TMFromEulerAnglesAndPosition(MeMatrix4 tm, const MeReal xangle, const MeReal yangle,
                                                 const MeReal zangle, const MeReal x, const MeReal y, const MeReal z)
{
    MeMatrix3 R;
    MeMatrix3FromEulerAngles(R, xangle, yangle, zangle);
    MeMatrix4TMMakeFromRotationAndPosition(tm, R, x, y, z);
}

} // extern "C"
