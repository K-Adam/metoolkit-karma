#include <MeMath.h>

#include "check.h"

extern "C" MeReal MEAPI MeMatrix3Trace(MeMatrix3 m);

static const MeReal tol = 1e-5f;

static void checkMatrix3(const MeMatrix3 a, const MeMatrix3 b)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(a[i][j], b[i][j], tol);
}

static void checkSameRotation(const MeVector4 a, const MeVector4 b)
{
    CHECK_NEAR(fabs(MeVector4Dot(a, b)), 1, tol);
}

int main()
{
    const MeReal half = ME_PI / 2;

    // Rotations match the quaternion convention: v' = v R rotates x into y about z.
    MeMatrix3 R, Rq, I;
    MeVector4 q;
    MeVector3 z = {0, 0, 1}, x = {1, 0, 0}, y = {0, 1, 0}, v;
    MeMatrix3MakeRotationZ(R, half);
    MeQuaternionMake(q, z, half);
    MeQuaternionToR(Rq, q);
    checkMatrix3(R, Rq);
    MeMatrix3MultiplyVector(v, R, x);
    CHECK_NEAR(v[0], 0, tol);
    CHECK_NEAR(v[1], 1, tol);
    CHECK(MeMatrix3IsValidOrientationMatrix(R, tol));
    MeMatrix3MakeIdentity(I);
    CHECK(MeMatrix3IsIdentity(I, 0));
    CHECK(!MeMatrix3IsIdentity(R, tol));
    CHECK_NEAR(MeMatrix3Trace(I), 3, 0);

    // X and Y rotations agree with their quaternions too.
    for (int axis = 0; axis < 2; ++axis)
    {
        MeVector3 a = {MeReal(axis == 0), MeReal(axis == 1), 0};
        axis == 0 ? MeMatrix3MakeRotationX(R, 0.7f) : MeMatrix3MakeRotationY(R, 0.7f);
        MeQuaternionMake(q, a, 0.7f);
        MeQuaternionToR(Rq, q);
        checkMatrix3(R, Rq);
    }

    // Euler angles apply x, then y, then z.
    MeMatrix3 E, Rx, Ry, Rz, T;
    MeMatrix3FromEulerAngles(E, 0.3f, -0.4f, 1.1f);
    MeMatrix3MakeRotationX(Rx, 0.3f);
    MeMatrix3MakeRotationY(Ry, -0.4f);
    MeMatrix3MakeRotationZ(Rz, 1.1f);
    MeMatrix3MultiplyMatrix(T, Rx, Ry);
    MeMatrix3MultiplyMatrix(R, T, Rz);
    checkMatrix3(E, R);

    // Quaternion round trip through a TM.
    MeMatrix4 tm, tm2, P;
    MeMatrix4TMFromEulerAnglesAndPosition(tm, 2.5f, 0.2f, -2.9f, 1, 2, 3);
    CHECK(MeMatrix4IsTM(tm, tol));
    MeVector4 q2;
    MeQuaternionFromTM(q2, tm);
    MeMatrix3 back;
    MeQuaternionToR(back, q2);
    MeMatrix4TMGetRotation(R, tm);
    checkMatrix3(back, R);

    // TM inverse composes to identity, and compound works in place.
    MeMatrix4Copy(tm2, tm);
    MeMatrix4TMInvert(tm2);
    MeMatrix4TMCompound(P, tm, tm2);
    CHECK(MeMatrix4IsIdentity(P, tol));
    MeMatrix4TMCompound(tm2, tm, tm2);
    CHECK(MeMatrix4IsIdentity(tm2, tol));

    // General inverse, singular detection and tiny but regular matrices.
    MeMatrix3 A = {{2, 1, 0}, {1, 3, 1}, {0, 1, 4}}, Ainv;
    MeMatrix3Copy(Ainv, A);
    CHECK(MeMatrix3Invert(Ainv));
    MeMatrix3MultiplyMatrix(T, A, Ainv);
    CHECK(MeMatrix3IsIdentity(T, tol));
    MeMatrix3Copy(Ainv, A);
    CHECK(MeMatrix3SymmetricInvert(Ainv));
    MeMatrix3MultiplyMatrix(T, Ainv, A);
    CHECK(MeMatrix3IsIdentity(T, tol));
    MeMatrix3 S = {{1, 2, 3}, {2, 4, 6}, {1, 0, 1}};
    CHECK(!MeMatrix3Invert(S));
    CHECK_NEAR(S[0][1], 2, 0);
    MeMatrix3 small = {{1e-5f, 0, 0}, {0, 1e-5f, 0}, {0, 0, 1e-5f}};
    CHECK(MeMatrix3Invert(small));
    CHECK_NEAR(small[1][1], 1e5f, 1);

    // LU: rows of a permuted by SwappedRow equal L U, L unit lower, U upper.
    MeMatrix3 B = {{1, 2, 3}, {4, 5, 6}, {7, 8, 10}}, L, U, LU;
    unsigned int swapped[3];
    MeVector3 scale;
    MeMatrix3LUDecompose(L, U, B, swapped, scale);
    MeMatrix3MultiplyMatrix(LU, L, U);
    for (int i = 0; i < 3; ++i)
    {
        CHECK_NEAR(L[i][i], 1, 0);
        for (int j = 0; j < 3; ++j)
        {
            CHECK_NEAR(LU[i][j], B[swapped[i]][j], tol);
            if (j > i)
                CHECK_NEAR(L[i][j], 0, 0);
            if (j < i)
                CHECK_NEAR(U[i][j], 0, 0);
        }
    }
    CHECK_NEAR(scale[2], 0.1f, tol);

    MeMatrix3SwapColumns(B, 0, 2);
    CHECK_NEAR(B[2][0], 10, 0);
    CHECK_NEAR(B[2][2], 7, 0);

    // Shortest arc rotation, including the antiparallel case.
    MeVector3 from = {1, 1, 0}, to = {0, 0, 3}, out;
    MeQuaternionForRotation(q, from, to);
    MeQuaternionRotateVector3(out, q, from);
    CHECK_NEAR(out[2], sqrt(2.0), tol);
    MeVector3 back3 = {-1, -1, 0};
    MeQuaternionForRotation(q, from, back3);
    MeQuaternionRotateVector3(out, q, from);
    CHECK_NEAR(out[0], -1, tol);
    CHECK_NEAR(out[1], -1, tol);

    // Slerp halfway between identity and 90 degrees about z is 45 degrees, also from the negated end.
    MeVector4 id = {1, 0, 0, 0}, q90, q45, neg, s;
    MeQuaternionMake(q90, z, half);
    MeQuaternionMake(q45, z, half / 2);
    MeQuaternionSlerp(s, id, q90, 0.5f);
    checkSameRotation(s, q45);
    for (int i = 0; i < 4; ++i)
        neg[i] = -q90[i];
    MeQuaternionSlerp(s, id, neg, 0.5f);
    checkSameRotation(s, q45);

    // Finite rotation integrates a world angular velocity, so it agrees with the TM update.
    MeVector3 w = {0.3f, -1.2f, 2.0f}, vel = {1, 0, -1}, zero = {0, 0, 0};
    MeVector4 qi;
    MeQuaternionFromTM(qi, tm);
    MeQuaternionFiniteRotation(qi, w, 0.25f);
    MeMatrix4TMUpdateFromVelocities(tm2, 0, 0.25f, vel, w, tm);
    MeQuaternionToR(back, qi);
    MeMatrix4TMGetRotation(R, tm2);
    checkMatrix3(back, R);
    CHECK_NEAR(tm2[3][0], 1.25f, tol);
    CHECK_NEAR(tm2[3][2], 2.75f, tol);
    MeQuaternionFiniteRotation(q, zero, 1);

    // With constant acceleration the position follows v h + a h^2 / 2.
    MeVector3 acc = {0, 0, -10};
    MeMatrix4TMUpdateFromVelocitiesAndAcceler(tm2, 0, 0.5f, vel, acc, zero, zero, tm);
    CHECK_NEAR(tm2[3][2], 3 - 0.5f - 1.25f, tol);
    MeMatrix4TMGetRotation(back, tm2);
    MeMatrix4TMGetRotation(R, tm);
    checkMatrix3(back, R);

    // Orthonormalisation repairs a skewed TM and keeps its position.
    MeMatrix4Copy(tm2, tm);
    tm2[0][0] += 0.05f;
    tm2[1][2] -= 0.03f;
    CHECK(!MeMatrix4IsTM(tm2, 1e-3f));
    MeMatrix4TMOrthoNormalize(tm2);
    CHECK(MeMatrix4IsTM(tm2, tol));
    CHECK_NEAR(tm2[3][1], 2, 0);

    MeVector3 a = {1, 2, 3}, b = {4, 5, 6};
    MeVector3Swap(a, b);
    CHECK_NEAR(a[0], 4, 0);
    CHECK_NEAR(b[2], 3, 0);
    CHECK_NEAR(MeVector3AreaOfTriangle(x, y), 0.5f, tol);
    return 0;
}
