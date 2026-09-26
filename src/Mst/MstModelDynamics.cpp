#include <Mst.h>

extern "C" {

MdtBodyID MEAPI McdModelGetBody(const McdModelID m)
{
    return static_cast<MdtBodyID>(m->mBody);
}

// A model with a body collides at the body's transform and reports the body's velocities.
void MEAPI McdModelSetBody(const McdModelID m, const MdtBodyID b)
{
    m->mBody = b;
    if (!b)
    {
        McdModelSetTransformPtr(m, nullptr);
        McdModelSetLinearVelocityPtr(m, nullptr);
        McdModelSetAngularVelocityPtr(m, nullptr);
        return;
    }
    b->model = m;
    McdModelSetTransformPtr(m, MdtBodyGetTransformPtr(b));
    McdModelSetLinearVelocityPtr(m, MdtBodyGetLinearVelocityPtr(b));
    McdModelSetAngularVelocityPtr(m, MdtBodyGetAngularVelocityPtr(b));
}

// Only the pose and velocities are reset, the mass properties stay.
void MEAPI McdModelDynamicsReset(const McdModelID m)
{
    MdtBodyID b = McdModelGetBody(m);
    if (!b)
        return;
    const MeMatrix4 identity = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    MdtBodySetTransform(b, identity);
    MdtBodySetLinearVelocity(b, 0, 0, 0);
    MdtBodySetAngularVelocity(b, 0, 0, 0);
}

} // extern "C"
