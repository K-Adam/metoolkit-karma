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

} // extern "C"
