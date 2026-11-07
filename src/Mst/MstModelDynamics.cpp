#include <cstring>

#include <Mst.h>

#include "McdImpl.h"
#include "MdtImpl.h"

namespace {

// Model and body point at each other, so whichever is destroyed first unbinds the other.
void unlinkBody(McdModelID m)
{
    auto* b = static_cast<MdtBodyID>(m->mBody);
    if (b && b->model == m)
        b->model = nullptr;
}

// The model stays where the body last put it.
void unlinkModel(MdtBodyID b)
{
    auto* m = static_cast<McdModelID>(b->model);
    if (!m || m->mBody != b)
        return;
    std::memcpy(impl(m)->ownTM, b->bodyTM, sizeof(MeMatrix4));
    McdModelSetBody(m, nullptr);
}

} // namespace

extern "C" {

MdtBodyID MEAPI McdModelGetBody(const McdModelID m)
{
    return static_cast<MdtBodyID>(m->mBody);
}

// A model with a body collides at the body's transform and reports the body's velocities.
void MEAPI McdModelSetBody(const McdModelID m, const MdtBodyID b)
{
    unlinkBody(m);
    impl(m->frame)->modelDestroyHook = unlinkBody;
    m->mBody = b;
    if (b)
        impl(b->world)->bodyDestroyHook = unlinkModel;
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
