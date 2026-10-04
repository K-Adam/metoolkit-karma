// Rendering is outside this library. MeApp exists so demo code links, and does nothing.

#include <MeApp.h>

extern "C" {

MeApp* MEAPI MeAppCreateFromUniverse(const MstUniverseID u, RRender *rc)
{
    return {};
}

MstUniverseID MEAPI MeAppGetMstUniverse(MeApp *app)
{
    return {};
}

MeApp* MEAPI MeAppCreate(const MdtWorldID world, const McdSpaceID space, RRender *rc)
{
    return {};
}

void MEAPI MeAppDestroy(MeApp* const app)
{
}

void MEAPI MeAppDrawContactsInit(MeApp* const app, float color[4], int max)
{
}

void MEAPI MeAppDrawContacts(const MeApp *app)
{
}

void MEAPI MeAppToggleDrawContacts(MeApp *app, MeBool d)
{
}

void MEAPI MeAppSetContactDrawLength(MeApp *app, MeReal length)
{
}

void MEAPI MeAppFindClickDir(const MeApp *const app, int x, int y, MeVector3 normClickDir)
{
    normClickDir[0] = normClickDir[1] = normClickDir[2] = 0;
}

void MEAPI MeAppFindClickPos(const MeApp *const app, int x, int y, MeVector3 clickPos)
{
    clickPos[0] = clickPos[1] = clickPos[2] = 0;
}

McdModelID MEAPI MeAppPickMcdModel(const MeApp *const app, int x, int y, MeVector3 normClickDir, MeVector3 pos,
                                   MeBool orth)
{
    return {};
}

void MEAPI MeAppUpdateMouseSpring(MeApp *app)
{
}

void MEAPI MeAppMousePickCB(RRender *rc, int x, int y, int modifiers, RMouseButtonWhich which, RMouseButtonEvent event,
                            void *userdata)
{
}

void MEAPI MeAppStep(MeApp *app)
{
}

} // extern "C"
