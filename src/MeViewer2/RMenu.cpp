// Rendering is outside this library. The viewer API exists so demo code links, and does nothing.

#include <RMenu.h>

extern "C" {

void MEAPI RRenderDisplayDefaultMenu(RRender *rc)
{
}

RMenu* MEAPI RRenderGetCurrentMenu(RRender *rc)
{
    return {};
}

void MEAPI RRenderHideCurrentMenu(RRender *rc)
{
}

void MEAPI RMenuNextEntry(RRender *rc)
{
}

void MEAPI RMenuPreviousEntry(RRender *rc)
{
}

void MEAPI RMenuExecute1Entry(RRender *rc)
{
}

void MEAPI RMenuExecute2Entry(RRender *rc)
{
}

} // extern "C"
