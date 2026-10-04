// Rendering is outside this library. The viewer API exists so demo code links, and does nothing.

#include <MeViewer.h>

extern "C" {

void MEAPI RParseText(RRender *rc, char *text_in, char *text_out, int outbuffersize)
{
    if (text_out && outbuffersize > 0)
        text_out[0] = 0;
}

int MEAPI RBmpLoad(RRender* rc, char *filename, RImage *p_image, int bRequireBGR)
{
    return {};
}

RRender* MEAPI RRenderContextCreate2(MeCommandLineOptions* options, MeCommandLineOptions* overrideoptions, int eat,
                                     void *pWnd, RCreateCallback cb)
{
    return {};
}

int MEAPI RRenderContextDestroy(RRender *rc)
{
    return {};
}

void MEAPI RRenderQuit(RRender *rc)
{
}

void MEAPI RRenderSetPause(RRender *rc, int p)
{
}

void MEAPI RRenderUpdateProjectionMatrices(RRender *rc)
{
}

void MEAPI RRenderUpdateGraphicMatrices(RRender *rc)
{
}

int MEAPI RRenderTextureCreate(RRender *rc, const char *filename)
{
    return {};
}

void MEAPI RRenderToggleUserHelp(RRender *const rc)
{
}

void MEAPI RRenderCreateUserHelp(RRender *const rc, const char *const help[], const int arraySize)
{
}

void MEAPI RRenderSkydomeCreate(RRender *const rc, const char * name, int tileU, int tileV)
{
}

void MEAPI RRenderSetAppName(RRender* rc, const char* appname)
{
}

void MEAPI RRenderSetBackgroundColor(RRender* rc, const float color[4])
{
}

void MEAPI RRenderSetSphereResolution(RRender* r, int res)
{
}

int MEAPI RRenderGetSphereResolution(RRender* r)
{
    return {};
}

RGraphic* MEAPI RGraphicCreate(RRender *rc, char *filename, AcmeReal xScale, AcmeReal yScale, AcmeReal zScale,
                               const float color[4], MeMatrix4Ptr matrix, MeBool is2D, MeBool bKeepAspectRatio)
{
    return {};
}

RGraphic* MEAPI RGraphicCreateEmpty(int numVertices)
{
    return {};
}

RGraphic* MEAPI RGraphicLoadMEG(char *filename)
{
    return {};
}

RGraphic* MEAPI RGraphicLoadASE(RRender *rc, char* filename, AcmeReal xScale, AcmeReal yScale, AcmeReal zScale,
                                const float color[4], MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicLoadASEwithOffset(RRender *rc, char* filename, AcmeReal xScale, AcmeReal yScale,
                                          AcmeReal zScale, const float color[4], MeVector3Ptr offset,
                                          MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicLoadASENoMCD(RRender *rc, char* filename, AcmeReal xScale, AcmeReal yScale, AcmeReal zScale,
                                     const float color[4], MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicLoadASEwithOffsetNoMcd(RRender *rc, char* filename, AcmeReal xScale, AcmeReal yScale,
                                               AcmeReal zScale, const float color[4], MeVector3Ptr offset,
                                               MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicLoadASEwithOffsetNoMcdFromStream(RRender *rc, MeStream str, AcmeReal xScale, AcmeReal yScale,
                                                         AcmeReal zScale, const float color[4], MeVector3Ptr offset,
                                                         MeMatrix4Ptr matrix)
{
    return {};
}

int MEAPI RGraphicSave(RGraphic *rg, char *filename)
{
    return {};
}

void MEAPI RGraphicDestroy(RGraphic *rg)
{
}

void MEAPI RGraphicDelete(RRender *rc, RGraphic *rg, int is2D)
{
}

RGraphic* MEAPI RGraphicCombine(RRender* rc, RGraphic* a, RGraphic* b, MeMatrix4Ptr transform)
{
    return {};
}

void MEAPI RGraphicAddToList(RRender *rc, RGraphic *rg, int is2D)
{
}

void MEAPI RGraphicRemoveFromList(RRender *rc, RGraphic *rg, int is2D)
{
}

int MEAPI RGraphicSetTexture(RRender *rc, RGraphic *rg, const char *filename)
{
    return {};
}

void MEAPI RGraphicFillObjectBuffer(char *filename)
{
}

void MEAPI RGraphicNormalize(RGraphic *rg, int bKeepAspectRatio)
{
}

void MEAPI RGraphicScale(RGraphic *rg, AcmeReal xScale, AcmeReal yScale, AcmeReal zScale)
{
}

RGraphic* MEAPI RGraphic2DRectangleCreate(RRender *rc, AcmeReal orig_x, AcmeReal orig_y, AcmeReal width,
                                          AcmeReal height, const float color[4])
{
    return {};
}

RGraphic* MEAPI RGraphicBoxCreate(RRender *rc, AcmeReal width, AcmeReal height, AcmeReal depth, const float color[4],
                                  MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicConeCreate(RRender *rc, AcmeReal radius, AcmeReal upper_height, AcmeReal lower_height,
                                   const float color[4], MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicCylinderCreate(RRender *rc, AcmeReal radius, AcmeReal height, const float color[4],
                                       MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicGroundPlaneCreate(RRender *rc, AcmeReal side_length, int triangles_per_side,
                                          const float color[4], AcmeReal y_position)
{
    return {};
}

RGraphic* MEAPI RGraphicLineCreate(RRender *rc, AcmeReal origin[3], AcmeReal end[3], const float color[4],
                                   MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicSphereCreate(RRender *rc, AcmeReal radius, const float color[4], MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicSquareCreate(RRender *rc, AcmeReal side, const float color[4], MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicTextCreate(RRender *rc, char *text, AcmeReal orig_x, AcmeReal orig_y, const float color[4])
{
    return {};
}

RGraphic* MEAPI RGraphicTorusCreate(RRender *rc, AcmeReal innerRadius, AcmeReal outerRadius, const float color[4],
                                    MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicFrustumCreate(RRender *rc, AcmeReal bottomRadius, AcmeReal topRadius, AcmeReal bottom,
                                      AcmeReal top, int sides, const float color[4], MeMatrix4Ptr matrix)
{
    return {};
}

RGraphic* MEAPI RGraphicSphylCreate(RRender *rc, AcmeReal radius, AcmeReal height, const float color[4],
                                    MeMatrix4Ptr matrix)
{
    return {};
}

int MEAPI RGraphicLineMoveEnds(RGraphic *lineG, AcmeReal *origin, AcmeReal *end)
{
    return {};
}

void MEAPI RGraphicSetTransformPtr(RGraphic *g, MeMatrix4Ptr matrix)
{
}

void MEAPI RGraphicSetColor(RGraphic *g, const float color[4])
{
}

void MEAPI RGraphicGetColor(RGraphic *g, float color[4])
{
    color[0] = color[1] = color[2] = color[3] = 0;
}

void MEAPI RGraphicSetWireframe(RGraphic *g, MeBool b)
{
}

MeBool MEAPI RGraphicIsWireframe(RGraphic *g)
{
    return {};
}

void MEAPI RGraphicCalcOGLDisplayList(RGraphic *rg)
{
}

void MEAPI RGraphicTranslateVertices(RRender *rc, RGraphic *rg, MeVector3Ptr offset)
{
}

void MEAPI RLineAdd(RRender*const rc, const AcmeVector3 start, const AcmeVector3 end, const AcmeReal r,
                    const AcmeReal g, const AcmeReal b)
{
}

void MEAPI RLineRemoveAll(RRender* rc)
{
}

void MEAPI RNLineAddToList(RRender *rc, RNativeLine *line)
{
}

void MEAPI RNLineRemoveFromList(RRender *rc, RNativeLine *line)
{
}

void MEAPI RNLineDestroy(RNativeLine *line)
{
}

void MEAPI RNLineDelete(RRender *rc, RNativeLine *line)
{
}

RNativeLine* MEAPI RNLineCreate(RRender*const rc, const AcmeReal start[3], const AcmeReal end[3],
                                const AcmeReal color[4], const MeVector4 *const matrix)
{
    return {};
}

void MEAPI RNLineSetColor(RNativeLine* line, AcmeReal color[4])
{
}

void MEAPI RNLineMoveEnds(RNativeLine*const line, const AcmeReal start[3], const AcmeReal end[3])
{
}

void MEAPI RLightSwitchOn(RRender* rc, RRenderLight light)
{
}

void MEAPI RLightSwitchOff(RRender* rc, RRenderLight light)
{
}

void MEAPI RCameraGetLookAt(RRender *rc, AcmeVector3 camlookat)
{
    camlookat[0] = camlookat[1] = camlookat[2] = 0;
}

void MEAPI RCameraGetPosition(RRender *rc, AcmeVector3 pos)
{
    pos[0] = pos[1] = pos[2] = 0;
}

void MEAPI RCameraGetLookDir(RRender *rc, AcmeVector3 dir)
{
    dir[0] = dir[1] = dir[2] = 0;
}

AcmeReal MEAPI RCameraGetDistance(RRender *rc)
{
    return {};
}

AcmeReal MEAPI RCameraGetElevation(RRender *rc)
{
    return {};
}

AcmeReal MEAPI RCameraGetRotation(RRender *rc)
{
    return {};
}

void MEAPI RCameraGetUp(RRender *rc, AcmeVector3 up)
{
    up[0] = up[1] = up[2] = 0;
}

void MEAPI RCameraSetRotation(RRender *rc, const MeReal rotation)
{
}

void MEAPI RCameraSetElevation(RRender* rc, const MeReal elevation)
{
}

void MEAPI RCameraSetOffset(RRender* rc, const MeReal offset)
{
}

void MEAPI RCameraSetLookAt(RRender *rc, const AcmeVector3 lookAt)
{
}

void MEAPI RCameraSetLookAtAndPosition(RRender *rc, const MeVector3 lookAt, const MeVector3 position)
{
}

void MEAPI RCameraSetView(RRender *rc, AcmeReal dist, AcmeReal angle, AcmeReal elevation)
{
}

void MEAPI RCameraSetMatrix(RRender *rc, MeMatrix4Ptr m)
{
}

void MEAPI RCameraSetFov(RRender *rc, AcmeReal fov)
{
}

void MEAPI RCameraPanX(RRender *rc, AcmeReal dist)
{
}

void MEAPI RCameraPanY(RRender *rc, AcmeReal dist)
{
}

void MEAPI RCameraPanZ(RRender *rc, AcmeReal dist)
{
}

void MEAPI RCameraRotateAngle(RRender *rc, AcmeReal d_theta)
{
}

void MEAPI RCameraRotateElevation(RRender *rc, AcmeReal d_phi)
{
}

void MEAPI RCameraZoom(RRender *rc, AcmeReal dist)
{
}

void MEAPI RCameraUpdate(RRender *rc)
{
}

void MEAPI RCameraSetOrthographicView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RCameraSetFrontView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RCameraSetBackView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RCameraSetLeftView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RCameraSetRightView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RCameraSetTopView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RCameraSetBottomView(RRender* rc, AcmeReal zoomFactor, AcmeReal viewDepth)
{
}

void MEAPI RRenderSetActionNCallBackWithText(RRender *rc, int N, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetActionNKey(RRender* rc, const unsigned int N, const char key)
{
}

void MEAPI RRenderSetDown2CallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetDownCallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetJoystickCallBackWithText(RRender *rc, RJoystickCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetLeft2CallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetLeftCallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetMouseCallBackWithText(RRender *rc, RMouseCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetRight2CallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetRightCallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetUp2CallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

void MEAPI RRenderSetUpCallBackWithText(RRender *rc, RButtonPressCallBack func, void *userdata, char *text)
{
}

RPerformanceBar* MEAPI RPerformanceBarCreate(RRender *rc)
{
    return {};
}

void MEAPI RPerformanceBarUpdate(RRender *rc, AcmeReal coltime, AcmeReal dyntime, AcmeReal rentime, AcmeReal idletime)
{
}

void MEAPI RRenderDisplayFps(RRender *rc, AcmeReal fps)
{
}

RParticleSystem* MEAPI RParticleSystemNew(int numParticles, MeVector3 *positions, int tex_id, const float color[4],
                                          AcmeReal size)
{
    return {};
}

RParticleSystem* MEAPI RParticleSystemCreate(RRender *rc, int numParticles, MeVector3 *positions, char *tex_filename,
                                             const float color[4], AcmeReal tri_size)
{
    return {};
}

void MEAPI RParticleSystemDestroy(RParticleSystem *ps)
{
}

void MEAPI RParticleSystemAddToList(RRender *rc, RParticleSystem *ps)
{
}

void MEAPI RParticleSystemRemoveFromList(RRender *rc, RParticleSystem *ps)
{
}

RMenu* MEAPI RMenuCreate(RRender* rc, const char* name)
{
    return {};
}

void MEAPI RMenuDestroy(RMenu* rm)
{
}

void MEAPI RRenderSetDefaultMenu(RRender *rc, RMenu* menu)
{
}

void MEAPI RMenuDisplay(RMenu* rm)
{
}

void MEAPI RMenuAddToggleEntry(RMenu* rm, const char * name, RMenuToggleCallback func, MeBool defaultValue)
{
}

void MEAPI RMenuAddValueEntry(RMenu* rm, const char * name, RMenuValueCallback func, MeReal hi, MeReal lo,
                              MeReal increment, MeReal defaultValue)
{
}

void MEAPI RMenuAddSubmenuEntry(RMenu* rm, const char * name, RMenu* submenu)
{
}

void MEAPI RCalculatePortalCoordinates(RRender* rc, AcmeReal width, AcmeReal height, int x, int y, AcmeReal* portalx,
                                       AcmeReal* portaly)
{
    *portalx = *portaly = 0;
}

void MEAPI RDisplayBanner(void)
{
}

void MEAPI RDisplayCommandLineHelp(void)
{
}

} // extern "C"
