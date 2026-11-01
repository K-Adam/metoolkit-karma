#include <cstdio>
#include <cstring>

#include <MeBounding.h>
#include <MeChunk.h>
#include <MeCommandLine.h>
#include <MeMath.h>
#include <MeMisc.h>
#include <MeProfile.h>
#include <MeSimpleFile.h>

#include "check.h"

int main()
{
    // Command line options: eating removes entries, values come from the next argument.
    const char* argv[] = {"demo", "-gl", "-n", "42", "-f", "0.5", "-name", "ball", "-x"};
    MeCommandLineOptions* o = MeCommandLineOptionsCreate(9, argv);
    CHECK(MeCommandLineOptionsGetPos(o, "-n") == 2);
    CHECK(MeCommandLineOptionsCheckFor(o, "-gl", 1));
    CHECK(!MeCommandLineOptionsCheckFor(o, "-gl", 0));
    CHECK(MeCommandLineOptionsGetNumeric(o, "-n", 1) == 42);
    CHECK_NEAR(MeCommandLineOptionsGetFloat(o, "-f", 0), 0.5, 0);
    CHECK(std::strcmp(MeCommandLineOptionsGetString(o, "-name", 1), "ball") == 0);
    const char* list[] = {"-d3d", "-x", nullptr};
    CHECK(MeCommandLineOptionsCheckForList(o, list, 1));
    CHECK(o->m_argc == 3);
    CHECK(MeCommandLineOptionsGetString(o, "-missing", 0) == nullptr);
    CHECK(MeCommandLineOptionsGetString(o, "-f", 0) == argv[5]);
    CHECK(std::strcmp(argv[1], "-gl") == 0);
    MeCommandLineOptionsDestroy(o);

    // Chunk grows on demand, reuses in keep mode and frees in free mode.
    MeChunk c;
    MeChunkInit(&c, 16);
    void* m = MeChunkGetMem(&c, 100);
    CHECK(m && ((MeUintPtr)m % 16) == 0);
    CHECK(MeChunkIsInUse(&c));
    CHECK(MeChunkGetMem(&c, 10) == nullptr);
    MeChunkPutMem(&c, m);
    CHECK(MeChunkGetMem(&c, 50) == m);
    MeChunkPutMem(&c, m);
    m = MeChunkGetMem(&c, 150);
    CHECK(MeChunkGetCurrentSize(&c) == 200);
    CHECK(MeChunkGetMaxSize(&c) == 150);
    MeChunkSetMode(&c, kMeChunkModeFreeOnPut);
    MeChunkPutMem(&c, m);
    CHECK(MeChunkGetCurrentSize(&c) == 0 && !MeChunkIsInUse(&c));
    CHECK(MeChunkGetAlignment(&c) == 16);
    MeChunkTerm(&c);
    MeChunkInit(&c, 0);
    m = MeChunkGetMem(&c, 10);
    CHECK(m);
    MeChunkPutMem(&c, m);
    MeChunkTerm(&c);

    // Bounding sphere contains every point and is close to minimal.
    MeVector3 pts[] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0.2f, 0.3f, -0.9f}};
    MeVector3 centre;
    MeReal r;
    MeBoundingSphereCalc(pts, 6, centre, &r);
    for (auto& p : pts)
        CHECK(MeVector3Distance(p, centre) <= r + 1e-5f);
    CHECK(r < 1.1f);

    float rgb[3];
    MeHSVtoRGB(120, 1, 1, rgb);
    CHECK(rgb[0] == 0 && rgb[1] == 1 && rgb[2] == 0);
    MeHSVtoRGB(360, 0.5f, 0.8f, rgb);
    CHECK_NEAR(rgb[0], 0.8, 1e-6);
    CHECK_NEAR(rgb[2], 0.4, 1e-6);

    // Files round trip through the whole file helpers.
    const char* text = "karma\nfile";
    int f = MeOpen("me_utils.tmp", kMeOpenModeWRONLY);
    CHECK(f >= 0);
    MeSaveWholeFileHandle(f, text, (int)std::strlen(text));
    MeClose(f);
    // Both ends use text mode, so the newline round trips on every platform.
    f = MeOpenWithSearch("me_utils.tmp", kMeOpenModeRDONLY);
    CHECK(f >= 0);
    int size = 0;
    char* data = MeLoadWholeFileHandle(f, &size);
    MeClose(f);
    CHECK(data && std::strcmp(data, text) == 0 && size == (int)std::strlen(text));
    MeMemoryAPI.destroy(data);
    CHECK(MeOpenWithSearch("no_such_file.tmp", kMeOpenModeRDONLY) < 0);
    CHECK(MeGetDefaultFileLocation(0) != nullptr);
    CHECK(MeGetDefaultFileLocation(1000) == nullptr);
    std::remove("me_utils.tmp");

    // Profiler accumulates named sections per frame.
    MeProfileTimerMode mode = {};
    MeProfileStartTiming(mode, kMeProfileLogTotals);
    MeProfileStartFrame();
    MeProfileStartSection("a", 1);
    volatile double sink = 0;
    for (int i = 0; i < 100000; ++i)
        sink += i;
    MeProfileStartSection("b", 0);
    MeProfileEndSection("b");
    MeProfileEndFrame();
    CHECK(MeProfileGetSectionTime("a") > 0);
    CHECK(MeProfileGetAllSectionTime() >= MeProfileGetSectionTime("a"));
    CHECK(MeProfileGetSectionTime("none") == 0);
    MeProfileTimerResult t0, t1;
    MeProfileGetTimerValue(&t0);
    MeProfileGetTimerValue(&t1);
    CHECK(t1.cpuCycles >= t0.cpuCycles && MeProfileGetClockSpeed() > 0);
    MeProfileStopTiming();
    return 0;
}
