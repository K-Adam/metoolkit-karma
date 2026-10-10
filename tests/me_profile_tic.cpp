#include <cstdio>
#include <cstring>

#include <MeProfile.h>

#include "check.h"

// The tic functions are only declared for PS2 builds, where the header does not turn them into macros.
#undef ticInit
#undef tic
#undef ticOutputResults
#undef ticCleanup
extern "C" {
void MEAPI ticInit();
void MEAPI tic(const char* name);
void MEAPI ticOutputResults(const char* filename);
void MEAPI ticCleanup();
}

int main()
{
    MeProfileOutput output = {kMeProfileOutputNormal, 0, 0};
    MeProfileSetOutputParameters(output);
    MeProfileTimerMode mode = {};
    MeProfileStartTiming(mode, kMeProfileLogTotals);
    MeProfileStartFrame();
    MeProfileStartSection("work", 0);
    MeProfileEndSection("work");
    MeProfileEndFrame();
    MeProfileOutputResults();
    MeProfileStopTiming();

    const char* path = "me_profile_tic.txt";
    ticInit();
    tic("start");
    tic("end");
    ticOutputResults(path);
    ticCleanup();

    std::FILE* f = std::fopen(path, "r");
    CHECK(f);
    char name[2][16];
    double ms[2];
    CHECK(std::fscanf(f, "%15s %lf %15s %lf", name[0], &ms[0], name[1], &ms[1]) == 4);
    std::fclose(f);
    std::remove(path);
    CHECK(std::strcmp(name[0], "start") == 0 && std::strcmp(name[1], "end") == 0);
    CHECK(ms[0] == 0 && ms[1] >= 0);
    return 0;
}
