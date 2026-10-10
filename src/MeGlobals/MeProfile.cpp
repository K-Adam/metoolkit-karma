#include <chrono>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include <MeProfile.h>

// Outside PS2 builds the header turns the tic functions into empty macros. They are still defined here so that
// code built for the PS2 interface links.
#undef ticInit
#undef tic
#undef ticOutputResults
#undef ticCleanup

namespace {

using Clock = std::chrono::steady_clock;

struct Section
{
    Clock::time_point start;
    Clock::duration frame{};
    Clock::duration total{};
    bool running = false;
    bool autoStop = false;
};

// ponytail: one global profiler that keeps last frame and running totals per section. Per frame logs (kMeProfileLogAll) need a history here.
struct Profiler
{
    bool enabled = false;
    int frames = 0;
    std::map<std::string, Section> sections;
};

Profiler profiler;
// Survives MeProfileStartTiming, which only resets the measurements.
MeProfileOutput output{kMeProfileOutputAverage, 0, 0};

struct TicEvent
{
    const char* name;
    Clock::time_point time;
};

std::vector<TicEvent> tics;

double ms(Clock::duration d)
{
    return std::chrono::duration<double, std::milli>(d).count();
}

void stop(Section& s)
{
    if (!s.running)
        return;
    s.frame += Clock::now() - s.start;
    s.running = false;
}

} // namespace

extern "C" {

void MEAPI MeProfileStartTiming(MeProfileTimerMode, MeProfileLogModes_enum)
{
    profiler = Profiler{};
    profiler.enabled = true;
}

void MEAPI MeProfileStopTiming()
{
    profiler = Profiler{};
}

void MEAPI MeProfileStartFrame()
{
    for (auto& [name, s] : profiler.sections)
        s.frame = {};
}

void MEAPI MeProfileEndFrame()
{
    if (!profiler.enabled)
        return;
    MeProfileStopTimers();
    for (auto& [name, s] : profiler.sections)
        s.total += s.frame;
    ++profiler.frames;
}

void MEAPI MeProfileStartSectionFn(const char* codeSection, char unsigned autoStop)
{
    if (!profiler.enabled)
        return;
    for (auto& [name, s] : profiler.sections)
        if (s.autoStop)
            stop(s);
    Section& s = profiler.sections[codeSection];
    stop(s);
    s.start = Clock::now();
    s.running = true;
    s.autoStop = autoStop != 0;
}

void MEAPI MeProfileEndSectionFn(const char* codeSection)
{
    auto it = profiler.sections.find(codeSection);
    if (it != profiler.sections.end())
        stop(it->second);
}

void MEAPI MeProfileGetTimerValue(MeProfileTimerResult* const result)
{
    result->cpuCycles = static_cast<MeU64>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
    result->count0 = 0;
    result->count1 = 0;
}

void MEAPI MeProfileStopTimers()
{
    for (auto& [name, s] : profiler.sections)
        stop(s);
}

void MEAPI MeProfileOutputResults()
{
    int frames = profiler.frames > 0 ? profiler.frames : 1;
    // Gnuplot output prints like the normal style, as writing plot files was never requested here.
    if (output.style == kMeProfileOutputAverage)
    {
        std::printf("%-32s %12s\n", "section", "average ms");
        for (auto& [name, s] : profiler.sections)
            std::printf("%-32s %12.3f\n", name.c_str(), ms(s.total) / frames);
        return;
    }
    std::printf("%-32s %12s %12s %12s\n", "section", "last ms", "average ms", "total ms");
    for (auto& [name, s] : profiler.sections)
        std::printf("%-32s %12.3f %12.3f %12.3f\n", name.c_str(), ms(s.frame), ms(s.total) / frames, ms(s.total));
}

MeReal MEAPI MeProfileGetSectionTime(const char* codeSection)
{
    auto it = profiler.sections.find(codeSection);
    return it == profiler.sections.end() ? 0 : static_cast<MeReal>(ms(it->second.frame));
}

MeReal MEAPI MeProfileGetAllSectionTime()
{
    double sum = 0;
    for (auto& [name, s] : profiler.sections)
        sum += ms(s.frame);
    return static_cast<MeReal>(sum);
}

MeU64 MEAPI MeProfileGetClockSpeed()
{
    // MeProfileGetTimerValue counts nanoseconds.
    return 1000000000u;
}

void MEAPI MeProfileSetOutputParameters(MeProfileOutput o)
{
    output = o;
}

void MEAPI ticInit()
{
    tics.clear();
}

// Names are kept by pointer, as the low overhead interface expects string literals.
void MEAPI tic(const char* name)
{
    tics.push_back({name, Clock::now()});
}

// Writes each event with the milliseconds since the previous one.
void MEAPI ticOutputResults(const char* filename)
{
    std::FILE* f = std::fopen(filename, "w");
    if (!f)
        return;
    for (size_t i = 0; i < tics.size(); ++i)
        std::fprintf(f, "%s %.6f\n", tics[i].name ? tics[i].name : "", i ? ms(tics[i].time - tics[i - 1].time) : 0.0);
    std::fclose(f);
}

void MEAPI ticCleanup()
{
    tics = {};
}

} // extern "C"
