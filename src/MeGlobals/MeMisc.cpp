#include <cmath>

#include <MeCall.h>
#include <MeMisc.h>

extern "C" void MEAPI MeHSVtoRGB(float h, float s, float v, float rgb[3])
{
    // h is in degrees, s and v in [0, 1].
    float sector = std::fmod(h, 360.0f);
    if (sector < 0)
        sector += 360.0f;
    sector /= 60.0f;
    int i = static_cast<int>(sector);
    float f = sector - i;
    float p = v * (1 - s), q = v * (1 - s * f), t = v * (1 - s * (1 - f));
    float table[6][3] = {{v, t, p}, {q, v, p}, {p, v, t}, {p, q, v}, {t, p, v}, {v, p, q}};
    for (int c = 0; c < 3; ++c)
        rgb[c] = table[i % 6][c];
}
