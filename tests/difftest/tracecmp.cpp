// tracecmp <exe> <golden>          compare the exe's trace with the golden trace
// tracecmp --write <exe> <golden>  record the exe's trace as the golden trace
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#ifdef _WIN32
#define popen _popen
#define pclose _pclose
#endif

namespace {

const int kSkip = 77;

std::vector<std::string> traceOf(const std::string& exe)
{
    std::vector<std::string> lines;
    FILE* p = popen(("\"" + exe + "\"").c_str(), "r");
    if (!p)
        return lines;
    char buf[1024];
    while (std::fgets(buf, sizeof buf, p))
    {
        std::string line(buf);
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
            line.pop_back();
        if (!line.empty() && (line[0] == '=' || line[0] == '~'))
            lines.push_back(line);
    }
    if (pclose(p) != 0)
        lines.push_back("! scenario exited with an error");
    return lines;
}

bool isNumber(const std::string& s, double& v)
{
    char* end;
    v = std::strtod(s.c_str(), &end);
    return !s.empty() && *end == '\0';
}

bool same(const std::string& got, const std::string& want)
{
    if (got == want)
        return true;
    if (want[0] != '~')
        return false;
    std::istringstream g(got), w(want);
    std::string tolToken;
    w >> tolToken;
    double tol = std::atof(tolToken.c_str() + 1);
    std::string a, b;
    g >> a;
    if (a != tolToken)
        return false;
    while (true)
    {
        bool moreA = static_cast<bool>(g >> a), moreB = static_cast<bool>(w >> b);
        if (!moreA || !moreB)
            return moreA == moreB;
        double x, y;
        if (isNumber(a, x) && isNumber(b, y) ? std::fabs(x - y) > tol : a != b)
            return false;
    }
}

} // namespace

int main(int argc, char** argv)
{
    bool write = argc == 4 && std::strcmp(argv[1], "--write") == 0;
    if (argc != 3 && !write)
    {
        std::fprintf(stderr, "usage: tracecmp [--write] <exe> <golden>\n");
        return 2;
    }
    std::string exe = argv[argc - 2], golden = argv[argc - 1];
    std::vector<std::string> got = traceOf(exe);

    if (write)
    {
        std::ofstream out(golden);
        for (const std::string& l : got)
            out << l << '\n';
        return 0;
    }

    std::ifstream in(golden);
    if (!in)
    {
        std::printf("no golden trace at %s\n", golden.c_str());
        return kSkip;
    }
    std::vector<std::string> want;
    for (std::string l; std::getline(in, l);)
    {
        // Golden traces are committed with CRLF, which getline keeps outside Windows.
        if (!l.empty() && l.back() == '\r')
            l.pop_back();
        if (!l.empty())
            want.push_back(l);
    }

    int failures = 0;
    for (size_t i = 0; i < want.size() || i < got.size(); ++i)
    {
        const std::string& g = i < got.size() ? got[i] : "<missing>";
        const std::string& w = i < want.size() ? want[i] : "<missing>";
        if (i >= got.size() || i >= want.size() || !same(g, w))
        {
            std::printf("line %zu\n  reference: %s\n  ours:      %s\n", i + 1, w.c_str(), g.c_str());
            ++failures;
        }
    }
    return failures ? 1 : 0;
}
