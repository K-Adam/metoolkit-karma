#include <algorithm>
#include <vector>

#include "McdImpl.h"

extern "C" {

// Keeps the deepest contact, then repeatedly the one farthest (across avgNormal) from those kept, so the
// survivors span the contact patch. faceNormalsFirst and scale have no effect on this selection.
int MEAPI McdContactSimplify(const MeReal* avgNormal, McdContact* contacts_src, const int nContacts_src,
                             McdContact* contacts_dst, const int nContacts_dest, const int faceNormalsFirst,
                             const MeReal scale)
{
    (void)faceNormalsFirst;
    (void)scale;
    int keep = std::min(nContacts_src, nContacts_dest);
    if (keep <= 0)
        return 0;
    btVector3 n = toBt(avgNormal);
    auto across = [&](int i, int j) {
        btVector3 d = toBt(contacts_src[i].position) - toBt(contacts_src[j].position);
        return (d - n * n.dot(d)).length2();
    };
    std::vector<McdContact> out;
    int deepest = 0;
    for (int i = 1; i < nContacts_src; ++i)
        if (contacts_src[i].separation < contacts_src[deepest].separation)
            deepest = i;
    out.push_back(contacts_src[deepest]);
    std::vector<btScalar> nearest(nContacts_src);
    for (int i = 0; i < nContacts_src; ++i)
        nearest[i] = across(i, deepest);
    nearest[deepest] = -1;
    while ((int)out.size() < keep)
    {
        int best = (int)(std::max_element(nearest.begin(), nearest.end()) - nearest.begin());
        out.push_back(contacts_src[best]);
        for (int i = 0; i < nContacts_src; ++i)
            if (nearest[i] >= 0)
                nearest[i] = std::min(nearest[i], across(i, best));
        nearest[best] = -1;
    }
    // The source and destination may overlap, so the selection is copied through a buffer.
    std::copy(out.begin(), out.end(), contacts_dst);
    return keep;
}

} // extern "C"
