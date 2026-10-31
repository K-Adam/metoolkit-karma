#pragma once

#include <MeAssetDBXMLIO.h>

#include "check.h"

static const char* kRagDolls = KARMA_REFERENCE_DIR "/KarmaData/RagDolls.ka";

// Returns null when the file is missing so a test can skip.
static MeAssetDB* load(const char* path)
{
    MeStream stream = MeStreamOpen(path, kMeOpenModeRDONLY);
    if (!stream)
        return nullptr;
    MeAssetDB* db = MeAssetDBCreate();
    MeAssetDBXMLInput* input = MeAssetDBXMLInputCreate(db, NULL);
    CHECK(MeAssetDBXMLInputRead(input, stream));
    MeAssetDBXMLInputDestroy(input);
    MeStreamClose(stream);
    return db;
}
