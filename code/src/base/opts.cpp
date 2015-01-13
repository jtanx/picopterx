/**
 * @file opts.cpp
 * @brief Options and persistent configurations handler
 */

#include "picopter.h"
#include "rapidjson/document.h"
#include <rapidjson/filereadstream.h>

using picopter::Options;
using rapidjson::Document;
using rapidjson::UTF8;
using rapidjson::FileReadStream;

Options::Options(const char *file) {
    Document *d = new Document();
    
    if (file) {
        FILE *fp = fopen(file, "rb");
        if (fp) {
            char buffer[BUFSIZ];
            FileReadStream is(fp, buffer, sizeof(buffer));
            d->ParseStream<0, UTF8<>, FileReadStream>(is);
            fclose(fp);
            
            m_file = std::string(file);
        }
    }
    m_doc = d;
}

/**
 * Constructs an options class with no initial file.
 * This is the same as calling Options::Options(NULL).
 */
Options::Options() : Options(NULL) {}

Options::~Options() {
    delete static_cast<Document*>(m_doc);
}

void Options::SetFamily(const char *family) {
    m_family = std::string(family);
}

int Options::GetInt(const char *key, int otherwise) {
    Document *d = static_cast<Document*>(m_doc);
    const char *family = m_family.empty() ? OPT_FAMILY_DEFAULT : m_family.c_str();

    auto fi = d->FindMember(family);
    if (fi != d->MemberEnd() && fi->value.IsObject()) {
        auto &fam = fi->value;
        auto vi = fam.FindMember(key);
        if (vi != fam.MemberEnd() && vi->value.IsInt()) {
            return vi->value.GetInt();
        }
    }
    
    return otherwise;
}