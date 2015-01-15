/**
 * @file opts.cpp
 * @brief Options and persistent configurations handler
 */

#include "picopter.h"
#include "rapidjson/document.h"
#include <rapidjson/filereadstream.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>

using picopter::Options;
using rapidjson::Document;
using rapidjson::UTF8;
using rapidjson::FileReadStream;
using rapidjson::FileWriteStream;
using rapidjson::PrettyWriter;
using rapidjson::Value;
using rapidjson::Type;
using StringRefT = rapidjson::GenericValue<UTF8<>>::StringRefType;

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
    
    if (!d->IsObject()) {
        d->SetObject();
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

bool Options::GetBool(const char *key, bool otherwise) {
    Document *d = static_cast<Document*>(m_doc);
    const char *family = m_family.empty() ? OPT_FAMILY_DEFAULT : m_family.c_str();

    auto fi = d->FindMember(family);
    if (fi != d->MemberEnd() && fi->value.IsObject()) {
        auto &fam = fi->value;
        auto vi = fam.FindMember(key);
        if (vi != fam.MemberEnd() && vi->value.IsBool()) {
            return vi->value.GetBool();
        }
    }
    
    return otherwise;
}

std::string Options::GetString(const char *key, const char *otherwise) {
    Document *d = static_cast<Document*>(m_doc);
    const char *family = m_family.empty() ? OPT_FAMILY_DEFAULT : m_family.c_str();

    auto fi = d->FindMember(family);
    if (fi != d->MemberEnd() && fi->value.IsObject()) {
        auto &fam = fi->value;
        auto vi = fam.FindMember(key);
        if (vi != fam.MemberEnd() && vi->value.IsString()) {
            return std::string(vi->value.GetString());
        }
    }
    
    return std::string(otherwise);
}

double Options::GetReal(const char *key, double otherwise) {
    Document *d = static_cast<Document*>(m_doc);
    const char *family = m_family.empty() ? OPT_FAMILY_DEFAULT : m_family.c_str();

    auto fi = d->FindMember(family);
    if (fi != d->MemberEnd() && fi->value.IsObject()) {
        auto &fam = fi->value;
        auto vi = fam.FindMember(key);
        if (vi != fam.MemberEnd() && vi->value.IsDouble()) {
            return vi->value.GetDouble();
        }
    }
    
    return otherwise;
}

template<typename T>
bool Options::Store(const char *key, const T& val) {
    Document *d = static_cast<Document*>(m_doc);
    const char *family = m_family.empty() ? OPT_FAMILY_DEFAULT : m_family.c_str();

    auto fi = d->FindMember(family);
    Value* fpt;
    if (fi == d->MemberEnd()) {
        Value f(Type::kObjectType);
        d->AddMember(static_cast<StringRefT>(family), f, d->GetAllocator());
        fpt = &(*d)[family];
    } else {
        fpt = &fi->value;
    }
    if (fpt->IsObject()) {
        Value v(val);
        fpt->AddMember(static_cast<StringRefT>(key), v, d->GetAllocator());
    } else {
        return false;
    }

    return true;
}

void Options::Save() {
    char buf[BUFSIZ];
    Store("TEST", "JAJAJA");
    Store("TEST2", 1.233);
    Store("TEST3", true);
    Store("TEST3", 20);
    FileWriteStream fp(stdout, buf, sizeof(buf));
    PrettyWriter<FileWriteStream> pw(fp);
    Document *d = static_cast<Document*>(m_doc);
    
    d->Accept(pw);
}