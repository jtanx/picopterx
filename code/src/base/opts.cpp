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
using rapidjson::StringRef;
using StringRefT = rapidjson::GenericValue<UTF8<>>::StringRefType;

const char* Options::FAMILY_DEFAULT = "picopter";

static inline Value* GetValue(Value *d, const char *key) {
    auto ret = d->FindMember(key);
    if (ret != d->MemberEnd()) {
        return &ret->value;
    }
    return nullptr;
}

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
    
    Value *fi = GetValue(d, FAMILY_DEFAULT);
    if (fi && !fi->IsObject()) {
        fi->SetObject();
    }
    m_family = FAMILY_DEFAULT;
    m_family_inst = fi;
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
    m_family = family ? family : FAMILY_DEFAULT;
    Value *fi = GetValue(static_cast<Document*>(m_doc), family);
    if (fi && !fi->IsObject()) {
        fi->SetObject();
    }
    m_family_inst = fi;
}

int Options::GetInt(const char *key, int otherwise) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = GetValue(fi, key);
        if (vpt && vpt->IsInt()) {
            return vpt->GetInt();
        }
    }
    return otherwise;
}

bool Options::GetBool(const char *key, bool otherwise) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = GetValue(fi, key);
        if (vpt && vpt->IsBool()) {
            return vpt->GetBool();
        }
    }
    return otherwise;
}

std::string Options::GetString(const char *key, const char *otherwise) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = GetValue(fi, key);
        if (vpt && vpt->IsString()) {
            return std::string(vpt->GetString());
        }
    }
    return std::string(otherwise);
}

double Options::GetReal(const char *key, double otherwise) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = GetValue(fi, key);
        if (vpt && vpt->IsDouble()) {
            return vpt->GetDouble();
        }
    }
    return otherwise;
}

template<typename T>
void Options::SetImpl(const char *key, T val) {
    Document *d = static_cast<Document*>(m_doc);
    Value *fi = static_cast<Value*>(m_family_inst);

    if (!fi) { //Family doesn't exist, so create it
        Value family_key(m_family.c_str(), d->GetAllocator());
        Value family_val(Type::kObjectType);
        Value entry_key(key, d->GetAllocator());
        Value entry_val(val);
        
        family_val.AddMember(entry_key, entry_val, d->GetAllocator());
        d->AddMember(family_key, family_val, d->GetAllocator());
        m_family_inst = &(*d)[m_family.c_str()];
    } else { //We have the family
        Value *existing = GetValue(fi, key);
        if (existing) { //The entry was already there, so just update value
            *existing = val;
        } else { //Create the new entry
            Value entry_key(key, d->GetAllocator());
            Value entry_val(val);
            fi->AddMember(entry_key, entry_val, d->GetAllocator());
        }
    }
}

void Options::Set(const char *key, int val) {
    SetImpl(key, val);
}

void Options::Set(const char *key, bool val) {
    SetImpl(key, val);
}

void Options::Set(const char *key, double val) {
    SetImpl(key, val);
}

void Options::Set(const char *key, const char *val) {
    Document *d = static_cast<Document*>(m_doc);
    Value *fi = static_cast<Value*>(m_family_inst);
    
    if (!fi) { //Family doesn't exist, so create it
        Value family_key(m_family.c_str(), d->GetAllocator());
        Value family_val(Type::kObjectType);
        Value entry_key(key, d->GetAllocator());
        Value entry_val(val, d->GetAllocator());
        
        family_val.AddMember(entry_key, entry_val, d->GetAllocator());
        d->AddMember(family_key, family_val, d->GetAllocator());
        m_family_inst = &(*d)[m_family.c_str()];
    } else { //We have the family
        Value *existing = GetValue(fi, key);
        if (existing) { //The entry was already there, so just update value
            existing->SetString(val, d->GetAllocator());
        } else { //Create the new entry
            Value entry_key(key, d->GetAllocator());
            Value entry_val(val, d->GetAllocator());
            fi->AddMember(entry_key, entry_val, d->GetAllocator());
        }
    }
}

bool Options::Remove(const char *key) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        return fi->RemoveMember(key);
    }
    return false;
}

void Options::Save() {
    char buf[BUFSIZ];
    FileWriteStream fp(stdout, buf, sizeof(buf));
    PrettyWriter<FileWriteStream> pw(fp);
    Document *d = static_cast<Document*>(m_doc);
    
    d->Accept(pw);
}