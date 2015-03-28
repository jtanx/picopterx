/**
 * @file opts.cpp
 * @brief Options and persistent configurations handler
 */

#include "common.h"
#include "opts.h"
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/filewritestream.h>
#include <rapidjson/prettywriter.h>

using picopter::Options;
using namespace rapidjson;

const char* Options::FAMILY_DEFAULT = "picopter";

/**
 * Finds the object based on the given key.
 * @param d The rapidjson document to search within.
 * @param key The key of the given entry.
 * @return A pointer to the given value, or nullptr if it doesn't exist.
 */
static Value* GetValue(Value *d, const char *key) {
    auto ret = d->FindMember(key);
    if (ret != d->MemberEnd()) {
        return &ret->value;
    }
    return nullptr;
}

/**
 * Constructor.
 * Initialised empty, optionally loading from a file.
 * @param file The location of a file containing options, or NULL if not present.
 */
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
        m_family_inst = nullptr;
    } else {
        Value *fi = GetValue(d, FAMILY_DEFAULT);
        if (fi && !fi->IsObject()) {
            fi->SetObject();
        }
        m_family_inst = fi;
    }
    
    m_doc = d;
    m_family = FAMILY_DEFAULT;
}

/**
 * Constructs an options class with no initial file.
 * This is the same as calling Options::Options(NULL).
 */
Options::Options() : Options(NULL) {}

/**
 * Destructor. Deletes the rapidjson instance.
 */
Options::~Options() {
    delete static_cast<Document*>(m_doc);
}

/**
 * Sets the family under which to store and retrieve settings from.
 * @param family The name of the family. If NULL, it will default to 
 *               Options::FAMILY_DEFAULT.
 */
void Options::SetFamily(const char *family) {
    m_family = family ? family : FAMILY_DEFAULT;
    Value *fi = GetValue(static_cast<Document*>(m_doc), family);
    if (fi && !fi->IsObject()) {
        fi->SetObject();
    }
    m_family_inst = fi;
}

/**
 * Retrieves the integer value associated with a key.
 * @param key The key to the value.
 * @param otherwise The value to return if it does not exist.
 * @return The retrieved value, or `otherwise` if it does not exist.
 */
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

/**
 * Retrieves the Boolean value associated with a key.
 * @param key The key to the value.
 * @param otherwise The value to return if it does not exist.
 * @return The retrieved value, or `otherwise` if it does not exist.
 */
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

/**
 * Retrieves the string value associated with a key.
 * @param key The key to the value.
 * @param otherwise The value to return if it does not exist.
 * @return The retrieved value, or `otherwise` if it does not exist. This value
 *         must not be modified. It will only be valid until this parameter is
 *         modified (via Set or Remove), or in the case of `otherwise` being 
 *         returned, until that value is either freed or goes out of scope. 
 */
const char* Options::GetString(const char *key, const char *otherwise) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = GetValue(fi, key);
        if (vpt && vpt->IsString()) {
            return vpt->GetString();
        }
    }
    return otherwise;
}

/**
 * Retrieves the Real (double precision) value associated with a key.
 * @param key The key to the value.
 * @param otherwise The value to return if it does not exist.
 * @return The retrieved value, or `otherwise` if it does not exist.
 */
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

/**
 * Helper method to store a new value.
 * If the currently selected family does not exist, it is first created before
 * adding the new value to it.
 * @param key The key to the value.
 * @param val The value to be stored.
 */
template<typename T>
void Options::SetImpl(const char *key, const T& val) {
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

/**
 * Specialisation of Options::SetImpl to store strings.
 * This specialisation is necessary because the value must be copied.
 * rapidjson requires that (a.) it explicitly be copied, or that (b.) it will
 * retain a reference that is guaranteed to be valid for longer than itself.
 * @param key The key to the value.
 * @param val The value to be stored.
 */
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

/**
 * Stores an integer value.
 * @param key The key to the value.
 * @param val The value to be stored.
 */
void Options::Set(const char *key, int val) {
    SetImpl(key, val);
}

/**
 * Stores an Boolean value.
 * @param key The key to the value.
 * @param val The value to be stored.
 */
void Options::Set(const char *key, bool val) {
    SetImpl(key, val);
}

/**
 * Stores an Real value.
 * @param key The key to the value.
 * @param val The value to be stored.
 */
void Options::Set(const char *key, double val) {
    SetImpl(key, val);
}

/**
 * Removes a value.
 * @param key The key to the value.
 * @param val The value to be stored.
 */
bool Options::Remove(const char *key) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        return fi->RemoveMember(key);
    }
    return false;
}

/**
 * Saves the current settings to the specified stream.
 */
void Options::Save(FILE *fp) {
    char buf[BUFSIZ];
    FileWriteStream fws(fp, buf, sizeof(buf));
    PrettyWriter<FileWriteStream> pw(fws);
    Document *d = static_cast<Document*>(m_doc);
    
    d->Accept(pw);
}

/**
 * Saves the current settings to a file.
 * Will save to the file that was specified on construction.
 * @throws std::invalid_argument If no file has been set previously.
 */
void Options::Save() {
    if (m_file.empty()) {
        throw std::invalid_argument("No input file specified.");
    } else {
        FILE *fp = fopen(m_file.c_str(), "wb");
        if (fp) {
            Save(fp);
            fclose(fp);
        }
    }
}

/**
 * Saves the current settings to the specified file.
 * Will keep a copy of the specified path for use with Save().
 */
void Options::Save(const char *file) {
    FILE *fp = fopen(file, "wb");
    if (fp) {
        Save(fp);
        fclose(fp);
        m_file = file;
    }
}