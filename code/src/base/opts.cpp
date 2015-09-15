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
 * Finds the object based on the given key (internal rep).
 * @param d The rapidjson document to search within.
 * @param key The key of the given entry.
 * @return A pointer to the given value, or nullptr if it doesn't exist.
 */
static Value* LGetValue(Value *d, const char *key) {
    auto ret = d->FindMember(key);
    if (ret != d->MemberEnd()) {
        return &ret->value;
    }
    return nullptr;
}

/**
 * Finds the object based on the given key.
 * @param d The rapidjson document to search within.
 * @param key The key of the given entry.
 * @return A pointer to the given value, or nullptr if it doesn't exist.
 */
void* Options::GetValue(void *d, const char *key) {
    return static_cast<void*>(LGetValue(static_cast<Value*>(d), key));
}

/**
 * Constructor.
 * Initialised empty, optionally loading from a file or serialised string.
 * @param file The location of a file containing options, or nullptr if not present.
 * @param json_string The serialised JSON string to load, or nullptr if not present.
 *                    Will take precedence over the contents of the specified
 *                    file, if any.
 */
Options::Options(const char *file, const char *json_string) {
    Document *d = new Document();

    if (json_string) {
        d->Parse<0>(json_string);
    }

    if (file) {
        FILE *fp = fopen(file, "rb");
        if (fp) {
            if (!json_string) {
                char buffer[BUFSIZ];
                FileReadStream is(fp, buffer, sizeof(buffer));
                d->ParseStream<0, UTF8<>, FileReadStream>(is);
            }

            fclose(fp);
            m_file = std::string(file);
        }
    }

    if (!d->IsObject()) {
        d->SetObject();
        m_family_inst = nullptr;
    } else {
        Value *fi = LGetValue(d, FAMILY_DEFAULT);
        if (fi && !fi->IsObject()) {
            fi->SetObject();
        }
        m_family_inst = fi;
    }

    m_doc = d;
    m_family = FAMILY_DEFAULT;
}

/**
 * Constructs an options class from the specified file or serialised string.
 * @param data The file location or serialised string.
 * @param is_serialised Determines if 'data' is a file location or if it is
 *                      serialised JSON data.
 */
Options::Options(const char *data, bool is_serialised)
: Options(is_serialised ? nullptr : data, is_serialised ? data : nullptr) {}

/**
 * Constructs an options class with no initial file.
 * This is the same as calling Options::Options(nullptr, nullptr);
 */
Options::Options() : Options(nullptr, nullptr) {}

/**
 * Destructor. Deletes the rapidjson instance.
 */
Options::~Options() {
    delete static_cast<Document*>(m_doc);
}

/**
 * Sets the family under which to store and retrieve settings from.
 * @param family The name of the family. If nullptr, it will default to
 *               Options::FAMILY_DEFAULT.
 */
void Options::SetFamily(const char *family) {
    m_family = family ? family : FAMILY_DEFAULT;
    Value *fi = LGetValue(static_cast<Document*>(m_doc), family);
    if (fi && !fi->IsObject()) {
        fi->SetObject();
    }
    m_family_inst = fi;
}

/**
 * Determines if the specified key exists.
 * @param [in] key The key to the value.
 * @return true iff the key exists.
 */
bool Options::Contains(const char *key) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        return LGetValue(fi, key) != nullptr;
    }
    return false;
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
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsInt()) {
            int ret = vpt->GetInt();
            LogSimple(LOG_INFO, "%s: %d", key, ret);
            return ret;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %d", key, otherwise);
    return otherwise;
}

/**
 * Retrieves the integer value associated with a key.
 * @param [in] key The key to the value.
 * @param [in,out] value The location to store the value. It will be left
 *                       unchanged if the key does not exist.
 * @return true iff the value was retreived.
 */
bool Options::GetInt(const char *key, int *value) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsInt()) {
            *value = vpt->GetInt();
            LogSimple(LOG_INFO, "%s: %d", key, *value);
            return true;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %d", key, *value);
    return false;
}

/**
 * Retrieves the integer value associated with a key.
 * This version will clamp the value to between the specified bounds.
 * @param [in] key The key to the value.
 * @param [in,out] value The location to store the value. It will be left
 *                       unchanged (incl. no clamping) if the key does not exist.
 * @param [in] min The minimum parameter value.
 * @param [in] max The maximum parameter value.
 * @return true iff the value was retrieved.
 */
bool Options::GetInt(const char *key, int *value, int min, int max) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsInt()) {
            *value = picopter::clamp(vpt->GetInt(), min, max);
            LogSimple(LOG_INFO, "%s: %d", key, *value);
            return true;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %d", key, *value);
    return false;
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
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsBool()) {
            bool ret = vpt->GetBool();
            LogSimple(LOG_INFO, "%s: %s", key, ret ? "true" : "false");
            return ret;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %s", key, otherwise ? "true" : "false");
    return otherwise;
}

/**
 * Retrieves the Boolean value associated with a key.
 * @param [in] key The key to the value.
 * @param [in,out] value The location to store the value. It will be left
 *                       unchanged if the key does not exist.
 * @return true iff the value was retreived.
 */
bool Options::GetBool(const char *key, bool *value) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsBool()) {
            *value = vpt->GetBool();
            LogSimple(LOG_INFO, "%s: %s", key, *value ? "true" : "false");
            return true;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %s", key, *value ? "true" : "false");
    return false;
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
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsString()) {
            const char *ret = vpt->GetString();
            LogSimple(LOG_INFO, "%s: %s", key, ret ? ret : "Null");
            return ret;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %s", key, otherwise ? otherwise : "Null");
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
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsDouble()) {
            double ret = vpt->GetDouble();
            LogSimple(LOG_INFO, "%s: %f", key, ret);
            return ret;
        } else if (vpt && vpt->IsInt()) {
            int ret = vpt->GetInt();
            LogSimple(LOG_INFO, "%s: %d", key, ret);
            return ret;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %f", key, otherwise);
    return otherwise;
}

/**
 * Retrieves the Real (double precision) value associated with a key.
 * This version will clamp the value to between the specified bounds.
 * @param [in] key The key to the value.
 * @param [in,out] value The location to store the value. It will be left
 *                       unchanged (incl. no clamping) if the key does not exist.
 * @param [in] min The minimum parameter value.
 * @param [in] max The maximum parameter value.
 * @return true iff the value was retrieved.
 */
bool Options::GetReal(const char *key, double *value, double min, double max) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsDouble()) {
            *value = picopter::clamp(vpt->GetDouble(), min, max);
            LogSimple(LOG_INFO, "%s: %f", key, *value);
            return true;
        } else if (vpt && vpt->IsInt()) {
            *value = picopter::clamp(static_cast<double>(vpt->GetInt()), min, max);
            LogSimple(LOG_INFO, "%s: %d", key, *value);
            return true;
        }
    }
    LogSimple(LOG_INFO, "%s [default]: %f", key, *value);
    return false;
}

/**
 * Retrieves a list from the options.
 * @param [in] key The key to the list.
 * @param [in] closure The user-specified value to pass to the callback.
 * @param [in] cb The callback to parse each object in the list.
 */
void Options::GetList(const char *key, void *closure, ListParser cb) {
    Value *fi = static_cast<Value*>(m_family_inst);
    if (fi) {
        Value *vpt = LGetValue(fi, key);
        if (vpt && vpt->IsObject()) {
            for(Value::ConstMemberIterator it=vpt->MemberBegin(); it != vpt->MemberEnd(); ++it) {
               cb(&it->value, closure);
            }
        } else if (vpt && vpt->IsArray()) {
            for (auto it = vpt->Begin(); it != vpt->End(); ++it) {
                if (it->IsObject()) {
                    cb(it, closure);
                }
            }
        }
        
    }

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
        Value *existing = LGetValue(fi, key);
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
        Value *existing = LGetValue(fi, key);
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
 * Merge one set of options into this one.
 * All settings in the input take precedence over anything currently stored.
 * @param json_string The serialised JSON string to merge from.
 * @return true iff successfully merged.
 */
bool Options::Merge(const char *json_string) {
    if (json_string) {
        Document d;
        d.Parse<0>(json_string);
        if (!d.HasParseError() && d.IsObject()) {
            //Loop through each family
            for (Value::ConstMemberIterator it = d.MemberBegin();
                 it != d.MemberEnd(); it++)
            {
                //Do we have a family of options? (e.g. an object)
                if (it->value.IsObject()) {
                    SetFamily(it->name.GetString());
                    //Loop through each option
                    for (Value::ConstMemberIterator optit = it->value.MemberBegin();
                         optit != it->value.MemberEnd(); optit++)
                    {
                        const char *optkey = optit->name.GetString();
                        switch (optit->value.GetType()) {
                            case kFalseType: case kTrueType:
                                Set(optkey, optit->value.GetBool());
                            case kNumberType:
                                if (optit->value.IsInt()) {
                                    Set(optkey, optit->value.GetInt());
                                } else if (optit->value.IsNumber()) {
                                    Set(optkey, optit->value.GetDouble());
                                } else if (optit->value.IsBool()) {
                                    Set(optkey, optit->value.GetBool());
                                }
                            break;
                            case kStringType:
                                Set(optkey, optit->value.GetString());
                            break;
                            case kNullType: //Ignore
                            break;
                            default:
                                Log(LOG_WARNING, "Ignoring unknown option %s of type %d",
                                    optkey, optit->value.GetType());
                        }
                    }
                } else {
                    Log(LOG_WARNING, "Ignoring unknown family %s of type %d",
                        it->name.GetString(), it->value.GetType());
                }
            }
            return true;
        }
    }

    return false;
}

/**
 * Serialises the options into a UTF-8 encoded JSON string.
 * @return The serialised option data.
 */
std::string Options::Serialise() {
    GenericStringBuffer<UTF8<>> buffer;
    PrettyWriter<GenericStringBuffer<UTF8<>>> pw(buffer);
    Document *d = static_cast<Document*>(m_doc);

    d->Accept(pw);
    return std::string(buffer.GetString());
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
