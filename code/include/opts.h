/**
 * @file opts.h
 * @brief Header for handling options and persistent configurations
 */

#ifndef _PICOPTERX_OPT_H
#define _PICOPTERX_OPT_H

#define OPT_FAMILY_DEFAULT "picopter"

namespace rapidjson {
    class Document;
}

namespace picopter {
    /**
     * Provides methods to persistently store and retrieve options.
     */
    class Options {
        public:
            Options();
            Options(const char *file);
            virtual ~Options();
            
            void SetFamily(const char *family);
            
            int GetInt(const char *key, int otherwise);
            int GetInt(const char *key);
            
            int GetBool(const char *key, bool otherwise);
            int GetBool(const char *key);
            
            std::string GetString(const char *key, const char *otherwise);
            std::string GetString(const char *key);
            
            double GetReal(const char *key, double otherwise);
            double GetReal(const char *key);
            
            template<typename T>
            bool Store(const char *key, const T& val);
            
            void Save();
            void Save(const char *file);
        private:
            std::string m_file;
            std::string m_family;
            
            rapidjson::Document *d = nullptr;
    };
}

#endif // _PICOPTERX_OPT_H