/**
 * @file opts.h
 * @brief Header for handling options and persistent configurations
 */

#ifndef _PICOPTERX_OPT_H
#define _PICOPTERX_OPT_H

namespace picopter {
    /**
     * Provides methods to persistently store and retrieve options.
     */
    class Options {
        public:
            Options();
            Options(const char *file);
            virtual ~Options();
            
            int GetInt(const char *family, const char *key, int otherwise);
            int GetInt(const char *family, const char *key);
            
            std::string GetString(const char *family, const char *key, const std::string &otherwise);
            std::string GetString(const char *family, const char *key);
            
            std::string GetReal(const char *family, const char *key, double otherwise);
            std::string GetReal(const char *family, const char *key);
            
            template<typename T>
            void Store(const char *family, const char *key, const T& val);
            
            void Save();
            void Save(const char *file);
    };
}

#endif // _PICOPTERX_OPT_H