/**
 * @file opts.h
 * @brief Header for handling options and persistent configurations
 */

#ifndef _PICOPTERX_OPT_H
#define _PICOPTERX_OPT_H

#define OPT_FAMILY_DEFAULT "picopter"

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
            
            int GetInt(const char *key, int otherwise = 0);
            int GetBool(const char *key, bool otherwise = false);
            std::string GetString(const char *key, const char *otherwise = "");
            double GetReal(const char *key, double otherwise = 0.0f);
            
            template<typename T>
            bool Store(const char *key, const T& val);
            
            void Save();
            void Save(const char *file);
        private:
            std::string m_file;
            std::string m_family;
            
            void *m_doc; //Because screw forward-declaring rapidjson
    };
}

#endif // _PICOPTERX_OPT_H