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
            
            void SetFamily(const char *family);
            
            int GetInt(const char *key, int otherwise = 0);
            bool GetBool(const char *key, bool otherwise = false);
            std::string GetString(const char *key, const char *otherwise = "");
            double GetReal(const char *key, double otherwise = 0.0f);
            
            void Set(const char *key, int val);
            void Set(const char *key, bool val);
            void Set(const char *key, const char *val);
            void Set(const char *key, double val);
            
            bool Remove(const char *key);
            
            void Save();
            void Save(const char *file);
        private:
            static const char* FAMILY_DEFAULT;
            std::string m_file;
            std::string m_family;
            
            void *m_doc; //Because screw forward-declaring rapidjson
            void *m_family_inst;
            
            Options(const Options &other);
            template<typename T>
            void SetImpl(const char *key, T val);
    };
}

#endif // _PICOPTERX_OPT_H