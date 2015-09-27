/**
 * @file opts.h
 * @brief Header for handling options and persistent configurations
 */

#ifndef _PICOPTERX_OPT_H
#define _PICOPTERX_OPT_H

namespace picopter {
    /** List parsing callback function **/
    typedef void (*ListParser)(const void *val, void *closure);
    
    /**
     * Provides methods to persistently store and retrieve options.
     * This class is not thread safe.
     */
    class Options {
        public:
            static void* GetValue(void *d, const char *key);
            
            Options();
            Options(const char *data, bool is_serialised=false);
            Options(const char *file, const char *json_string);
            virtual ~Options();

            void SetFamily(const char *family);
            bool Contains(const char *key);

            int GetInt(const char *key, int otherwise = 0);
            bool GetBool(const char *key, bool otherwise = false);
            const char* GetString(const char *key, const char *otherwise = "");
            double GetReal(const char *key, double otherwise = 0.0f);
            
            void GetList(const char *key, void *closure, ListParser callback);

            bool GetInt(const char *key, int *value);
            bool GetInt(const char *key, int *value, int min, int max);
            bool GetBool(const char *key, bool *value);
            bool GetReal(const char *key, double *value, double min, double max);

            void Set(const char *key, int val);
            void Set(const char *key, bool val);
            void Set(const char *key, const char *val);
            void Set(const char *key, double val);

            bool Remove(const char *key);

            bool Merge(const char *json_string);
            std::string Serialise();
            void Save();
            void Save(const char *file);
            void Save(FILE *fp);
        private:
            /** The default family to save settings under. **/
            static const char* FAMILY_DEFAULT;
            /** The path to the save file (if any) **/
            std::string m_file;
            /** The current family to save settings under. **/
            std::string m_family;

            /** The rapidjson Document. **/
            void *m_doc; //Because screw forward-declaring rapidjson
            /** A pointer to the current family instance **/
            void *m_family_inst;

            /** Copy constructor (disabled) **/
            Options(const Options &other);
            /** Assignment operator (disabled) **/
            Options& operator= (const Options &other);

            template<typename T>
            void SetImpl(const char *key, const T& val);
    };
}

#endif // _PICOPTERX_OPT_H