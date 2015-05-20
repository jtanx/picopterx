#include "gtest/gtest.h"
#include "picopter.h"

using picopter::Options;

class OptionsTest : public ::testing::Test {
    protected:
        OptionsTest() {
            LogInit();
        }
};

TEST_F(OptionsTest, TestFileLoad) {
    Options opts("data/opts_data.txt");
    
    ASSERT_STREQ("", opts.GetString("a"));
    ASSERT_EQ(0, opts.GetInt("a"));
    ASSERT_FALSE(opts.GetBool("a"));
    ASSERT_EQ(0.0f, opts.GetReal("a"));
    
    ASSERT_STREQ("string_test\u00eeaaa", opts.GetString("b"));
    
    ASSERT_DOUBLE_EQ(1.2433, opts.GetReal("c"));
    ASSERT_EQ(20, opts.GetInt("d"));
    ASSERT_TRUE(opts.GetBool("e"));
}

TEST_F(OptionsTest, TestFamilySetting) {
    Options opts("data/opts_data.txt");
    
    ASSERT_EQ(20, opts.GetInt("d"));
    opts.SetFamily("GPS");
    ASSERT_EQ(0, opts.GetInt("d"));
    
    ASSERT_FALSE(opts.GetBool("sane", true));
    
    opts.SetFamily("IMU");
    ASSERT_DOUBLE_EQ(204.44, opts.GetReal("yaw"));
    ASSERT_DOUBLE_EQ(-17.78, opts.GetReal("pitch"));
    ASSERT_DOUBLE_EQ(1093.33, opts.GetReal("elevation"));
	
    opts.SetFamily("");
    ASSERT_TRUE(opts.GetBool("empty"));
}

TEST_F(OptionsTest, TestInvalidFamily) {
    Options opts("data/opts_data.txt");
    
    opts.SetFamily("DOES_NOT_EXIST");
    ASSERT_FALSE(opts.GetBool("yaw"));
    ASSERT_EQ(0, opts.GetInt("yaw"));
    ASSERT_EQ(0.0f, opts.GetReal("yaw"));
    ASSERT_STREQ("", opts.GetString("yaw"));
}

TEST_F(OptionsTest, TestNonExistantFile) {
    Options opts("data/__nowhere.txt");
    ASSERT_FALSE(opts.GetBool("yaw"));
    ASSERT_EQ(0, opts.GetInt("yaw"));
    ASSERT_EQ(0.0f, opts.GetReal("yaw"));
    ASSERT_STREQ("", opts.GetString("yaw"));
}

TEST_F(OptionsTest, TestModifying) {
    Options opts("data/opts_data.txt");
    
    opts.Set("a", 11201131);
    ASSERT_EQ(11201131, opts.GetInt("a"));
    opts.Set("a", "nope");
    ASSERT_EQ(0, opts.GetInt("a"));
    ASSERT_STREQ("nope", opts.GetString("a"));
    
    opts.Set("b", 1.2441);
    ASSERT_DOUBLE_EQ(1.2441, opts.GetReal("b"));
    
    opts.Set("c", true);
    ASSERT_TRUE(opts.GetBool("c"));
}

TEST_F(OptionsTest, TestModifyingAddingFamily) {
    Options opts("data/opts_data.txt");
    
    opts.SetFamily("GPS");
    opts.Set("yaw", 1241);
    ASSERT_EQ(1241, opts.GetInt("yaw"));
    
    ASSERT_EQ(0, opts.GetInt("bling"));
    opts.Set("bling", 400);
    ASSERT_EQ(400, opts.GetInt("bling"));
}

TEST_F(OptionsTest, TestEmpty) {
    Options opts;
    
    ASSERT_FALSE(opts.GetBool("yaw"));
    ASSERT_EQ(0, opts.GetInt("yaw"));
    ASSERT_EQ(0.0f, opts.GetReal("yaw"));
    ASSERT_STREQ("", opts.GetString("yaw"));
    
    opts.Set("yaw", "helo");
    ASSERT_STREQ("helo", opts.GetString("yaw"));
    
    opts.Set("aaa", 331);
    opts.Set("bbb", true);
    opts.Set("ccc", 2.3311);
    ASSERT_EQ(331, opts.GetInt("aaa"));
    ASSERT_TRUE(opts.GetBool("bbb"));
    ASSERT_DOUBLE_EQ(2.3311, opts.GetReal("ccc"));
}

TEST_F(OptionsTest, TestRemove) {
    Options opts;
    
    opts.Set("tfm", "vfr");
    ASSERT_STREQ("vfr", opts.GetString("tfm"));
    opts.Remove("tfm");
    ASSERT_STREQ("", opts.GetString("tfm"));
    opts.Remove("tfm"); //???
}


TEST_F(OptionsTest, TestRemoveExisting) {
    Options opts("data/opts_data.txt");
    
    opts.Remove("b");
    ASSERT_STREQ("", opts.GetString("b"));
}

TEST_F(OptionsTest, TestSaveUnset) {
    Options opts;
    
    opts.Set("tfm", "vfr");
    ASSERT_THROW(opts.Save(), std::invalid_argument);
}

TEST_F(OptionsTest, TestLoadFromString) {
    Options opts("{\"picopter\" : {\"tpp\" : 414, \"tzp\" : \"alpha\"}}", true);
    ASSERT_EQ(414, opts.GetInt("tpp"));
    ASSERT_STREQ("alpha", opts.GetString("tzp"));
}

TEST_F(OptionsTest, TestLoadFromStringWrong) {
    Options opts("{\"picopter\" : {\"tpp\" : 414, \"tzp\" : \"alpha\"}}", false);
    ASSERT_EQ(0, opts.GetInt("tpp",0));
    ASSERT_STREQ("", opts.GetString("tzp",""));
}

TEST_F(OptionsTest, TestMerge) {
    Options opts("data/opts_data.txt");
    ASSERT_EQ(0, opts.GetInt("tpp", 0));
    ASSERT_STREQ("", opts.GetString("tzp", ""));
    ASSERT_TRUE(opts.Merge("{\"picopter\" : {\"tpp\" : 414, \"tzp\" : \"alpha\"}}"));
    ASSERT_EQ(414, opts.GetInt("tpp"));
    ASSERT_STREQ("alpha", opts.GetString("tzp"));
}

TEST_F(OptionsTest, TestSerialisation) {
    Options opts("data/opts_data.txt");
    std::string ser = opts.Serialise();
    Options par(ser.c_str(), true);
    
    ASSERT_STREQ("string_test\u00eeaaa", par.GetString("b"));
    ASSERT_DOUBLE_EQ(1.2433, par.GetReal("c"));
    ASSERT_EQ(20, par.GetInt("d"));
    ASSERT_TRUE(par.GetBool("e"));
    
    par.SetFamily("GPS");
    ASSERT_FALSE(par.GetBool("sane"));
    par.SetFamily("IMU");
    ASSERT_DOUBLE_EQ(204.44, par.GetReal("yaw"));
    ASSERT_DOUBLE_EQ(-17.78, par.GetReal("pitch"));
    ASSERT_DOUBLE_EQ(1093.33, par.GetReal("elevation"));
    par.SetFamily("");
    ASSERT_TRUE(par.GetBool("empty"));
}   