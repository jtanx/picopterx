/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */
 
 #include "picopter.h"
 
 using namespace std;
 
 class TESTER {
    public:
    void test() { Log(LOG_WARNING, "HAHA"); }
 };
 
 int main(int argc, char *argv[]) {
    Log(LOG_NOTICE, "TEST");
    TESTER t;
    t.test();
    return 0;
 }