/**
 * @file picopter.cpp
 * @brief The main entry point to the server.
 */
 
 #include "picopter.h"
 
 using namespace std;
 
 /**
  * @brief A test class.
  */
 class TESTER {
    public:
    
    /**
     * @brief A test method
     * IT DOES NOTHING
     */
    void test() { Log(LOG_WARNING, "HAHA"); }
 };
 
 int main(int argc, char *argv[]) {
    Log(LOG_NOTICE, "TEST");
    TESTER t;
    t.test();
    return 0;
 }