#include "gtest/gtest.h"
#include "picopter.h"

using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using picopter::Buzzer;

class BuzzerTest : public ::testing::Test {
    protected:
        BuzzerTest() {
            LogInit();
        }
        
        Buzzer b;
};

TEST_F(BuzzerTest, TestNegativeDuration) {
    b.Play(-100, 100, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestNegativeFrequency) {
    b.Play(500, -100, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestNegativeVolume) {
    b.Play(500, 100, -1);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestHighVolume) {
    b.Play(500, 100, 200);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestConcurrentBuzzers) {
    Buzzer b2;
    b.Play(500, 100, 100);
    b2.Play(300, 400, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestStop) {
    b.Play(500, 100, 100);
    sleep_for(milliseconds(100));
    b.Stop();
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestHighFrequency) {
    b.Play(500, 1200, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestBlocking) {
    b.PlayWait(500, 100, 100);
}
