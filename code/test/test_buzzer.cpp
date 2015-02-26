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
    b.play(-100, 100, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestNegativeFrequency) {
    b.play(500, -100, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestNegativeVolume) {
    b.play(500, 100, -1);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestHighVolume) {
    b.play(500, 100, 200);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestConcurrentBuzzers) {
    Buzzer b2;
    b.play(500, 100, 100);
    b2.play(300, 400, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestStop) {
    b.play(500, 100, 100);
    sleep_for(milliseconds(100));
    b.stop();
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestHighFrequency) {
    b.play(500, 1200, 100);
    sleep_for(milliseconds(100));
}

TEST_F(BuzzerTest, TestBlocking) {
    b.playWait(500, 100, 100);
}
