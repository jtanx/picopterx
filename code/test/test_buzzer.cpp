#include "gtest/gtest.h"
#include "picopter.h"

using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using picopter::Buzzer;


TEST(BuzzerTest, TestNegativeDuration) {
    Buzzer b;
    b.play(-100, 100, 100);
    sleep_for(milliseconds(600));
}

TEST(BuzzerTest, TestNegativeFrequency) {
    Buzzer b;
    b.play(500, -100, 100);
    sleep_for(milliseconds(600));
}

TEST(BuzzerTest, TestNegativeVolume) {
    Buzzer b;
    b.play(500, 100, -1);
    sleep_for(milliseconds(600));
}

TEST(BuzzerTest, TestHighVolume) {
    Buzzer b;
    b.play(500, 100, 200);
    sleep_for(milliseconds(600));
}

TEST(BuzzerTest, TestConcurrentBuzzers) {
    Buzzer b1, b2;
    b1.play(500, 100, 100);
    b2.play(300, 400, 100);
    sleep_for(milliseconds(600));
}

TEST(BuzzerTest, TestStop) {
    Buzzer b;
    b.play(500, 100, 100);
    sleep_for(milliseconds(200));
    b.stop();
    sleep_for(milliseconds(400));
}