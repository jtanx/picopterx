#include "gtest/gtest.h"
#include "picopter.h"

using namespace picopter::navigation;

class NavigationTest : public ::testing::Test {
    protected:
        NavigationTest() {
            LogInit();
        }
};

TEST_F(NavigationTest, TestToRadians) {
    Coord2D c1 = {60, 180};
    Coord2D c2 = {45, 57};
    Coord3D c3 = {30, 120, 10};
    
    CoordInRadians(c1);
    CoordInRadians(c2);
    CoordInRadians(c3);
    
    ASSERT_DOUBLE_EQ(M_PI/3.0, c1.lat);
    ASSERT_DOUBLE_EQ(M_PI, c1.lon);
    ASSERT_DOUBLE_EQ(M_PI/4.0, c2.lat);
    ASSERT_DOUBLE_EQ(19.0*M_PI/60.0, c2.lon);
    ASSERT_DOUBLE_EQ(M_PI/6.0, c3.lat);
    ASSERT_DOUBLE_EQ(2.0*M_PI/3.0, c3.lon);
    ASSERT_DOUBLE_EQ(10, c3.alt);
}

TEST_F(NavigationTest, TestToDegrees) {
    Coord2D c1 = {M_PI/3.0, M_PI};
    Coord2D c2 = {M_PI/4, 19.0*M_PI/60.0};
    Coord3D c3 = {M_PI/6.0, 2.0*M_PI/3.0, 17};
    
    CoordInDegrees(c1);
    CoordInDegrees(c2);
    CoordInDegrees(c3);

    ASSERT_DOUBLE_EQ(60, c1.lat);
    ASSERT_DOUBLE_EQ(180, c1.lon);
    ASSERT_DOUBLE_EQ(45, c2.lat);
    ASSERT_DOUBLE_EQ(57, c2.lon);
    ASSERT_DOUBLE_EQ(30, c3.lat);
    ASSERT_DOUBLE_EQ(120, c3.lon);
    ASSERT_DOUBLE_EQ(17, c3.alt);
}

TEST_F(NavigationTest, TestInBoundsA) {
    Coord2D a = {-34, 116}; //Below Perth
    Coord2D b = {-30, 116}; //Above Perth
    Coord2D c = {-32, 114}; //To left of Perth
    Coord2D d = {-32, 118}; //To right of Perth
    Coord2D e = {-32, 116}; //Within Perth
    Coord2D f = {-31, 115}; //Upper left limit
    Coord2D g = {-33, 117}; //Bottom right limit
    
    ASSERT_FALSE(CoordInBounds(a, PERTH_BL, PERTH_TR));
    ASSERT_FALSE(CoordInBounds(b, PERTH_BL, PERTH_TR));
    ASSERT_FALSE(CoordInBounds(c, PERTH_BL, PERTH_TR));
    ASSERT_FALSE(CoordInBounds(d, PERTH_BL, PERTH_TR));
    
    ASSERT_TRUE(CoordInBounds(e, PERTH_BL, PERTH_TR));
    ASSERT_TRUE(CoordInBounds(f, PERTH_BL, PERTH_TR));
    ASSERT_TRUE(CoordInBounds(g, PERTH_BL, PERTH_TR));
}

//Test values cross-referenced from Python implementation
TEST_F(NavigationTest, TestCoordDistance) {
    Coord2D a = {-30, 150}, b = {-31, 150}, g = {-35, 151};

    ASSERT_DOUBLE_EQ(0.0, CoordDistance(a, a));
    ASSERT_DOUBLE_EQ(111089.56111761599, CoordDistance(a, b));
    ASSERT_DOUBLE_EQ(563283.2589389302, CoordDistance(a, g));
}

TEST_F(NavigationTest, TestCoordBearing) {
    Coord2D a = {-30, 150}, b = {-31, 150}, g = {-35, 151};
    
    ASSERT_DOUBLE_EQ(0.0, CoordBearing(a, a));
    ASSERT_DOUBLE_EQ(180, CoordBearing(a, b));
    ASSERT_DOUBLE_EQ(0.0, CoordBearing(b, a));
    ASSERT_DOUBLE_EQ(170.6912616092665, CoordBearing(a, g));
    ASSERT_DOUBLE_EQ(350.1534404199453, CoordBearing(g, a));
}