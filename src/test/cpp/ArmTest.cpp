#include <gtest/gtest.h>
#include <iostream>
#include "Arm.h"

class ArmTest : public testing::Test {

};

TEST_F(ArmTest, Something) {
    std::cout << "testing" << std::endl ;
    Arm arm;
    arm.Periodic();
    //EXPECT_EQ(1, 2);
}