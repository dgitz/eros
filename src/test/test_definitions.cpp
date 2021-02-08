/*! \file test_definitions.cpp
 */
#include <eros/eROS_Definitions.h>
#include <gtest/gtest.h>
#include <stdio.h>
TEST(BasicTest, TestDefintions) {
    // Test Class: System
    {
        // Test Type: MainSystem
        for (uint8_t i = 1; i < (uint8_t)(System::MainSystem::END_OF_LIST); ++i) {
            EXPECT_FALSE(System::MainSystemString((System::MainSystem)(i)) == "UNKNOWN");
        }
        // Test Type: SubSystem
        for (uint8_t i = 1; i < (uint8_t)(System::SubSystem::END_OF_LIST); ++i) {
            EXPECT_FALSE(System::SubSystemString((System::SubSystem)(i)) == "UNKNOWN");
        }
        // Test Type: Component
        for (uint8_t i = 1; i < (uint8_t)(System::Component::END_OF_LIST); ++i) {
            EXPECT_FALSE(System::ComponentString((System::Component)(i)) == "UNKNOWN");
        }
    }

    // Test Class: Level
    {
        // Test Type: Type
        for (uint8_t i = 1; i < (uint8_t)(Level::Type::END_OF_LIST); ++i) {
            EXPECT_FALSE(Level::LevelString((Level::Type)(i)) == "UNKNOWN");
        }
    }

    // Test Class: ArmDisarm
    {
        // Test Type: Type
        for (uint8_t i = 1; i < (uint8_t)(ArmDisarm::Type::END_OF_LIST); ++i) {
            EXPECT_FALSE(ArmDisarm::ArmDisarmString((ArmDisarm::Type)(i)) == "UNKNOWN");
        }
    }

    // Test Class: Command
    {
        // Test Type: Type
        for (uint8_t i = 1; i < (uint8_t)(Command::Type::END_OF_LIST); ++i) {
            EXPECT_FALSE(Command::CommandString((Command::Type)(i)) == "UNKNOWN");
        }
    }

    // Test Class: Node
    {
        // Test Type: State
        for (uint8_t i = 1; i < (uint8_t)(Node::State::END_OF_LIST); ++i) {
            EXPECT_FALSE(Node::NodeStateString((Node::State)(i)) == "UNKNOWN");
        }
    }
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}