#include <eros/SafetyNode/SafetyNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace eros;

uint64_t heartbeat_count = 0;
void heartbeat_Callback(const eros::heartbeat& msg) {
    (void)msg;
    heartbeat_count++;
}
TEST(SafetyNode, TestSafetyNode) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_safetyNode");

    std::string heartbeat_topic = "/test/safety_node/heartbeat";
    ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    sleep(5.0);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());
    usleep(1.0 * 1000000.0);  // Wait for Master Node to Start.
    EXPECT_TRUE(heartbeat_count > 0);

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_safetyNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}