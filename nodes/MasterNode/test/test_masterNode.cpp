#include <eros/MasterNode/MasterNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace eros;
std::string robot_namespace = "/test";

uint64_t heartbeat_count = 0;
void heartbeat_Callback(const eros::heartbeat& msg) {
    (void)msg;
    heartbeat_count++;
}
TEST(MasterNode, TestMaster) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_masterNode");

    std::string heartbeat_topic = "/test/master_node/heartbeat";
    ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());

    ros::Publisher armedstate_pub =
        nh.advertise<eros::armed_state>(robot_namespace + "/ArmedState", 20);
    sleep(1.0);  // Wait for Master Node to Start.

    EXPECT_TRUE(heartbeat_count > 0);
    EXPECT_GT(armedstate_pub.getNumSubscribers(), 0);
    eros::armed_state armedState;
    armedstate_pub.publish(armedState);

    sleep(1.0);

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_masterNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}