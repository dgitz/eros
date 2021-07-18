#include <eros/DiagnosticNode/DiagnosticNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace eros;
uint64_t heartbeat_count = 0;
void heartbeat_Callback(const eros::heartbeat& msg) {
    (void)msg;
    heartbeat_count++;
}
TEST(DiagnosticNode, TestBasics) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_diagnosticNode");

    std::string heartbeat_topic = "/test/diagnostic_node/heartbeat";
    ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, sub.getNumPublishers());
    usleep(1.0 * 1000000.0);  // Wait for DiagnosticNode to Start.
    EXPECT_TRUE(heartbeat_count > 0);

    // Check Diagnostic Service
    std::string diagnostic_srv_topic = "/test/srv_system_diagnostics";
    ros::ServiceClient client = nh.serviceClient<eros::srv_get_diagnostics>(diagnostic_srv_topic);
    eros::srv_get_diagnostics srv_get_diagnostics;
    EXPECT_EQ(client.call(srv_get_diagnostics), true);

    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_diagnosticNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}