#include <eros/MasterNode/MasterNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace eros;
std::string robot_namespace = "/test/";
std::string unittest_nodename = "master_node";
uint64_t heartbeat_count = 0;
eros::heartbeat latest_hearbeat;
void heartbeat_Callback(const eros::heartbeat& msg) {
    latest_hearbeat = msg;
    heartbeat_count++;
}
uint64_t diagnostic_count = 0;
eros::diagnostic latest_diagnostic;
void diagnostic_Callback(const eros::diagnostic& msg) {
    latest_diagnostic = msg;
    diagnostic_count++;
}
TEST(MasterNode, TestMaster) {
    ros::NodeHandle nh("~");
    Logger* logger = new Logger("DEBUG", "test_masterNode");

    std::string heartbeat_topic = robot_namespace + unittest_nodename + "/heartbeat";
    std::string diagnostic_topic = robot_namespace + unittest_nodename + "/diagnostic";
    ros::Subscriber heartbeat_sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
    ros::Subscriber diagnostic_sub = nh.subscribe(diagnostic_topic, 100, &diagnostic_Callback);
    sleep(10.0);
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    for (int i = 0; i < master_topics.size(); ++i) {
        printf("%d topic: %s\n", master_topics.at(i).name.c_str());
    }
    ros::V_string master_nodes;
    ros::master::getNodes(master_nodes);
    for (int i = 0; i < master_nodes.size(); ++i) {
        printf("%d node: %s\n", master_nodes.at(i).c_str());
    }
    EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, heartbeat_sub.getNumPublishers());
    EXPECT_NE(ros::topic::waitForMessage<eros::diagnostic>(diagnostic_topic, ros::Duration(10)),
              nullptr);
    EXPECT_EQ(1, diagnostic_sub.getNumPublishers());

    ros::Publisher armedstate_pub =
        nh.advertise<eros::armed_state>(robot_namespace + "/ArmedState", 20);

    ros::Publisher modestate_pub =
        nh.advertise<eros::mode_state>(robot_namespace + "/ModeState", 20);
    sleep(1.0);  // Wait for Master Node to Start.

    EXPECT_GT(heartbeat_count, 0);
    EXPECT_GT(diagnostic_count, 0);
    EXPECT_GT(armedstate_pub.getNumSubscribers(), 0);
    EXPECT_GT(modestate_pub.getNumSubscribers(), 0);
    eros::armed_state armedState;
    armedstate_pub.publish(armedState);

    eros::mode_state modeState;
    modestate_pub.publish(modeState);
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_firmware>(
            robot_namespace + unittest_nodename + "/srv_firmware");
        eros::srv_firmware req;
        EXPECT_TRUE(client.call(req));
        EXPECT_EQ(req.response.BaseNodeName, unittest_nodename);
    }
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_logger_level>(
            robot_namespace + unittest_nodename + "/srv_loggerlevel");
        eros::srv_logger_level req;
        req.request.LoggerLevel = "UNKNOWN";
        logger->log_warn("Checking Failure Case...");
        EXPECT_FALSE(client.call(req));
        req.request.LoggerLevel = "DEBUG";
        EXPECT_TRUE(client.call(req));
        req.request.LoggerLevel = "NOTICE";
        EXPECT_TRUE(client.call(req));
    }
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_get_diagnostics>(
            robot_namespace + unittest_nodename + "/srv_diagnostics");
        eros::srv_get_diagnostics req;
        req.request.MinLevel = 0;        // All
        req.request.DiagnosticType = 0;  // All
        EXPECT_TRUE(client.call(req));
        EXPECT_LT(req.response.worst_diag.Level, (uint8_t)Level::Type::WARN);
        for (auto diag : req.response.diag_list) {
            EXPECT_LT(diag.Level, (uint8_t)Level::Type::WARN);
        }
    }
    {
        ros::ServiceClient client = nh.serviceClient<eros::srv_change_nodestate>(
            robot_namespace + unittest_nodename + "/srv_nodestate_change");
        eros::srv_change_nodestate req;
        req.request.RequestedNodeState = "RESET";
        EXPECT_TRUE(client.call(req));
    }
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