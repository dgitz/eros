#include <eros/SnapshotNode/SnapshotNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace eros;
uint64_t heartbeat_count = 0;
uint64_t commandstate_count = 0;
eros::command_state commandstate_msg;
void heartbeat_Callback(const eros::heartbeat& msg) {
    (void)msg;
    heartbeat_count++;
}
void commandstate_Callback(const eros::command_state& msg) {
    commandstate_count++;
    commandstate_msg = msg;
}
TEST(SnapshotNode, TestMaster) {
    int TEST_COUNT = 50;
    Logger* logger = new Logger("DEBUG", "test_SnapshotNode");
    for (int i = 0; i < TEST_COUNT; ++i) {
        logger->log_info("Test iteration: " + std::to_string(i) + "/" + std::to_string(TEST_COUNT));
        heartbeat_count = 0;
        commandstate_count = 0;
        ros::NodeHandle nh("~");

        std::string heartbeat_topic = "/test/snapshot_node/heartbeat";
        ros::Subscriber sub = nh.subscribe(heartbeat_topic, 100, &heartbeat_Callback);
        EXPECT_NE(ros::topic::waitForMessage<eros::heartbeat>(heartbeat_topic, ros::Duration(10)),
                  nullptr);
        EXPECT_EQ(1, sub.getNumPublishers());

        ros::Subscriber commandstate_sub =
            nh.subscribe("/test/SystemCommandState", 100, &commandstate_Callback);
        ros::Publisher command_pub = nh.advertise<eros::command>("/test/SystemCommand", 20);
        ros::ServiceClient client =
            nh.serviceClient<eros::srv_filetransfer>("/test/snapshot_node/srv_filetransfer");
        usleep(1.0 * 1000000.0);  // Wait for Snapshot Node to Start.
        EXPECT_TRUE(heartbeat_count > 0);

        eros::command snapshot_command;
        logger->log_notice("Testing Clearing of Snapshots.");
        snapshot_command.Command = (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT;
        snapshot_command.Option1 =
            (uint8_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS;
        command_pub.publish(snapshot_command);
        usleep(1.0 * 1000000.0);

        EXPECT_TRUE(commandstate_count > 0);
        EXPECT_TRUE(commandstate_msg.CurrentCommand.Command ==
                    (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT);
        EXPECT_TRUE(commandstate_msg.CurrentCommand.Option1 ==
                    (uint8_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS);
        EXPECT_TRUE(commandstate_msg.diag.DiagnosticMessage ==
                    (uint8_t)Diagnostic::Message::NOERROR);
        usleep(2.0 * 1000000.0);

        eros::srv_filetransfer req;
        req.request.path = "~/test/storage/SNAPSHOT/DEVICESNAPSHOT/";
        req.response.files.clear();
        EXPECT_EQ(client.call(req), true);
        EXPECT_TRUE(req.response.files.size() == 0);
        commandstate_count = 0;

        logger->log_notice("Testing Generation of Snapshot");
        snapshot_command.Command = (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT;
        snapshot_command.Option1 = (uint8_t)eros::Command::GenerateSnapshot_Option1::RUN_SLAVE;
        command_pub.publish(snapshot_command);
        usleep(5.0 * 1000000.0);
        EXPECT_TRUE(commandstate_count > 0);
        EXPECT_TRUE(commandstate_msg.CurrentCommand.Command ==
                    (uint8_t)eros::Command::Type::GENERATE_SNAPSHOT);
        EXPECT_TRUE(commandstate_msg.CurrentCommand.Option1 ==
                    (uint8_t)eros::Command::GenerateSnapshot_Option1::RUN_SLAVE);
        EXPECT_TRUE(commandstate_msg.diag.DiagnosticMessage ==
                    (uint8_t)Diagnostic::Message::NOERROR);
        usleep(5.0 * 1000000.0);

        req.response.files.clear();
        EXPECT_EQ(client.call(req), true);
        EXPECT_TRUE(req.response.files.size() == 1);
        for (std::size_t i = 0; i < req.response.files.size(); ++i) {
            EXPECT_TRUE(req.response.files.at(i).status ==
                        (uint8_t)FileHelper::FileStatus::FILE_OK);
            EXPECT_TRUE(req.response.files.at(i).data_length > 0);
            /*
            Don't unit test, but keep this for future examples.
            char arr[req.response.files.at(i).data_length];
            std::copy(req.response.files.at(i).data.begin(), req.response.files.at(i).data.end(),
            arr); FileHelper::FileInfo fileInfo = BaseNodeProcess::write_file(
                "~/test/zip/" + std::to_string(i) + ".zip", arr,
            req.response.files.at(i).data_length); EXPECT_TRUE(fileInfo.fileStatus ==
            FileHelper::FileStatus::FILE_OK);
            */
        }
    }
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_snapshotNode");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}