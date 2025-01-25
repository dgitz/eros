/*! \file test_diagnostics.cpp
 */
#include <eros/BaseNode.h>
#include <eros/BaseNodeProcess.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
static std::string get_hostname() {
    char name[1024];
    name[1023] = '\0';
    gethostname(name, 1023);
    return std::string(name);
}
class BaseNodeProcessTester : public BaseNodeProcess
{
   public:
    BaseNodeProcessTester() {
    }
    ~BaseNodeProcessTester() {
    }

    Diagnostic::DiagnosticDefinition finish_initialization() {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        return diag;
    }

    void reset() {
    }
    void cleanup() {
        base_cleanup();
    }

    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time) {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        diag = base_update(t_dt, t_ros_time);
        return diag;
    }
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg) {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        std::vector<Diagnostic::DiagnosticDefinition> diag_list;
        (void)msg;
        return diag_list;
    }
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables() {
        Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
        std::vector<Diagnostic::DiagnosticDefinition> diag_list;
        return diag_list;
    }
};
TEST(BasicTest, TestOperation_BaseNodeProcess) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    EXPECT_TRUE(tester->get_logger()->log_warn("A Log to Write") ==
                Logger::LoggerStatus::LOG_WRITTEN);

    json json_obj = tester->read_configuration(
        "", true, std::string(ZIPFILETESTDATA_DIR) + "/config/DeviceList.json");
    printf("host: %s\n", tester->get_hostname().c_str());
    EXPECT_TRUE(json_obj.size() > 0);
    printf("json:%s\n", json_obj.dump().c_str());
    EXPECT_TRUE(tester->update(0.0, 0.0).level <= Level::Type::NOTICE);
    tester->cleanup();
    delete logger;
    delete tester;
}
TEST(BasicTest, SupportFunctions) {
    {  // Test File Packaging
        FileHelper::FileInfo fileInfo;
        fileInfo = BaseNodeProcess::read_file("AFilethatwillNeverExist.zip");
        EXPECT_TRUE(fileInfo.byte_size == 0);
        EXPECT_TRUE(fileInfo.fileType == FileHelper::FileType::ZIP);
        EXPECT_TRUE(fileInfo.fileStatus == FileHelper::FileStatus::FILE_ERROR);

        fileInfo = BaseNodeProcess::read_file(std::string(ZIPFILETESTDATA_DIR) + "/zip/Zip1.zip");
        EXPECT_TRUE(fileInfo.byte_size > 0);
        EXPECT_TRUE(fileInfo.fileType == FileHelper::FileType::ZIP);
        EXPECT_TRUE(fileInfo.fileStatus == FileHelper::FileStatus::FILE_OK);
        EXPECT_TRUE(fileInfo.file_name.size() > 0);
        EXPECT_TRUE(fileInfo.folder.size() > 0);
        EXPECT_TRUE(fileInfo.full_path.size() > 0);

        fileInfo =
            BaseNodeProcess::write_file("~/test/Zip2.zip", fileInfo.data, fileInfo.byte_size);
        EXPECT_TRUE(fileInfo.fileStatus == FileHelper::FileStatus::FILE_OK);
    }
}
TEST(BasicTest, TestOperation_StateChange) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    // Sequence through States
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);
    EXPECT_FALSE(tester->request_statechange(Node::State::RUNNING));  // Not Allowed
    Diagnostic::DiagnosticDefinition diag = tester->update(0.0, 0.0);
    EXPECT_TRUE(diag.level <= Level::Type::NOTICE);

    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);
    EXPECT_FALSE(tester->request_statechange(Node::State::RUNNING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);

    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZED));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZED);
    EXPECT_FALSE(tester->request_statechange(Node::State::INITIALIZING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZED);

    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::PAUSED));
    EXPECT_FALSE(tester->request_statechange(Node::State::INITIALIZING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::PAUSED);

    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_FALSE(tester->request_statechange(Node::State::INITIALIZING));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::RESET));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RESET);
    EXPECT_FALSE(tester->request_statechange(Node::State::START));  // Not Allowed
    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);
    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZED));

    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);
    EXPECT_TRUE(tester->request_statechange(Node::State::RESET));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RESET);
    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::FINISHED));
    EXPECT_FALSE(tester->request_statechange(Node::State::START));  // Not Allowed
    EXPECT_EQ(tester->get_nodestate(), Node::State::FINISHED);

    delete logger;
    delete tester;
}
TEST(BasicTest, TestOperation_StateChange_Override) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    // Sequence through States
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);
    EXPECT_TRUE(tester->request_statechange(Node::State::RUNNING, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RUNNING);

    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZING, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZING);

    EXPECT_TRUE(tester->request_statechange(Node::State::PAUSED, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::PAUSED);

    EXPECT_TRUE(tester->request_statechange(Node::State::INITIALIZED, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::INITIALIZED);

    EXPECT_TRUE(tester->request_statechange(Node::State::RESET, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::RESET);

    EXPECT_TRUE(tester->request_statechange(Node::State::START, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);

    EXPECT_TRUE(tester->request_statechange(Node::State::FINISHED, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::FINISHED);

    EXPECT_TRUE(tester->request_statechange(Node::State::START, true));
    EXPECT_EQ(tester->get_nodestate(), Node::State::START);

    delete logger;
    delete tester;
}
TEST(TestConversion, ConvertTime) {
    {
        double t_in = 0.0;
        ros::Time t = BaseNodeProcess::convert_time(t_in);
        EXPECT_EQ(t.sec, 0);
        EXPECT_EQ(t.nsec, 0);
    }
    {
        struct timeval t_in;
        t_in.tv_sec = 0;
        t_in.tv_usec = 0;
        ros::Time t = BaseNodeProcess::convert_time(t_in);
        EXPECT_EQ(t.sec, 0);
        EXPECT_EQ(t.nsec, 0);
    }
}
TEST(TestConversion, ArmedState) {
    {
        ArmDisarm::State in;
        in.state = ArmDisarm::Type::ARMED;
        eros::armed_state out = BaseNodeProcess::convert(in);
        EXPECT_EQ((uint8_t)in.state, (uint8_t)out.armed_state);
    }
    {
        eros::armed_state in;
        in.armed_state = (uint8_t)ArmDisarm::Type::ARMED;
        ArmDisarm::State out = BaseNodeProcess::convert(in);
        EXPECT_EQ((uint8_t)in.armed_state, (uint8_t)out.state);
    }
}
TEST(TestConversion, Diagnostic) {
    {
        eros::diagnostic in;
        in.DeviceName = "a";
        in.NodeName = "b";
        in.System = (uint8_t)System::MainSystem::SIMROVER;
        in.SubSystem = (uint8_t)System::SubSystem::ROBOT_MONITOR;
        in.Component = (uint8_t)System::Component::DIAGNOSTIC;
        in.DiagnosticType = (uint8_t)Diagnostic::DiagnosticType::SOFTWARE;
        in.DiagnosticMessage = (uint8_t)Diagnostic::Message::INITIALIZING;
        in.Level = (uint8_t)Level::Type::INFO;
        in.Description = "c";
        Diagnostic::DiagnosticDefinition out = BaseNodeProcess::convert(in);
        EXPECT_EQ(in.DeviceName, out.device_name);
        EXPECT_EQ(in.NodeName, out.node_name);
        EXPECT_EQ((uint8_t)in.System, (uint8_t)out.system);
        EXPECT_EQ((uint8_t)in.SubSystem, (uint8_t)out.subsystem);
        EXPECT_EQ((uint8_t)in.Component, (uint8_t)out.component);
        EXPECT_EQ((uint8_t)in.DiagnosticType, (uint8_t)out.type);
        EXPECT_EQ((uint8_t)in.DiagnosticMessage, (uint8_t)out.message);
        EXPECT_EQ((uint8_t)in.Level, (uint8_t)out.level);
        EXPECT_EQ(in.Description, out.description);
    }
    {
        Diagnostic::DiagnosticDefinition in;
        in.device_name = "a";
        in.node_name = "b";
        in.system = System::MainSystem::SIMROVER;
        in.subsystem = System::SubSystem::ROBOT_MONITOR;
        in.component = System::Component::DIAGNOSTIC;
        in.type = Diagnostic::DiagnosticType::SOFTWARE;
        in.message = Diagnostic::Message::INITIALIZING;
        in.level = Level::Type::INFO;
        in.description = "c";
        eros::diagnostic out = BaseNodeProcess::convert(in);
        EXPECT_EQ(out.DeviceName, in.device_name);
        EXPECT_EQ(out.NodeName, in.node_name);
        EXPECT_EQ((uint8_t)out.System, (uint8_t)in.system);
        EXPECT_EQ((uint8_t)out.SubSystem, (uint8_t)in.subsystem);
        EXPECT_EQ((uint8_t)out.Component, (uint8_t)in.component);
        EXPECT_EQ((uint8_t)out.DiagnosticType, (uint8_t)in.type);
        EXPECT_EQ((uint8_t)out.DiagnosticMessage, (uint8_t)in.message);
        EXPECT_EQ((uint8_t)out.Level, (uint8_t)in.level);
        EXPECT_EQ(out.Description, in.description);
    }
}
TEST(ConfigurationTests, ConfigurationTestCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    {  // Failure Tests
        logger->log_warn("Testing Failure Cases...");
        std::string FilePathOfZeroLength = "";
        json empty;
        json obj;
        obj = tester->read_configuration("abcd", true, FilePathOfZeroLength);
        EXPECT_EQ(obj, empty);

        std::string AFilePathThatWillNeverExist = "/AFilePathThatWillNeverExist";
        obj = tester->read_configuration("abcd", true, AFilePathThatWillNeverExist);
        EXPECT_EQ(obj, empty);
    }
    {
        std::string ConfigPath = std::string(getenv("HOME")) + "/config/DeviceList.json";
        json empty;
        json obj;

        obj = tester->read_configuration(tester->get_hostname(), true, ConfigPath);
        EXPECT_NE(obj, empty);
    }
    delete tester;
    delete logger;
}
TEST(FileReadTests, FileReadTestCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    logger->log_warn("Testing Failure Cases...");
    {
        // Failure Tests: Unsupported File Types
        std::string ConfigPath =
            std::string(getenv("HOME")) +
            "/config/DeviceList.json";  // File Exists, but this function isn't intended to support.
        FileHelper::FileInfo info = tester->read_file(ConfigPath);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::UNKNOWN);
    }
    {
        // Failure Tests: Corrupted Files
        std::string filePath = "EmptyZip.zip";
        std::string cmd = "touch " + filePath;
        ExecResult res = exec(cmd.c_str(), true);
        EXPECT_FALSE(res.AnyError);
        FileHelper::FileInfo info = tester->read_file(filePath);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::ZIP);
        cmd = "rm " + filePath;
        res = exec(cmd.c_str(), true);
        EXPECT_FALSE(res.AnyError);
    }
    delete tester;
    delete logger;
}
TEST(FileWriteTests, FileWriteTestCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    logger->log_warn("Testing Failure Cases...");
    {
        // Failure Tests: Unsupported File Types
        std::string validFile = "TestFile.txt";
        char someData[7] = "Hello";
        FileHelper::FileInfo info = tester->write_file(validFile, someData, 1);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::UNKNOWN);

        std::string inValidFile = "/AFileThatShouldNeverExist.zip";
        info = tester->write_file(inValidFile, someData, 1);
        EXPECT_EQ(info.fileStatus, FileHelper::FileStatus::FILE_ERROR);
        EXPECT_EQ(info.fileType, FileHelper::FileType::ZIP);
    }

    delete tester;
    delete logger;
}
TEST(DirectoryReadTests, DirectoryReadTestsCases) {
    Logger* logger = new Logger("DEBUG", "UnitTestBaseNodeProcess");
    BaseNodeProcessTester* tester = new BaseNodeProcessTester;
    tester->initialize("UnitTestBaseNodeProcess",
                       "UnitTestBaseNodeProcessInstance",
                       get_hostname(),
                       System::MainSystem::SIMROVER,
                       System::SubSystem::ENTIRE_SYSTEM,
                       System::Component::ENTIRE_SUBSYSTEM,
                       logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    tester->enable_diagnostics(diagnostic_types);
    {
        std::vector<std::string> fileList = tester->get_files_indir(std::string(getenv("HOME")));
        EXPECT_GT(fileList.size(), 0);
    }

    delete tester;
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}