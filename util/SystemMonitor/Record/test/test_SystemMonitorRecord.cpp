/*! \file test_SystemMonitorRecord.cpp
 */
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/BaseRecord.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Record/IRecord.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
class TesterIRecord : public IRecord
{
   public:
    TesterIRecord() {
    }
    virtual ~TesterIRecord() {
    }
    std::vector<std::shared_ptr<IField>> getFields() {
        return fields;
    }
    void setFields(std::vector<std::shared_ptr<IField>> newFields) {
        fields = newFields;
    }

   private:
    std::vector<std::shared_ptr<IField>> fields;
};
TEST(BasicTest, IRecordTest) {
    TesterIRecord record;
    std::vector<std::shared_ptr<IField>> fields;
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "1";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "2";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "3";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    record.setFields(fields);
    EXPECT_EQ(fields.size(), record.getFields().size());
    EXPECT_GT(fields.size(), 0);
    uint16_t counter = 1;
    for (auto field : record.getFields()) {
        uint16_t v = std::stoi(field->getData().data);
        EXPECT_EQ(v, counter);
        counter++;
    }
}
TEST(BasicTest, BaseRecordTest) {
    BaseRecord record;
    std::vector<std::shared_ptr<IField>> fields;
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "1";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "2";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "3";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    record.setFields(fields);
    EXPECT_EQ(fields.size(), record.getFields().size());
    EXPECT_GT(fields.size(), 0);
    uint16_t counter = 1;
    for (auto field : record.getFields()) {
        uint16_t v = std::stoi(field->getData().data);
        EXPECT_EQ(v, counter);
        counter++;
    }
}
TEST(BasicTest, GenericRecordTest) {
    GenericRecord record;
    std::vector<std::shared_ptr<IField>> fields;
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "1";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "2";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "3";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    record.setFields(fields);
    EXPECT_EQ(fields.size(), record.getFields().size());
    EXPECT_GT(fields.size(), 0);
    uint16_t counter = 1;
    for (auto field : record.getFields()) {
        uint16_t v = std::stoi(field->getData().data);
        EXPECT_EQ(v, counter);
        counter++;
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
