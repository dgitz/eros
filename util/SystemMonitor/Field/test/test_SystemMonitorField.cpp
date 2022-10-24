/*! \file test_SystemMonitorField.cpp
 */
#include <eros/SystemMonitor/Field/BaseField.h>
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Field/IField.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
class TesterIField : public IField
{
   public:
    TesterIField() {
    }
    virtual ~TesterIField() {
    }
    RenderData getData() {
        return data;
    }
    void setData(RenderData newData) {
        data = newData;
    }

   private:
    RenderData data;
};
TEST(BasicTest, IFieldTest) {
    TesterIField field;
    RenderData data;
    field.setData(data);
    EXPECT_EQ(data.data, field.getData().data);
}
TEST(BasicTest, BaseFieldTest) {
    TesterIField field;
    RenderData data;
    field.setData(data);
    EXPECT_EQ(data.data, field.getData().data);
}
TEST(BasicTest, GenericFieldTest) {
    GenericField field;
    RenderData data;
    field.setData(data);
    EXPECT_EQ(data.data, field.getData().data);
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
