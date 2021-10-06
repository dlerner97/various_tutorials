#include "ToDoCore/ToDo.hpp"

#include <string>
    using std::string;

#include <gtest/gtest.h>
    using ::testing::Test;

namespace ToDoCore {
    namespace testing {
        class ToDoTest : public Test {
        private:
            /* data */
        protected:
            ToDoTest(/* args */);
            ~ToDoTest();

            virtual void SetUp() {}
            virtual void TearDown() {}

            ToDo list;

            static const size_t taskCount{3};
            static const string tasks[taskCount];
        };

        const string ToDoTest::tasks[taskCount] = {"write code", "compile", "test"};

        TEST_F(ToDoTest, constructor_createsEmptyList) {
            EXPECT_EQ(list.size(), size_t(0));
        }

        TEST_F(ToDoTest, addTask_threeTimes_sizeIsThree) {
            list.addTask(tasks[0]);
            list.addTask(tasks[1]);
            list.addTask(tasks[2]);
            EXPECT_EQ(list.size(), taskCount);
        }

        TEST_F(ToDoTest, getTask_withOneTask_returnsCorrectString) {
            list.addTask(tasks[0]);
            ASSERT_EQ(list.size(), size_t(1));
            EXPECT_EQ(list.getTask(0), tasks[0]);
        }

        TEST_F(ToDoTest, getTask_withThreeTasks_returnsCorrectStringForEachIndex) {
            list.addTask(tasks[0]);
            list.addTask(tasks[1]);
            list.addTask(tasks[2]);
            ASSERT_EQ(list.size(), taskCount);
            EXPECT_EQ(list.getTask(0), tasks[0]);
            EXPECT_EQ(list.getTask(1), tasks[1]);
            EXPECT_EQ(list.getTask(2), tasks[2]);
        }
    }
}

