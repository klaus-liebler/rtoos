// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "rtos/log/log.hpp"

namespace {

namespace lg = rtos::log;

std::uint32_t g_now = 0;
std::uint32_t now_ms() noexcept { return g_now; }

using SmallJournal = lg::Journal<4, 16>;

struct Read {
    std::vector<lg::Record> records;
    std::vector<std::string> texts;
    std::uint32_t lost = 0;
};

Read read_all(SmallJournal& j, lg::Cursor& c) {
    Read r;
    lg::Record rec;
    std::array<char, 16> text{};
    std::uint32_t lost = 0;
    while (j.read(c, rec, text, lost)) {
        r.lost += lost;
        r.texts.emplace_back(rec.text);
        rec.text = {};
        r.records.push_back(rec);
    }
    r.lost += lost;
    return r;
}

class LogTest : public ::testing::Test {
protected:
    SmallJournal journal;
    void SetUp() override {
        g_now = 1234;
        lg::set_sink(journal.sink(&now_ms));
        lg::set_level(lg::Level::Info);
    }
    void TearDown() override { lg::set_sink({}); }
};

TEST_F(LogTest, FormatsIntoARecordWithPrefixData) {
    RTOS_LOG_WARN("x=%d %s", 42, "ok");
    lg::Cursor c{};
    const Read r = read_all(journal, c);
    ASSERT_EQ(r.records.size(), 1u);
    EXPECT_EQ(r.texts[0], "x=42 ok");
    EXPECT_EQ(r.records[0].level, lg::Level::Warn);
    EXPECT_EQ(r.records[0].time_ms, 1234u);
    EXPECT_STREQ(r.records[0].file, "log_test.cpp");
    EXPECT_FALSE(r.records[0].continuation);
}

TEST_F(LogTest, LevelFilterDropsBeforeFormatting) {
    RTOS_LOG_DEBUG("hidden");
    lg::set_level(lg::Level::Off);
    RTOS_LOG_FATAL("hidden too");
    EXPECT_EQ(journal.written(), 0u);
}

TEST_F(LogTest, WithoutSinkNothingHappens) {
    lg::set_sink({});
    RTOS_LOG_ERROR("nowhere");
    EXPECT_EQ(journal.written(), 0u);
}

TEST_F(LogTest, LongTextIsCutToTheRecord) {
    RTOS_LOG_INFO("%s", "0123456789abcdefghijklmnop");
    lg::Cursor c{};
    const Read r = read_all(journal, c);
    ASSERT_EQ(r.texts.size(), 1u);
    EXPECT_EQ(r.texts[0], "0123456789abcdef");
}

TEST_F(LogTest, SlowReaderLearnsWhatItLostAndWritersNeverWait) {
    for (int i = 0; i < 10; ++i) {
        RTOS_LOG_INFO("m%d", i);
    }
    lg::Cursor c{};
    const Read r = read_all(journal, c);
    EXPECT_EQ(r.lost, 6u);
    ASSERT_EQ(r.texts.size(), 4u);
    EXPECT_EQ(r.texts.front(), "m6");
    EXPECT_EQ(r.texts.back(), "m9");
    EXPECT_EQ(r.records.front().sequence, 6u);
}

TEST_F(LogTest, ReadersHaveIndependentPositions) {
    lg::Cursor fast{}, slow{};
    RTOS_LOG_INFO("a");
    EXPECT_EQ(read_all(journal, fast).texts.size(), 1u);
    RTOS_LOG_INFO("b");
    EXPECT_EQ(read_all(journal, fast).texts, std::vector<std::string>{"b"});
    EXPECT_EQ(read_all(journal, slow).texts, (std::vector<std::string>{"a", "b"}));
}

TEST_F(LogTest, OldestCursorStartsWithTheOldestRecordHeld) {
    for (int i = 0; i < 6; ++i) {
        RTOS_LOG_INFO("m%d", i);
    }
    lg::Cursor c = journal.oldest();
    const Read r = read_all(journal, c);
    EXPECT_EQ(r.lost, 0u);
    EXPECT_EQ(r.texts.front(), "m2");
}

TEST_F(LogTest, BlockLinesStayTogetherAndFollowAsContinuations) {
    {
        auto b = RTOS_LOG_BLOCK(Info);
        b.add("banner");
        b.add("line %d", 2);
        RTOS_LOG_INFO("before");  // lands before the whole block
        b.add("line %d", 3);
    }
    lg::Cursor c{};
    const Read r = read_all(journal, c);
    ASSERT_EQ(r.texts.size(), 4u);
    EXPECT_EQ(r.texts, (std::vector<std::string>{"before", "banner", "line 2", "line 3"}));
    EXPECT_FALSE(r.records[1].continuation);
    EXPECT_TRUE(r.records[2].continuation);
    EXPECT_TRUE(r.records[3].continuation);
}

TEST_F(LogTest, ListenerRunsAfterEveryAppend) {
    int calls = 0;
    journal.set_listener([](void* n) noexcept { ++*static_cast<int*>(n); }, &calls);
    RTOS_LOG_INFO("one");
    {
        auto b = RTOS_LOG_BLOCK(Info);
        b.add("x");
        b.add("y");
    }
    EXPECT_EQ(calls, 2);
}

TEST(LogFormatTest, ConsoleLinePrefixAndIndentation) {
    std::array<char, 96> out{};
    lg::Record r{.sequence = 0, .time_ms = 123456, .level = lg::Level::Error, .continuation = false,
                 .line = 7, .file = "main.cpp", .text = "boom"};
    std::size_t n = lg::format_line(r, out, false);
    EXPECT_EQ(std::string(out.data(), n), "   123 ERR main.cpp:007: boom\r\n");
    r.continuation = true;
    n = lg::format_line(r, out, false);
    EXPECT_EQ(std::string(out.data(), n), std::string(lg::prefix_width, ' ') + "boom\r\n");
    r.continuation = false;
    n = lg::format_line(r, std::span(out).first(10), false);
    EXPECT_EQ(n, 9u) << "cut to the buffer";
}

}  // namespace
