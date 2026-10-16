#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include <std_msgs/String.hxx>

#include "minibag/bag.h"
#include "minibag/query.h"
#include "minibag/unordered_view.h"
#include "minibag/view.h"

namespace {

struct Expected {
    miniros::Time time;
    std::string topic;
    std::string data;
};

struct TempBag {
    std::filesystem::path path;

    TempBag() {
        auto const stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = std::filesystem::temp_directory_path() /
               ("miniroscpp-unordered-view-" + std::to_string(stamp) + ".bag");
    }

    ~TempBag() {
        std::error_code ignore;
        std::filesystem::remove(path, ignore);
    }
};

std::vector<Expected> writeShuffled(std::string const& path, minibag::CompressionType compression,
                                    uint32_t chunk_threshold, int count) {
    std::vector<Expected> written;
    written.reserve(static_cast<size_t>(count));

    minibag::Bag bag;
    bag.open(path, minibag::bagmode::Write);
    bag.setCompression(compression);
    bag.setChunkThreshold(chunk_threshold);

    for (int i = 0; i < count; ++i) {
        // File order is the write order. Timestamps are deliberately not sorted,
        // and nsec stays unique so View's time order differs from file order.
        uint32_t const sec = static_cast<uint32_t>(1 + ((i * 3) % count));
        std_msgs::String msg;
        msg.data = "m" + std::to_string(i);
        std::string const topic = (i % 2 == 0) ? "even" : "odd";
        miniros::Time const time(sec, static_cast<uint32_t>(i));
        bag.write(topic, time, msg);
        written.push_back(Expected{time, topic, msg.data});
    }
    bag.close();
    return written;
}

std::vector<Expected> readUnordered(std::string const& path) {
    std::vector<Expected> got;
    minibag::UnorderedView view(path);
    for (minibag::MessageInstance const& m : view) {
        auto msg = m.instantiate<std_msgs::String>();
        EXPECT_TRUE(msg);
        if (!msg)
            continue;
        got.push_back(Expected{m.getTime(), m.getTopic(), msg->data});
    }
    return got;
}

void expectSame(std::vector<Expected> const& got, std::vector<Expected> const& want) {
    ASSERT_EQ(got.size(), want.size());
    for (size_t i = 0; i < got.size(); ++i) {
        EXPECT_EQ(got[i].time, want[i].time) << i;
        EXPECT_EQ(got[i].topic, want[i].topic) << i;
        EXPECT_EQ(got[i].data, want[i].data) << i;
    }
}

void roundTrip(minibag::CompressionType compression, uint32_t chunk_threshold, int count) {
    TempBag temp;
    std::vector<Expected> const written = writeShuffled(temp.path.string(), compression, chunk_threshold, count);

    expectSame(readUnordered(temp.path.string()), written);

    // The indexed view still returns timestamp order.
    std::vector<Expected> by_time = written;
    std::sort(by_time.begin(), by_time.end(), [](Expected const& a, Expected const& b) {
        if (a.time < b.time)
            return true;
        if (b.time < a.time)
            return false;
        return a.topic < b.topic;
    });
    minibag::Bag bag;
    bag.open(temp.path.string(), minibag::bagmode::Read);
    std::vector<Expected> indexed;
    for (minibag::MessageInstance const& m : minibag::View(bag)) {
        auto msg = m.instantiate<std_msgs::String>();
        ASSERT_TRUE(msg);
        indexed.push_back(Expected{m.getTime(), m.getTopic(), msg->data});
    }
    expectSame(indexed, by_time);

    // Scanning a bag that already loaded its index yields file order too.
    std::vector<Expected> from_open_bag;
    for (minibag::MessageInstance const& m : minibag::UnorderedView(bag)) {
        auto msg = m.instantiate<std_msgs::String>();
        ASSERT_TRUE(msg);
        from_open_bag.push_back(Expected{m.getTime(), m.getTopic(), msg->data});
    }
    expectSame(from_open_bag, written);
    bag.close();
}

} // namespace

TEST(UnorderedView, FileOrderAcrossChunks) {
    // One message per chunk, so an INDEX_DATA record sits between every chunk.
    roundTrip(minibag::compression::Uncompressed, 1, 40);
}

TEST(UnorderedView, FileOrderInsideOneChunk) {
    roundTrip(minibag::compression::Uncompressed, 1024 * 1024, 30);
}

TEST(UnorderedView, TopicAndTimeFilterKeepFileOrder) {
    TempBag temp;
    std::vector<Expected> const written =
        writeShuffled(temp.path.string(), minibag::compression::Uncompressed, 1, 20);

    std::vector<miniros::Time> even_times;
    for (Expected const& message : written) {
        if (message.topic == "even")
            even_times.push_back(message.time);
    }
    std::sort(even_times.begin(), even_times.end());
    miniros::Time const start = even_times[even_times.size() / 4];
    miniros::Time const end = even_times[(even_times.size() * 3) / 4];

    std::vector<Expected> want;
    for (Expected const& message : written) {
        if (message.topic != "even")
            continue;
        if (message.time < start || end < message.time)
            continue;
        want.push_back(message);
    }
    ASSERT_FALSE(want.empty());

    std::vector<Expected> got;
    minibag::UnorderedView view(temp.path.string(), minibag::TopicQuery("even"), start, end);
    for (minibag::MessageInstance const& m : view) {
        auto msg = m.instantiate<std_msgs::String>();
        ASSERT_TRUE(msg);
        got.push_back(Expected{m.getTime(), m.getTopic(), msg->data});
    }
    expectSame(got, want);
}

TEST(UnorderedView, RestartReadsFromTheBeginning) {
    TempBag temp;
    std::vector<Expected> const written =
        writeShuffled(temp.path.string(), minibag::compression::Uncompressed, 64, 10);

    minibag::UnorderedView view(temp.path.string());
    int seen = 0;
    for (minibag::MessageInstance const& m : view) {
        (void)m;
        if (++seen == 3)
            break;
    }
    ASSERT_EQ(seen, 3);

    std::vector<Expected> got;
    for (minibag::MessageInstance const& m : view) {
        auto msg = m.instantiate<std_msgs::String>();
        ASSERT_TRUE(msg);
        got.push_back(Expected{m.getTime(), m.getTopic(), msg->data});
    }
    expectSame(got, written);
}

#ifdef MINIBAG_HAS_LZ4
TEST(UnorderedView, Lz4Chunks) {
    roundTrip(minibag::compression::LZ4, 64, 20);
}
#endif

#ifdef MINIBAG_HAS_BZIP2
TEST(UnorderedView, Bz2Chunks) {
    roundTrip(minibag::compression::BZ2, 64, 12);
}
#endif

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
