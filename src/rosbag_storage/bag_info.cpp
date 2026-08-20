#include "minibag/bag_info.h"

#include "minibag/bag.h"
#include "minibag/constants.h"
#include "minibag/structures.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace minibag {

struct InfoRow
{
    std::string field;
    std::string value;
};

struct DatatypeInfo
{
    std::string datatype;
    std::string md5sum;
};

struct TopicInfo
{
    std::string datatype;
    uint64_t message_count = 0;
    uint32_t connection_count = 0;
    bool has_frequency = false;
    double frequency = 0.0;
};

class BagInfoFormatter
{
public:
    static std::string format(Bag const& bag, BagInfoOptions const& opts);
};

namespace {

std::string humanReadableSize(double size)
{
    const double multiple = 1024.0;
    const char* suffixes[] = {"KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB"};
    for (const char* suffix : suffixes) {
        size /= multiple;
        if (size < multiple) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << size << ' ' << suffix;
            return oss.str();
        }
    }
    return "-";
}

std::string humanReadableFrequency(double freq)
{
    const double multiple = 1000.0;
    const char* suffixes[] = {"Hz", "kHz", "MHz", "GHz", "THz", "PHz", "EHz", "ZHz", "YHz"};
    for (const char* suffix : suffixes) {
        if (freq < multiple) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << freq << ' ' << suffix;
            return oss.str();
        }
        freq /= multiple;
    }
    return "-";
}

std::string timeToStr(double secs)
{
    const double whole_secs = std::floor(secs);
    const double frac = secs - whole_secs;

    std::time_t tt = static_cast<std::time_t>(whole_secs);
    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &tt);
#else
    localtime_r(&tt, &tm_buf);
#endif

    // Match rosbag: strftime(...%H:%M:%S) + ("%.2f" % frac)[1:]  →  "...42.02"
    std::ostringstream frac_oss;
    frac_oss << std::fixed << std::setprecision(2) << frac;
    const std::string frac_str = frac_oss.str();  // e.g. "0.02"

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%b %d %Y %H:%M:%S");
    if (frac_str.size() > 1)
        oss << frac_str.substr(1);  // ".02"
    return oss.str();
}

std::string formatDuration(double duration)
{
    const int dur_secs = static_cast<int>(duration) % 60;
    int dur_mins = static_cast<int>(duration / 60.0);
    const int dur_hrs = dur_mins / 60;
    if (dur_hrs > 0) {
        dur_mins %= 60;
        std::ostringstream oss;
        oss << dur_hrs << "hr " << dur_mins << ':' << std::setfill('0') << std::setw(2) << dur_secs
            << "s (" << static_cast<int>(duration) << "s)";
        return oss.str();
    }
    if (dur_mins > 0) {
        std::ostringstream oss;
        oss << dur_mins << ':' << std::setfill('0') << std::setw(2) << dur_secs
            << "s (" << static_cast<int>(duration) << "s)";
        return oss.str();
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << duration << 's';
    return oss.str();
}

double median(std::vector<double> values)
{
    if (values.empty())
        return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    if (n % 2 == 1)
        return values[n / 2];
    return (values[n / 2 - 1] + values[n / 2]) / 2.0;
}

std::vector<ConnectionInfo const*> getConnectionsForTopic(
    std::map<uint32_t, ConnectionInfo*> const& connections, std::string const& topic)
{
    std::vector<ConnectionInfo const*> result;
    for (auto const& entry : connections) {
        if (entry.second->topic == topic)
            result.push_back(entry.second);
    }
    return result;
}

uint64_t countMessages(
    ConnectionInfo const* connection,
    std::vector<ChunkInfo> const& chunks,
    std::map<uint32_t, std::multiset<IndexEntry>> const& connection_indexes)
{
    uint64_t count = 0;
    if (!chunks.empty()) {
        for (ChunkInfo const& chunk : chunks) {
            auto it = chunk.connection_counts.find(connection->id);
            if (it != chunk.connection_counts.end())
                count += it->second;
        }
    } else {
        auto it = connection_indexes.find(connection->id);
        if (it != connection_indexes.end())
            count = it->second.size();
    }
    return count;
}

std::vector<double> collectTimestamps(
    std::vector<ConnectionInfo const*> const& connections,
    std::map<uint32_t, std::multiset<IndexEntry>> const& connection_indexes)
{
    std::vector<double> stamps;
    for (ConnectionInfo const* connection : connections) {
        auto it = connection_indexes.find(connection->id);
        if (it == connection_indexes.end())
            continue;
        for (IndexEntry const& entry : it->second)
            stamps.push_back(entry.time.toSec());
    }
    std::sort(stamps.begin(), stamps.end());
    return stamps;
}

double medianFrequency(std::vector<double> const& stamps)
{
    if (stamps.size() < 2)
        return std::numeric_limits<double>::quiet_NaN();
    std::vector<double> periods;
    periods.reserve(stamps.size() - 1);
    for (size_t i = 1; i < stamps.size(); ++i)
        periods.push_back(stamps[i] - stamps[i - 1]);
    const double med_period = median(periods);
    if (med_period <= 0.0)
        return std::numeric_limits<double>::quiet_NaN();
    return 1.0 / med_period;
}

void appendCompressionRows(
    std::vector<InfoRow>& rows,
    std::vector<ChunkHeader> const& chunk_headers,
    double duration)
{
    if (chunk_headers.empty()) {
        rows.push_back({"compression", "none"});
        return;
    }

    std::map<std::string, uint32_t> compression_counts;
    std::map<std::string, uint64_t> compression_uncompressed;
    std::map<std::string, uint64_t> compression_compressed;
    for (ChunkHeader const& chunk_header : chunk_headers) {
        compression_counts[chunk_header.compression]++;
        compression_uncompressed[chunk_header.compression] += chunk_header.uncompressed_size;
        compression_compressed[chunk_header.compression] += chunk_header.compressed_size;
    }

    const uint32_t chunk_count = static_cast<uint32_t>(chunk_headers.size());
    std::vector<std::pair<uint32_t, std::string>> sorted_counts;
    for (auto const& entry : compression_counts)
        sorted_counts.emplace_back(entry.second, entry.first);
    std::sort(sorted_counts.begin(), sorted_counts.end());

    std::ostringstream compression_line;
    bool first = true;
    for (auto it = sorted_counts.rbegin(); it != sorted_counts.rend(); ++it) {
        if (!first)
            compression_line << ", ";
        first = false;
        const std::string& compression = it->second;
        const uint32_t count = it->first;
        if (compression != COMPRESSION_NONE) {
            const double fraction = 100.0 * static_cast<double>(compression_compressed[compression])
                / static_cast<double>(compression_uncompressed[compression]);
            compression_line << compression << " [" << count << '/' << chunk_count << " chunks; "
                             << std::fixed << std::setprecision(2) << fraction << "%]";
        } else {
            compression_line << compression << " [" << count << '/' << chunk_count << " chunks]";
        }
    }
    rows.push_back({"compression", compression_line.str()});

    bool all_uncompressed = true;
    for (auto const& entry : compression_counts) {
        if (entry.first != COMPRESSION_NONE) {
            all_uncompressed = false;
            break;
        }
    }

    if (!all_uncompressed) {
        uint64_t total_uncompressed_size = 0;
        uint64_t total_compressed_size = 0;
        for (ChunkHeader const& chunk_header : chunk_headers) {
            total_uncompressed_size += chunk_header.uncompressed_size;
            total_compressed_size += chunk_header.compressed_size;
        }

        const std::string total_uncompressed_size_str = humanReadableSize(static_cast<double>(total_uncompressed_size));
        const std::string total_compressed_size_str = humanReadableSize(static_cast<double>(total_compressed_size));
        const size_t total_size_str_length = std::max(total_uncompressed_size_str.size(), total_compressed_size_str.size());

        if (duration > 0.0) {
            const std::string uncompressed_rate_str = humanReadableSize(total_uncompressed_size / duration);
            const std::string compressed_rate_str = humanReadableSize(total_compressed_size / duration);
            const size_t rate_str_length = std::max(uncompressed_rate_str.size(), compressed_rate_str.size());

            std::ostringstream uncompressed;
            uncompressed << std::setw(static_cast<int>(total_size_str_length)) << total_uncompressed_size_str
                         << " @ " << std::setw(static_cast<int>(rate_str_length)) << uncompressed_rate_str << "/s";
            rows.push_back({"uncompressed", uncompressed.str()});

            std::ostringstream compressed;
            compressed << std::setw(static_cast<int>(total_size_str_length)) << total_compressed_size_str
                       << " @ " << std::setw(static_cast<int>(rate_str_length)) << compressed_rate_str << "/s ("
                       << std::fixed << std::setprecision(2)
                       << (100.0 * total_compressed_size) / total_uncompressed_size << "%)";
            rows.push_back({"compressed", compressed.str()});
        } else {
            rows.push_back({"uncompressed", total_uncompressed_size_str});
            rows.push_back({"compressed", total_compressed_size_str});
        }
    }
}

std::vector<DatatypeInfo> collectDatatypes(std::map<uint32_t, ConnectionInfo*> const& connections)
{
    std::set<std::string> seen;
    std::vector<DatatypeInfo> datatypes;
    for (auto const& entry : connections) {
        ConnectionInfo const* connection = entry.second;
        if (!seen.insert(connection->datatype).second)
            continue;
        datatypes.push_back({connection->datatype, connection->md5sum});
    }
    std::sort(datatypes.begin(), datatypes.end(), [](DatatypeInfo const& a, DatatypeInfo const& b) {
        return a.datatype < b.datatype;
    });
    return datatypes;
}

std::map<std::string, TopicInfo> collectTopics(
    std::map<uint32_t, ConnectionInfo*> const& connections,
    std::vector<ChunkInfo> const& chunks,
    std::map<uint32_t, std::multiset<IndexEntry>> const& connection_indexes,
    bool compute_frequency)
{
    std::set<std::string> topics;
    for (auto const& entry : connections)
        topics.insert(entry.second->topic);

    std::map<std::string, TopicInfo> topic_infos;
    for (std::string const& topic : topics) {
        std::vector<ConnectionInfo const*> topic_connections = getConnectionsForTopic(connections, topic);
        if (topic_connections.empty())
            continue;

        TopicInfo info;
        info.datatype = topic_connections.front()->datatype;
        info.connection_count = static_cast<uint32_t>(topic_connections.size());
        for (ConnectionInfo const* connection : topic_connections)
            info.message_count += countMessages(connection, chunks, connection_indexes);

        if (compute_frequency) {
            const double freq = medianFrequency(collectTimestamps(topic_connections, connection_indexes));
            if (!std::isnan(freq)) {
                info.has_frequency = true;
                info.frequency = freq;
            }
        }

        topic_infos.emplace(topic, info);
    }
    return topic_infos;
}

std::string formatTextRows(std::vector<InfoRow> const& rows)
{
    size_t first_column_width = 0;
    for (InfoRow const& row : rows) {
        if (!row.field.empty())
            first_column_width = std::max(first_column_width, row.field.size());
    }
    first_column_width += 1;

    std::ostringstream oss;
    for (InfoRow const& row : rows) {
        if (!row.field.empty())
            oss << std::setw(static_cast<int>(first_column_width)) << std::left << (row.field + ":") << ' ' << row.value << '\n';
        else
            oss << std::setw(static_cast<int>(first_column_width)) << std::left << "" << ' ' << row.value << '\n';
    }
    std::string result = oss.str();
    if (!result.empty() && result.back() == '\n')
        result.pop_back();
    return result;
}

std::string formatBagInfoText(
    Bag const& bag,
    double start_stamp,
    double end_stamp,
    uint64_t num_messages,
    std::vector<DatatypeInfo> const& datatypes,
    std::map<std::string, TopicInfo> const& topic_infos)
{
    std::vector<InfoRow> rows;
    if (!bag.getFileName().empty())
        rows.push_back({"path", bag.getFileName()});
    rows.push_back({"version", std::to_string(bag.getMajorVersion()) + '.' + std::to_string(bag.getMinorVersion())});

    const double duration = end_stamp - start_stamp;
    rows.push_back({"duration", formatDuration(duration)});
    {
        std::ostringstream start;
        start << timeToStr(start_stamp) << " (" << std::fixed << std::setprecision(2) << start_stamp << ')';
        rows.push_back({"start", start.str()});
    }
    {
        std::ostringstream end;
        end << timeToStr(end_stamp) << " (" << std::fixed << std::setprecision(2) << end_stamp << ')';
        rows.push_back({"end", end.str()});
    }
    rows.push_back({"size", humanReadableSize(static_cast<double>(bag.getSize()))});
    rows.push_back({"messages", std::to_string(num_messages)});

    appendCompressionRows(rows, bag.collectChunkHeaders(), duration);

    size_t max_datatype_len = 0;
    for (DatatypeInfo const& datatype : datatypes)
        max_datatype_len = std::max(max_datatype_len, datatype.datatype.size());

    for (size_t i = 0; i < datatypes.size(); ++i) {
        std::ostringstream line;
        line << std::setw(static_cast<int>(max_datatype_len)) << std::left << datatypes[i].datatype
             << " [" << datatypes[i].md5sum << ']';
        rows.push_back({i == 0 ? "types" : "", line.str()});
    }

    size_t max_topic_len = 0;
    size_t max_msg_count_len = 0;
    size_t max_freq_len = 0;
    for (auto const& entry : topic_infos) {
        max_topic_len = std::max(max_topic_len, entry.first.size());
        max_msg_count_len = std::max(max_msg_count_len, std::to_string(entry.second.message_count).size());
        if (entry.second.has_frequency)
            max_freq_len = std::max(max_freq_len, humanReadableFrequency(entry.second.frequency).size());
    }

    size_t topic_index = 0;
    for (auto const& entry : topic_infos) {
        std::ostringstream line;
        line << std::setw(static_cast<int>(max_topic_len)) << std::left << entry.first << "   "
             << std::setw(static_cast<int>(max_msg_count_len)) << std::right << entry.second.message_count << ' '
             << (entry.second.message_count == 1 ? "msg " : "msgs");
        // Always reserve the frequency column width (may be 0), matching rosbag spacing.
        if (entry.second.has_frequency)
            line << " @ " << std::setw(static_cast<int>(max_freq_len)) << std::right
                 << humanReadableFrequency(entry.second.frequency);
        else
            line << "   " << std::setw(static_cast<int>(max_freq_len)) << std::right << "";
        line << " : " << std::setw(static_cast<int>(max_datatype_len)) << std::left << entry.second.datatype;
        if (entry.second.connection_count > 1)
            line << " (" << entry.second.connection_count << " connections)";
        rows.push_back({topic_index == 0 ? "topics" : "", line.str()});
        ++topic_index;
    }

    return formatTextRows(rows);
}

std::string formatBagInfoYaml(
    Bag const& bag,
    double start_stamp,
    double end_stamp,
    uint64_t num_messages,
    std::vector<DatatypeInfo> const& datatypes,
    std::map<std::string, TopicInfo> const& topic_infos,
    bool compute_frequency,
    std::string const& key)
{
    auto finish = [](std::string result) {
        if (!result.empty() && result.back() == '\n')
            result.pop_back();
        return result;
    };

    // Scalar keys: rosbag prints the bare value (no "key: " prefix).
    if (key == "path")
        return bag.getFileName();
    if (key == "version")
        return std::to_string(bag.getMajorVersion()) + '.' + std::to_string(bag.getMinorVersion());
    if (key == "duration") {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << (end_stamp - start_stamp);
        return oss.str();
    }
    if (key == "start") {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << start_stamp;
        return oss.str();
    }
    if (key == "end") {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << end_stamp;
        return oss.str();
    }
    if (key == "size")
        return std::to_string(bag.getSize());
    if (key == "messages")
        return std::to_string(num_messages);
    if (key == "indexed")
        return "True";

    if (key == "compression" || key == "uncompressed" || key == "compressed") {
        std::vector<ChunkHeader> chunk_headers = bag.collectChunkHeaders();
        if (chunk_headers.empty()) {
            if (key == "compression")
                return "none";
            return {};
        }

        std::map<std::string, uint32_t> compression_counts;
        uint64_t total_uncompressed_size = 0;
        uint64_t total_compressed_size = 0;
        for (ChunkHeader const& chunk_header : chunk_headers) {
            compression_counts[chunk_header.compression]++;
            total_uncompressed_size += chunk_header.uncompressed_size;
            total_compressed_size += chunk_header.compressed_size;
        }

        if (key == "compression") {
            std::vector<std::pair<uint32_t, std::string>> sorted_counts;
            for (auto const& entry : compression_counts)
                sorted_counts.emplace_back(entry.second, entry.first);
            std::sort(sorted_counts.begin(), sorted_counts.end());
            return sorted_counts.back().second;
        }

        bool all_uncompressed = true;
        for (auto const& entry : compression_counts) {
            if (entry.first != COMPRESSION_NONE) {
                all_uncompressed = false;
                break;
            }
        }
        if (all_uncompressed)
            return {};
        if (key == "uncompressed")
            return std::to_string(total_uncompressed_size);
        if (key == "compressed")
            return std::to_string(total_compressed_size);
        return {};
    }

    if (key == "types") {
        if (datatypes.empty())
            return {};
        std::ostringstream oss;
        for (DatatypeInfo const& datatype : datatypes) {
            oss << "- type: " << datatype.datatype << '\n';
            oss << "  md5: " << datatype.md5sum << '\n';
        }
        return finish(oss.str());
    }

    if (key == "topics") {
        if (topic_infos.empty())
            return {};
        std::ostringstream oss;
        for (auto const& entry : topic_infos) {
            oss << "- topic: " << entry.first << '\n';
            oss << "  type: " << entry.second.datatype << '\n';
            oss << "  messages: " << entry.second.message_count << '\n';
            if (entry.second.connection_count > 1)
                oss << "  connections: " << entry.second.connection_count << '\n';
            if (compute_frequency && entry.second.has_frequency)
                oss << "  frequency: " << std::fixed << std::setprecision(4) << entry.second.frequency << '\n';
        }
        return finish(oss.str());
    }

    if (!key.empty())
        return {};

    std::ostringstream oss;
    oss << "path: " << bag.getFileName() << '\n';
    oss << "version: " << bag.getMajorVersion() << '.' << bag.getMinorVersion() << '\n';

    const double duration = end_stamp - start_stamp;
    oss << "duration: " << std::fixed << std::setprecision(6) << duration << '\n';
    oss << "start: " << std::fixed << std::setprecision(6) << start_stamp << '\n';
    oss << "end: " << std::fixed << std::setprecision(6) << end_stamp << '\n';
    oss << "size: " << bag.getSize() << '\n';
    oss << "messages: " << num_messages << '\n';
    oss << "indexed: True\n";

    {
        std::vector<ChunkHeader> chunk_headers = bag.collectChunkHeaders();
        if (chunk_headers.empty()) {
            oss << "compression: none\n";
        } else {
            std::map<std::string, uint32_t> compression_counts;
            for (ChunkHeader const& chunk_header : chunk_headers)
                compression_counts[chunk_header.compression]++;
            std::vector<std::pair<uint32_t, std::string>> sorted_counts;
            for (auto const& entry : compression_counts)
                sorted_counts.emplace_back(entry.second, entry.first);
            std::sort(sorted_counts.begin(), sorted_counts.end());
            oss << "compression: " << sorted_counts.back().second << '\n';

            bool all_uncompressed = true;
            for (auto const& entry : compression_counts) {
                if (entry.first != COMPRESSION_NONE) {
                    all_uncompressed = false;
                    break;
                }
            }
            if (!all_uncompressed) {
                uint64_t total_uncompressed_size = 0;
                uint64_t total_compressed_size = 0;
                for (ChunkHeader const& chunk_header : chunk_headers) {
                    total_uncompressed_size += chunk_header.uncompressed_size;
                    total_compressed_size += chunk_header.compressed_size;
                }
                oss << "uncompressed: " << total_uncompressed_size << '\n';
                oss << "compressed: " << total_compressed_size << '\n';
            }
        }
    }

    if (!datatypes.empty()) {
        oss << "types:\n";
        for (DatatypeInfo const& datatype : datatypes) {
            oss << "    - type: " << datatype.datatype << '\n';
            oss << "      md5: " << datatype.md5sum << '\n';
        }
    }

    if (!topic_infos.empty()) {
        oss << "topics:\n";
        for (auto const& entry : topic_infos) {
            oss << "    - topic: " << entry.first << '\n';
            oss << "      type: " << entry.second.datatype << '\n';
            oss << "      messages: " << entry.second.message_count << '\n';
            if (entry.second.connection_count > 1)
                oss << "      connections: " << entry.second.connection_count << '\n';
            if (compute_frequency && entry.second.has_frequency)
                oss << "      frequency: " << std::fixed << std::setprecision(4) << entry.second.frequency << '\n';
        }
    }

    return finish(oss.str());
}

} // namespace

std::string BagInfoFormatter::format(Bag const& bag, BagInfoOptions const& opts)
{
    const bool indexed = !bag.connection_indexes_.empty() || !bag.chunks_.empty();

    if (!indexed) {
        std::vector<InfoRow> rows;
        if (!bag.getFileName().empty())
            rows.push_back({"path", bag.getFileName()});
        rows.push_back({"version", std::to_string(bag.getMajorVersion()) + '.' + std::to_string(bag.getMinorVersion())});
        rows.push_back({"size", humanReadableSize(static_cast<double>(bag.getSize()))});
        if (opts.yaml) {
            if (!opts.key.empty() && opts.key != "path" && opts.key != "version" && opts.key != "size")
                return {};
            std::ostringstream oss;
            if (opts.key.empty() || opts.key == "path")
                oss << "path: " << bag.getFileName() << '\n';
            if (opts.key.empty() || opts.key == "version")
                oss << "version: " << bag.getMajorVersion() << '.' << bag.getMinorVersion() << '\n';
            if (opts.key.empty() || opts.key == "size")
                oss << "size: " << bag.getSize() << '\n';
            if (opts.key.empty())
                oss << "indexed: False\n";
            std::string result = oss.str();
            if (!result.empty() && result.back() == '\n')
                result.pop_back();
            return result;
        }
        return formatTextRows(rows);
    }

    double start_stamp = 0.0;
    double end_stamp = 0.0;
    if (!bag.chunks_.empty()) {
        start_stamp = bag.chunks_.front().start_time.toSec();
        end_stamp = bag.chunks_.back().end_time.toSec();
    } else {
        bool have_start = false;
        bool have_end = false;
        for (auto const& entry : bag.connection_indexes_) {
            if (entry.second.empty())
                continue;
            const double first = entry.second.begin()->time.toSec();
            const double last = entry.second.rbegin()->time.toSec();
            if (!have_start || first < start_stamp) {
                start_stamp = first;
                have_start = true;
            }
            if (!have_end || last > end_stamp) {
                end_stamp = last;
                have_end = true;
            }
        }
    }

    uint64_t num_messages = 0;
    if (!bag.chunks_.empty()) {
        for (ChunkInfo const& chunk : bag.chunks_) {
            for (auto const& count : chunk.connection_counts)
                num_messages += count.second;
        }
    } else {
        for (auto const& entry : bag.connection_indexes_)
            num_messages += entry.second.size();
    }

    const std::vector<DatatypeInfo> datatypes = collectDatatypes(bag.connections_);
    std::map<std::string, TopicInfo> topic_infos = collectTopics(
        bag.connections_, bag.chunks_, bag.connection_indexes_, opts.freq || opts.yaml);

    if (opts.yaml)
        return formatBagInfoYaml(
            bag, start_stamp, end_stamp, num_messages, datatypes, topic_infos, opts.freq, opts.key);

    return formatBagInfoText(
        bag, start_stamp, end_stamp, num_messages, datatypes, topic_infos);
}

std::string formatBagInfo(Bag const& bag, BagInfoOptions const& opts)
{
    return BagInfoFormatter::format(bag, opts);
}

} // namespace minibag
