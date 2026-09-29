#define MCAP_IMPLEMENTATION
#include <mcap/mcap.hpp>

#include "protocol.hpp"
#include "recording.hpp"

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <sys/syscall.h>
#include <system_error>
#include <tuple>
#include <unistd.h>

namespace aviator {
namespace {
void checked(const mcap::Status& status) {
    if (!status.ok())
        throw std::runtime_error(status.message);
}
void io_error(const char* operation) {
    throw std::system_error(errno, std::generic_category(), operation);
}

// MCAP's default FileWriter does not report fflush/fclose failures. Use a
// checked POSIX sink and fsync before exposing the completed recording.
class FileSink : public mcap::IWritable {
  public:
    explicit FileSink(const std::string& path) {
        fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0640);
        if (fd_ < 0)
            io_error("create recording partial");
    }
    ~FileSink() override {
        if (fd_ >= 0)
            ::close(fd_);
    }
    void end() override {
        if (::fsync(fd_) != 0)
            io_error("fsync recording");
        const int fd = fd_;
        fd_ = -1;
        if (::close(fd) != 0)
            io_error("close recording");
    }
    std::uint64_t size() const override { return size_; }

  private:
    void handleWrite(const std::byte* data, std::uint64_t size) override {
        while (size > 0) {
            const auto amount = std::min<std::uint64_t>(size, 1024 * 1024);
            const auto written = ::write(fd_, data, amount);
            if (written < 0 && errno == EINTR)
                continue;
            if (written <= 0) {
                if (written == 0)
                    errno = EIO;
                io_error("write recording");
            }
            data += written;
            size -= written;
            size_ += written;
        }
    }
    int fd_ = -1;
    std::uint64_t size_ = 0;
};

std::string make_schema(std::string_view message_type) {
    // draft-07 里 "const" 单独出现是合法的，"type":"string" 属于冗余；但部分下游
    // 读取器只在显式给出 type 时才把它当字符串处理，所以每个 const 都配上 type。
    return std::string(
               "{\"$schema\":\"http://json-schema.org/draft-07/schema#\",\"type\":\"object\","
               "\"required\":[\"msg_type\",\"version\",\"sequence\",\"timestamp\","
               "\"sample_mono_us\",\"clock_id\",\"publisher_id\",\"session_id\",\"valid\"],"
               "\"properties\":{\"msg_type\":{\"type\":\"string\",\"const\":\"") +
           std::string(message_type) +
           "\"},\"version\":{\"type\":\"string\"},\"sequence\":{\"type\":\"integer\"},"
           "\"timestamp\":{\"type\":\"integer\"},\"sample_mono_us\":{\"type\":\"integer\"},"
           "\"clock_id\":{\"type\":\"string\"},\"publisher_id\":{\"type\":\"string\"},"
           "\"session_id\":{\"type\":\"string\"},\"valid\":{\"type\":\"boolean\"}}}";
}

} // namespace

struct RecordingWriter::Impl {
    using ChannelKey = std::tuple<std::string, std::string, std::string, std::string, std::string>;
    using SourceKey = std::tuple<std::string, std::string, std::string>;
    std::string path, partial, session;
    RecorderSummary summary;
    std::unique_ptr<FileSink> sink;
    mcap::McapWriter writer;
    std::map<std::string, mcap::SchemaId> schemas;
    std::map<ChannelKey, mcap::ChannelId> channels;
    std::map<SourceKey, std::uint64_t> last_sequence;
    bool finished = false;

    Impl(const std::string& output, const std::string& id, std::size_t chunk_size,
         const std::string& config)
        : path(output), partial(output + ".partial"), session(id) {
        if (path.empty() || session.empty())
            throw std::invalid_argument("empty output/session");
        if (std::filesystem::exists(path))
            throw std::runtime_error("output already exists: " + path);
        summary.path = path;
        sink = std::make_unique<FileSink>(partial);
        mcap::McapWriterOptions options("aviator");
        options.compression = mcap::Compression::None;
        options.chunkSize = chunk_size;
        options.enableDataCRC = true;
        try {
            writer.open(*sink, options);
            mcap::Metadata meta;
            meta.name = "aviator";
            meta.metadata = {{"node", "aviator_logger"},
                             {"effective_config", config},
                             {"session_id", session},
                             {"schema_scope", "common_header_only"},
                             {"completeness", "unverified_pubsub"}};
            checked(writer.write(meta));
        } catch (...) {
            writer.terminate();
            throw;
        }
    }
    ~Impl() { writer.terminate(); } // Never implicitly finalize during unwinding.
};

RecordingWriter::RecordingWriter(const std::string& path, const std::string& session,
                                 std::size_t chunk_size, const std::string& config)
    : impl_(std::make_unique<Impl>(path, session, chunk_size, config)) {}
RecordingWriter::~RecordingWriter() = default;

void RecordingWriter::append(std::string_view topic, std::string_view payload,
                             std::uint64_t receive_utc_ns) {
    auto& impl = *impl_;
    if (impl.finished)
        throw std::logic_error("recording already finished");
    Message decoded;
    std::string error;
    if (!decode(topic, payload, decoded, error)) {
        ++impl.summary.invalid;
        return;
    }
    const auto& h = decoded.header;
    const std::string type(message_type(decoded.topic));
    const auto schema_name = type + "/" + h.version;
    auto schema_it = impl.schemas.find(schema_name);
    if (schema_it == impl.schemas.end()) {
        if (impl.schemas.size() >= std::numeric_limits<mcap::SchemaId>::max())
            throw std::runtime_error("MCAP schema limit reached");
        mcap::Schema schema(schema_name, "jsonschema", make_schema(type));
        impl.writer.addSchema(schema);
        schema_it = impl.schemas.emplace(schema_name, schema.id).first;
    }
    const Impl::ChannelKey key{std::string(topic), h.version, h.publisher_id, h.session_id,
                               h.clock_id};
    auto channel_it = impl.channels.find(key);
    if (channel_it == impl.channels.end()) {
        // MCAP IDs are uint16; fail before wraparound rather than corrupt mappings.
        if (impl.channels.size() >= std::numeric_limits<mcap::ChannelId>::max())
            throw std::runtime_error("MCAP channel limit reached");
        mcap::Channel channel(std::string(topic), "json", schema_it->second,
                              {{"publisher_id", h.publisher_id},
                               {"session_id", h.session_id},
                               {"clock_id", h.clock_id},
                               {"version", h.version}});
        impl.writer.addChannel(channel);
        channel_it = impl.channels.emplace(key, channel.id).first;
        ++impl.summary.channels;
    }
    mcap::Message message;
    message.channelId = channel_it->second;
    message.sequence = static_cast<std::uint32_t>(h.sequence);
    message.logTime = receive_utc_ns;
    // timestamp is snapshot time, not publication time (SAD §10).
    message.publishTime = receive_utc_ns;
    message.data = reinterpret_cast<const std::byte*>(payload.data());
    message.dataSize = payload.size();
    checked(impl.writer.write(message));

    auto& last = impl.last_sequence[{std::string(topic), h.publisher_id, h.session_id}];
    if (last != 0) {
        if (h.sequence <= last)
            ++impl.summary.duplicate_or_reordered;
        else
            impl.summary.sequence_gaps += h.sequence - last - 1;
    }
    last = std::max(last, h.sequence);
    auto& stats = impl.summary.topics[std::string(topic)];
    stats.type = type;
    ++stats.messages;
    if (impl.summary.messages == 0) {
        impl.summary.start_log_ns = impl.summary.end_log_ns = receive_utc_ns;
    } else {
        impl.summary.start_log_ns = std::min(impl.summary.start_log_ns, receive_utc_ns);
        impl.summary.end_log_ns = std::max(impl.summary.end_log_ns, receive_utc_ns);
    }
    ++impl.summary.messages;
}

void RecordingWriter::metadata(const std::string& name, const nlohmann::json& value) {
    if (impl_->finished)
        throw std::logic_error("recording already finished");
    mcap::Metadata meta;
    meta.name = name;
    meta.metadata = {{"json", value.dump()}};
    checked(impl_->writer.write(meta));
}
void RecordingWriter::append_camera(const std::string& topic, const CameraFrame& frame,
                                    std::uint64_t log_ns) {
    auto& impl = *impl_;
    if (impl.finished)
        throw std::logic_error("recording already finished");
    auto schema = impl.schemas.find(camera_schema_name);
    if (schema == impl.schemas.end()) {
        if (impl.schemas.size() >= UINT16_MAX)
            throw std::runtime_error("MCAP schema limit reached");
        mcap::Schema value(camera_schema_name, "protobuf", camera_schema_descriptor());
        impl.writer.addSchema(value);
        schema = impl.schemas.emplace(camera_schema_name, value.id).first;
    }
    const auto& m = frame.metadata;
    const auto publisher = m.at("publisher_id").get<std::string>();
    const auto session = m.at("session_id").get<std::string>();
    const auto clock = m.at("clock_id").get<std::string>();
    const auto encoding = m.at("encoding").get<std::string>();
    Impl::ChannelKey key{topic, encoding, publisher, session, clock};
    auto channel = impl.channels.find(key);
    if (channel == impl.channels.end()) {
        if (impl.channels.size() >= UINT16_MAX)
            throw std::runtime_error("MCAP channel limit reached");
        mcap::Channel value(topic, "protobuf", schema->second,
                            {{"publisher_id", publisher},
                             {"session_id", session},
                             {"clock_id", clock},
                             {"encoding", encoding}});
        impl.writer.addChannel(value);
        channel = impl.channels.emplace(key, value.id).first;
        ++impl.summary.channels;
    }
    const auto payload = serialize_camera_frame(frame);
    const auto sequence = m.at("sequence").get<std::uint64_t>();
    mcap::Message message;
    message.channelId = channel->second;
    message.sequence = static_cast<std::uint32_t>(sequence);
    message.logTime = log_ns;
    message.publishTime = log_ns;
    message.data = reinterpret_cast<const std::byte*>(payload.data());
    message.dataSize = payload.size();
    checked(impl.writer.write(message));
    auto& last = impl.last_sequence[{topic, publisher, session}];
    if (last) {
        if (sequence <= last)
            ++impl.summary.duplicate_or_reordered;
        else
            impl.summary.sequence_gaps += sequence - last - 1;
    }
    last = std::max(last, sequence);
    auto& stats = impl.summary.topics[topic];
    stats.type = camera_schema_name;
    ++stats.messages;
    if (!impl.summary.messages)
        impl.summary.start_log_ns = impl.summary.end_log_ns = log_ns;
    else {
        impl.summary.start_log_ns = std::min(impl.summary.start_log_ns, log_ns);
        impl.summary.end_log_ns = std::max(impl.summary.end_log_ns, log_ns);
    }
    ++impl.summary.messages;
    ++impl.summary.camera_messages;
}

RecorderSummary RecordingWriter::finish(std::uint64_t rejected, std::uint64_t dropped) {
    auto& impl = *impl_;
    if (impl.finished)
        throw std::logic_error("recording already finished");
    impl.summary.rejected = rejected;
    impl.summary.dropped = dropped;
    nlohmann::json topics = nlohmann::json::array();
    for (const auto& [topic, stats] : impl.summary.topics)
        topics.push_back({{"topic", topic}, {"type", stats.type}, {"messages", stats.messages}});
    mcap::Metadata metadata;
    metadata.name = "summary";
    metadata.metadata = {
        {"session_id", impl.session},
        {"message_count", std::to_string(impl.summary.messages)},
        {"camera_message_count", std::to_string(impl.summary.camera_messages)},
        {"channel_count", std::to_string(impl.summary.channels)},
        {"invalid", std::to_string(impl.summary.invalid)},
        {"rejected", std::to_string(rejected)},
        {"queue_dropped", std::to_string(dropped)},
        {"sequence_gaps", std::to_string(impl.summary.sequence_gaps)},
        {"duplicate_or_reordered", std::to_string(impl.summary.duplicate_or_reordered)},
        {"start_log_ns", std::to_string(impl.summary.start_log_ns)},
        {"end_log_ns", std::to_string(impl.summary.end_log_ns)},
        {"completeness", "unverified_pubsub"},
        {"topics", topics.dump()}};
    checked(impl.writer.write(metadata));
    impl.writer.close();
    impl.finished = true;
    // Verify footer/index and message count before publishing the file.
    mcap::McapReader reader;
    checked(reader.open(impl.partial));
    checked(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan));
    std::uint64_t count = 0;
    for (const auto& view :
         reader.readMessages([](const mcap::Status& status) { checked(status); })) {
        if (!view.schema || !view.channel)
            throw std::runtime_error("missing MCAP mapping");
        ++count;
    }
    if (count != impl.summary.messages)
        throw std::runtime_error("MCAP count verification failed");
    reader.close();
    // Linux deployment baseline: atomic rename with no replacement, including
    // a competing creator after the initial existence check.
    if (::syscall(SYS_renameat2, AT_FDCWD, impl.partial.c_str(), AT_FDCWD, impl.path.c_str(),
                  1 /* RENAME_NOREPLACE */) != 0)
        io_error("finalize recording");
    auto parent = std::filesystem::path(impl.path).parent_path();
    if (parent.empty())
        parent = ".";
    const int directory = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory < 0)
        io_error("open recording directory");
    const int result = ::fsync(directory);
    const int saved_errno = errno;
    ::close(directory);
    if (result != 0) {
        errno = saved_errno;
        io_error("fsync recording directory");
    }
    return impl.summary;
}
} // namespace aviator
