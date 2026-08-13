#include "serial_arm/transport/serial_bus.hpp"
#include "serial_arm/transport/serial_port.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <dirent.h>
#include <exception>
#include <fcntl.h>
#include <limits.h>
#include <functional>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

using serial_arm::transport::SerialPort;
using serial_arm::transport::SerialBus;
using serial_arm::transport::SerialTransaction;
using serial_arm::transport::SerialTransactionOptions;
using serial_arm::transport::BusRegistry;
using serial_arm::transport::BusRegistryErr;

template<typename T, typename = void>
struct HasClose : std::false_type {
};

template<typename T>
struct HasClose<T, std::void_t<decltype(std::declval<T&>().close())>> : std::true_type {
};

template<typename T, typename = void>
struct HasNativeHandle : std::false_type {
};

template<typename T>
struct HasNativeHandle<T, std::void_t<decltype(std::declval<T&>().native_handle())>> : std::true_type {
};

template<typename T, typename = void>
struct HasOpen : std::false_type {
};

template<typename T>
struct HasOpen<T, std::void_t<decltype(std::declval<T&>().open(std::declval<std::string>()))>> : std::true_type {
};

template<typename T, typename = void>
struct HasSetConfig : std::false_type {
};

template<typename T>
struct HasSetConfig<T, std::void_t<decltype(std::declval<T&>().set_config(std::declval<SerialPort::Config>()))>>
    : std::true_type {
};

static_assert(!HasClose<SerialTransaction>::value, "SerialTransaction must not expose close()");
static_assert(!HasNativeHandle<SerialTransaction>::value, "SerialTransaction must not expose native_handle()");
static_assert(!HasOpen<SerialTransaction>::value, "SerialTransaction must not expose open()");
static_assert(!HasSetConfig<SerialTransaction>::value, "SerialTransaction must not expose set_config()");

class Pty {
public:
    Pty() {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if(master_ < 0) throw std::runtime_error("posix_openpt failed");
        if(::grantpt(master_) != 0 || ::unlockpt(master_) != 0) throw std::runtime_error("pty setup failed");
        const char* name = ::ptsname(master_);
        if(name == nullptr) throw std::runtime_error("ptsname failed");
        slave_ = name;
    }

    ~Pty() {
        if(master_ >= 0) (void)::close(master_);
    }

    Pty(const Pty&) = delete;
    Pty& operator=(const Pty&) = delete;

    int master() const noexcept { return master_; }
    const std::string& slave() const noexcept { return slave_; }

private:
    int master_{ -1 };
    std::string slave_;
};

SerialBus::Config bus_config(const Pty& pty) {
    SerialBus::Config config;
    config.serial_port = pty.slave();
    config.port_config.read_timeout = std::chrono::milliseconds(5);
    config.port_config.write_timeout = std::chrono::milliseconds(50);
    return config;
}

std::array<std::uint8_t, 2> read_master_request(int fd) {
    std::array<std::uint8_t, 2> value{};
    std::size_t offset = 0;
    while(offset < value.size()) {
        const ssize_t received = ::read(fd, value.data() + offset, value.size() - offset);
        if(received > 0) {
            offset += static_cast<std::size_t>(received);
            continue;
        }
        if(received < 0 && errno == EINTR) continue;
        throw std::runtime_error("pty master read failed");
    }
    return value;
}

void write_master_response(int fd, const std::array<std::uint8_t, 2>& value) {
    std::size_t offset = 0;
    while(offset < value.size()) {
        const ssize_t written = ::write(fd, value.data() + offset, value.size() - offset);
        if(written > 0) {
            offset += static_cast<std::size_t>(written);
            continue;
        }
        if(written < 0 && errno == EINTR) continue;
        throw std::runtime_error("pty master write failed");
    }
}

bool master_has_input(int fd, std::chrono::milliseconds timeout) {
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    int ret;
    do {
        ret = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
    }
    while(ret < 0 && errno == EINTR);
    if(ret < 0) throw std::runtime_error("pty master poll failed");
    return ret > 0 && (pfd.revents & POLLIN) != 0;
}

std::size_t count_open_fds_for_path(const std::string& path) {
    DIR* directory = ::opendir("/proc/self/fd");
    if(directory == nullptr) throw std::runtime_error("opendir(/proc/self/fd) failed");

    std::size_t count = 0;
    while(const dirent* entry = ::readdir(directory)) {
        if(entry->d_name[0] == '.') continue;
        const std::string fd_path = std::string("/proc/self/fd/") + entry->d_name;
        char target[PATH_MAX]{};
        const ssize_t length = ::readlink(fd_path.c_str(), target, sizeof(target) - 1);
        if(length < 0) continue;
        target[length] = '\0';
        if(path == target) ++count;
    }
    (void)::closedir(directory);
    return count;
}

} // namespace

TEST(SerialPortTests, RejectsInvalidConfiguration) {
    SerialPort::Config config;
    config.baud_rate = 12345;
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);

    config = SerialPort::Config{};
    config.data_bits = 9;
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);

    config = SerialPort::Config{};
    config.read_timeout = std::chrono::milliseconds(-1);
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);

    config = SerialPort::Config{};
    config.write_timeout = std::chrono::milliseconds(-1);
    EXPECT_THROW(SerialPort("/dev/null", config), std::invalid_argument);
}

TEST(SerialPortTests, ExplicitOperationTimeoutDoesNotMutatePersistentConfiguration) {
    Pty pty;
    SerialPort::Config config;
    config.read_timeout = std::chrono::milliseconds(100);
    config.write_timeout = std::chrono::milliseconds(100);
    SerialPort port(pty.slave(), config);

    std::array<std::uint8_t, 1> value{};
    EXPECT_EQ(port.read(value.data(), value.size(), std::chrono::milliseconds(1)), 0u);
    EXPECT_EQ(port.config().read_timeout, std::chrono::milliseconds(100));

    EXPECT_EQ(port.write({ 0x51 }), 1u);
    std::array<std::uint8_t, 1> output{};
    ASSERT_EQ(::read(pty.master(), output.data(), output.size()), 1);
    EXPECT_EQ(output[0], 0x51);
    EXPECT_EQ(port.config().write_timeout, std::chrono::milliseconds(100));
}

TEST(SerialPortTests, OpensAndReopensWithoutDroppingOldFdOnFailure) {
    Pty first;
    Pty second;
    SerialPort port(first.slave());
    ASSERT_TRUE(port.is_open());

    const std::array<std::uint8_t, 1> old_byte{ 0x41 };
    ASSERT_EQ(::write(first.master(), old_byte.data(), old_byte.size()), 1);
    std::array<std::uint8_t, 1> read_buf{};
    EXPECT_EQ(port.read_exact(read_buf.data(), read_buf.size()), 1);
    EXPECT_EQ(read_buf[0], old_byte[0]);

    SerialPort::Config invalid;
    invalid.baud_rate = 12345;
    EXPECT_THROW(port.open(second.slave(), invalid), std::invalid_argument);
    ASSERT_TRUE(port.is_open());
    const std::array<std::uint8_t, 1> still_old{ 0x42 };
    ASSERT_EQ(::write(first.master(), still_old.data(), still_old.size()), 1);
    EXPECT_EQ(port.read_exact(read_buf.data(), read_buf.size()), 1);
    EXPECT_EQ(read_buf[0], still_old[0]);

    port.open(second.slave());
    const std::array<std::uint8_t, 1> new_byte{ 0x43 };
    ASSERT_EQ(::write(second.master(), new_byte.data(), new_byte.size()), 1);
    EXPECT_EQ(port.read_exact(read_buf.data(), read_buf.size()), 1);
    EXPECT_EQ(read_buf[0], new_byte[0]);
}

TEST(SerialPortTests, ReadWriteTimeoutFlushDrainAndMove) {
    Pty pty;
    SerialPort::Config config;
    config.read_timeout = std::chrono::milliseconds(5);
    config.write_timeout = std::chrono::milliseconds(50);
    SerialPort port(pty.slave(), config);
    EXPECT_TRUE(port.is_open());
    EXPECT_EQ(port.config().read_timeout, std::chrono::milliseconds(5));
    EXPECT_EQ(port.config().write_timeout, std::chrono::milliseconds(50));

    const std::array<std::uint8_t, 3> out{ 1, 2, 3 };
    EXPECT_EQ(port.write(out.data(), out.size()), out.size());
    std::array<std::uint8_t, 3> from_port{};
    ASSERT_EQ(::read(pty.master(), from_port.data(), from_port.size()), 3);
    EXPECT_EQ(from_port, out);
    EXPECT_NO_THROW(port.drain());

    std::array<std::uint8_t, 2> in{ 4, 5 };
    ASSERT_EQ(::write(pty.master(), in.data(), in.size()), 2);
    EXPECT_GE(port.available(), 0u);
    std::array<std::uint8_t, 2> to_port{};
    EXPECT_EQ(port.read(to_port.data(), to_port.size()), to_port.size());
    EXPECT_EQ(to_port, in);
    EXPECT_EQ(port.read(to_port.data(), to_port.size()), 0u);

    port.flush();
    SerialPort moved(std::move(port));
    EXPECT_TRUE(moved.is_open());
    EXPECT_FALSE(port.is_open());
    SerialPort assigned;
    assigned = std::move(moved);
    EXPECT_TRUE(assigned.is_open());
    EXPECT_FALSE(moved.is_open());
}

TEST(SerialPortTests, BufferApisAndIndependentTimeoutSetters) {
    Pty pty;
    SerialPort port(pty.slave());
    port.set_read_timeout(std::chrono::milliseconds(3));
    port.set_write_timeout(std::chrono::milliseconds(40));
    EXPECT_EQ(port.config().read_timeout, std::chrono::milliseconds(3));
    EXPECT_EQ(port.config().write_timeout, std::chrono::milliseconds(40));

    EXPECT_EQ(port.write({ 7, 8 }), 2u);
    std::array<std::uint8_t, 2> from_port{};
    ASSERT_EQ(::read(pty.master(), from_port.data(), from_port.size()), 2);
    EXPECT_EQ(from_port[0], 7);
    EXPECT_EQ(from_port[1], 8);

    ASSERT_EQ(::write(pty.master(), from_port.data(), from_port.size()), 2);
    SerialPort::Buffer buffer;
    EXPECT_EQ(port.read(buffer, 2), 2u);
    EXPECT_EQ(buffer.size(), 2u);
}

TEST(SerialBusTests, TtyOwnershipKeyCanonicalizesSymlinkAlias) {
    Pty pty;
    const std::string alias = "/tmp/serial_arm_core_tty_alias";
    (void)::unlink(alias.c_str());
    ASSERT_EQ(::symlink(pty.slave().c_str(), alias.c_str()), 0);

    EXPECT_EQ(
        serial_arm::transport::tty_ownership_key(pty.slave()),
        serial_arm::transport::tty_ownership_key(alias));
    EXPECT_EQ(::unlink(alias.c_str()), 0);
}

TEST(SerialBusTests, OpenIsIdempotentAndCloseStopsProtocolAccess) {
    Pty pty;
    const auto initial_fd_count = count_open_fds_for_path(pty.slave());
    SerialBus bus(bus_config(pty));

    bus.open();
    EXPECT_TRUE(bus.is_open());
    EXPECT_EQ(count_open_fds_for_path(pty.slave()), initial_fd_count + 1);

    bus.open();
    EXPECT_EQ(count_open_fds_for_path(pty.slave()), initial_fd_count + 1);

    bus.transaction([](SerialTransaction& transaction) {
        EXPECT_EQ(transaction.write({ 0x11 }), 1u);
    });
    std::array<std::uint8_t, 1> byte{};
    ASSERT_EQ(::read(pty.master(), byte.data(), byte.size()), 1);
    EXPECT_EQ(byte[0], 0x11);

    bus.close();
    EXPECT_FALSE(bus.is_open());
    EXPECT_EQ(count_open_fds_for_path(pty.slave()), initial_fd_count);
    EXPECT_THROW(bus.transaction([](SerialTransaction& transaction) {
        (void)transaction.write({ 0x12 });
    }), std::runtime_error);
}

TEST(SerialBusTests, DestructorClosesOwnedSerialPort) {
    Pty pty;
    const auto initial_fd_count = count_open_fds_for_path(pty.slave());
    {
        SerialBus bus(bus_config(pty));
        bus.open();
        ASSERT_TRUE(bus.is_open());
        EXPECT_EQ(count_open_fds_for_path(pty.slave()), initial_fd_count + 1);
    }

    EXPECT_EQ(count_open_fds_for_path(pty.slave()), initial_fd_count);
}

TEST(SerialBusTests, ConcurrentTransactionsDoNotInterleaveReadWrite) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    std::array<std::uint8_t, 2> first{ 1, 2 };
    std::array<std::uint8_t, 2> second{ 3, 4 };
    std::atomic<bool> first_started{ false };
    std::atomic<bool> first_can_finish{ false };
    std::vector<std::uint8_t> master_bytes;
    master_bytes.reserve(4);

    std::thread first_thread([&]() {
        bus.transaction([&](SerialTransaction& transaction) {
            first_started = true;
            EXPECT_EQ(transaction.write(first.data(), first.size()), first.size());
            while(!first_can_finish) std::this_thread::yield();
        });
    });
    while(!first_started) std::this_thread::yield();

    std::thread second_thread([&]() {
        bus.transaction([&](SerialTransaction& transaction) {
            EXPECT_EQ(transaction.write(second.data(), second.size()), second.size());
        });
    });

    std::array<std::uint8_t, 2> read_buf{};
    ASSERT_EQ(::read(pty.master(), read_buf.data(), read_buf.size()), 2);
    master_bytes.insert(master_bytes.end(), read_buf.begin(), read_buf.end());
    first_can_finish = true;
    first_thread.join();
    second_thread.join();

    ASSERT_EQ(::read(pty.master(), read_buf.data(), read_buf.size()), 2);
    master_bytes.insert(master_bytes.end(), read_buf.begin(), read_buf.end());
    EXPECT_EQ(master_bytes, (std::vector<std::uint8_t>{ 1, 2, 3, 4 }));
}

TEST(SerialBusTests, ExceptionReleasesTransactionLock) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    EXPECT_THROW(bus.transaction([](SerialTransaction&) {
        throw std::runtime_error("transaction failed");
    }), std::runtime_error);

    EXPECT_NO_THROW(bus.transaction([](SerialTransaction&) {
    }));
}

TEST(SerialBusTests, DiagnosticsCountsTransactionsAndRethrowsOriginalException) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    EXPECT_NO_THROW(bus.transaction([](SerialTransaction&) {
    }));
    EXPECT_THROW(bus.transaction([](SerialTransaction&) {
        throw std::runtime_error("protocol callback failed");
    }), std::runtime_error);

    const auto diagnostics = bus.diagnostics();
    EXPECT_TRUE(diagnostics.is_open);
    EXPECT_EQ(diagnostics.transaction_count, 2u);
    EXPECT_EQ(diagnostics.failed_transaction_count, 1u);
    EXPECT_EQ(diagnostics.resource.kind, serial_arm::transport::BusResourceKind::SERIAL);
    EXPECT_EQ(diagnostics.resource.physical_id, pty.slave());
    EXPECT_EQ(diagnostics.resource.ownership_key, "tty:" + pty.slave());
    EXPECT_NE(diagnostics.resource.config_signature.find("serial|baudrate="), std::string::npos);
    EXPECT_EQ(diagnostics.resource.config_signature.find("timeout"), std::string::npos);
}

TEST(SerialBusTests, TimeoutDoesNotBlockFollowingTransaction) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    const std::size_t received = bus.transaction([](SerialTransaction& transaction) {
        std::array<std::uint8_t, 1> value{};
        return transaction.read(value.data(), value.size());
    });
    EXPECT_EQ(received, 0u);

    std::array<std::uint8_t, 1> input{ 9 };
    ASSERT_EQ(::write(pty.master(), input.data(), input.size()), 1);
    const std::size_t recovered = bus.transaction([](SerialTransaction& transaction) {
        std::array<std::uint8_t, 1> value{};
        return transaction.read(value.data(), value.size());
    });
    EXPECT_EQ(recovered, 1u);
}

TEST(SerialBusTests, FlushIsIsolatedInsideTransaction) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    std::array<std::uint8_t, 1> stale{ 7 };
    ASSERT_EQ(::write(pty.master(), stale.data(), stale.size()), 1);
    bus.transaction([](SerialTransaction& transaction) {
        transaction.flush(SerialTransaction::FlushDirection::Input);
    });

    const std::size_t received = bus.transaction([](SerialTransaction& transaction) {
        std::array<std::uint8_t, 1> value{};
        return transaction.read(value.data(), value.size());
    });
    EXPECT_EQ(received, 0u);
}

TEST(SerialBusTests, OneWayWriteUsesTransactionArbitration) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    bus.transaction([](SerialTransaction& transaction) {
        EXPECT_EQ(transaction.write({ 4, 5, 6 }), 3u);
    });

    std::array<std::uint8_t, 3> bytes{};
    ASSERT_EQ(::read(pty.master(), bytes.data(), bytes.size()), 3);
    EXPECT_EQ(bytes[0], 4);
    EXPECT_EQ(bytes[1], 5);
    EXPECT_EQ(bytes[2], 6);
}

TEST(SerialBusTests, BusRegistryRejectsConflictingSerialBusConfiguration) {
    Pty pty;
    auto config = bus_config(pty);
    auto first = BusRegistry::get_or_create<SerialBus>(
        "serial_bus_registry_a",
        SerialBus::resource_descriptor(config),
        [&]() {
            auto bus = std::make_shared<SerialBus>(config);
            bus->open();
            return bus;
        });
    ASSERT_TRUE(first);

    auto conflicting_config = config;
    conflicting_config.port_config.baud_rate = 57600;
    auto second = BusRegistry::get_or_create<SerialBus>(
        "serial_bus_registry_b",
        SerialBus::resource_descriptor(conflicting_config),
        [&]() {
            auto bus = std::make_shared<SerialBus>(conflicting_config);
            bus->open();
            return bus;
        });

    ASSERT_FALSE(second);
    EXPECT_EQ(second.error(), BusRegistryErr::CONFIG_CONFLICT);
}

TEST(SerialBusTests, ResourceDescriptorDoesNotTreatTimeoutAsPhysicalCompatibility) {
    Pty pty;
    auto first = bus_config(pty);
    auto second = first;
    second.port_config.read_timeout = std::chrono::milliseconds(250);
    second.port_config.write_timeout = std::chrono::milliseconds(500);

    const auto first_resource = SerialBus::resource_descriptor(first);
    const auto second_resource = SerialBus::resource_descriptor(second);
    EXPECT_EQ(first_resource.ownership_key, second_resource.ownership_key);
    EXPECT_EQ(first_resource.config_signature, second_resource.config_signature);
}

TEST(SerialBusTests, IndependentClientsDoNotInterleaveRequestResponseTransactions) {
    Pty pty;
    auto config = bus_config(pty);
    SerialBus bus(config);
    bus.open();

    constexpr std::array<std::uint8_t, 2> request_a{ 0xA1, 0x01 };
    constexpr std::array<std::uint8_t, 2> response_a{ 0xA2, 0x01 };
    constexpr std::array<std::uint8_t, 2> request_b{ 0xB1, 0x02 };
    constexpr std::array<std::uint8_t, 2> response_b{ 0xB2, 0x02 };

    std::array<std::uint8_t, 2> first_request{};
    std::array<std::uint8_t, 2> second_request{};
    std::atomic<bool> interleaved_before_first_response{ false };
    std::exception_ptr responder_error;

    std::thread responder([&]() {
        try {
            first_request = read_master_request(pty.master());
            interleaved_before_first_response = master_has_input(pty.master(), std::chrono::milliseconds(60));
            write_master_response(pty.master(), first_request == request_a ? response_a : response_b);

            second_request = read_master_request(pty.master());
            write_master_response(pty.master(), second_request == request_a ? response_a : response_b);
        }
        catch(...) {
            responder_error = std::current_exception();
        }
    });

    std::atomic<int> ready{ 0 };
    std::atomic<bool> start{ false };
    std::array<std::uint8_t, 2> client_a_response{};
    std::array<std::uint8_t, 2> client_b_response{};
    SerialTransactionOptions options;
    options.read_timeout = std::chrono::milliseconds(200);
    options.write_timeout = std::chrono::milliseconds(50);

    auto client = [&](const std::array<std::uint8_t, 2>& request, std::array<std::uint8_t, 2>& response) {
        ++ready;
        while(!start) std::this_thread::yield();
        bus.transaction(options, [&](SerialTransaction& transaction) {
            EXPECT_EQ(transaction.write(request.data(), request.size()), request.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
            EXPECT_EQ(transaction.read_exact(response.data(), response.size()), response.size());
        });
    };

    std::thread client_a(client, std::cref(request_a), std::ref(client_a_response));
    std::thread client_b(client, std::cref(request_b), std::ref(client_b_response));
    while(ready.load() != 2) std::this_thread::yield();
    start = true;

    client_a.join();
    client_b.join();
    responder.join();
    if(responder_error) std::rethrow_exception(responder_error);

    EXPECT_FALSE(interleaved_before_first_response.load());
    EXPECT_TRUE(
        (first_request == request_a && second_request == request_b) ||
        (first_request == request_b && second_request == request_a));
    EXPECT_EQ(client_a_response, response_a);
    EXPECT_EQ(client_b_response, response_b);
}

TEST(SerialBusTests, TimedOutClientReleasesBusForWaitingClientWithIndependentTimeout) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    constexpr std::array<std::uint8_t, 2> request_a{ 0xC1, 0x01 };
    constexpr std::array<std::uint8_t, 2> request_b{ 0xD1, 0x02 };
    constexpr std::array<std::uint8_t, 2> response_b{ 0xD2, 0x02 };

    std::atomic<bool> client_a_entered{ false };
    std::array<std::uint8_t, 2> response_a{};
    std::array<std::uint8_t, 2> received_b{};
    std::exception_ptr responder_error;

    std::thread responder([&]() {
        try {
            EXPECT_EQ(read_master_request(pty.master()), request_a);
            EXPECT_EQ(read_master_request(pty.master()), request_b);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            write_master_response(pty.master(), response_b);
        }
        catch(...) {
            responder_error = std::current_exception();
        }
    });

    SerialTransactionOptions short_timeout;
    short_timeout.read_timeout = std::chrono::milliseconds(10);
    short_timeout.write_timeout = std::chrono::milliseconds(50);
    SerialTransactionOptions long_timeout;
    long_timeout.read_timeout = std::chrono::milliseconds(100);
    long_timeout.write_timeout = std::chrono::milliseconds(50);

    std::thread client_a([&]() {
        bus.transaction(short_timeout, [&](SerialTransaction& transaction) {
            client_a_entered = true;
            EXPECT_EQ(transaction.write(request_a.data(), request_a.size()), request_a.size());
            EXPECT_EQ(transaction.read_exact(response_a.data(), response_a.size()), 0u);
        });
    });
    while(!client_a_entered) std::this_thread::yield();

    std::thread client_b([&]() {
        bus.transaction(long_timeout, [&](SerialTransaction& transaction) {
            EXPECT_EQ(transaction.write(request_b.data(), request_b.size()), request_b.size());
            EXPECT_EQ(transaction.read_exact(received_b.data(), received_b.size()), received_b.size());
        });
    });

    client_a.join();
    client_b.join();
    responder.join();
    if(responder_error) std::rethrow_exception(responder_error);

    EXPECT_EQ(received_b, response_b);
    EXPECT_TRUE(bus.is_open());
}

TEST(SerialBusTests, SharedRegistryOwnerSurvivesAfterOneClientReleasesReference) {
    Pty pty;
    auto config = bus_config(pty);

    std::shared_ptr<SerialBus> remaining_client;
    {
        auto first = BusRegistry::get_or_create<SerialBus>(
            "serial_bus_shared_owner_lifetime",
            SerialBus::resource_descriptor(config),
            [&]() {
                auto bus = std::make_shared<SerialBus>(config);
                bus->open();
                return bus;
            });
        auto second = BusRegistry::get_or_create<SerialBus>(
            "serial_bus_shared_owner_lifetime",
            SerialBus::resource_descriptor(config),
            [&]() {
                auto bus = std::make_shared<SerialBus>(config);
                bus->open();
                return bus;
            });

        ASSERT_TRUE(first);
        ASSERT_TRUE(second);
        EXPECT_EQ(first->get(), second->get());
        std::shared_ptr<SerialBus> released_client = *first;
        remaining_client = *second;
        released_client.reset();
    }

    ASSERT_TRUE(remaining_client);
    constexpr std::array<std::uint8_t, 2> request{ 0x31, 0x41 };
    constexpr std::array<std::uint8_t, 2> response{ 0x32, 0x42 };
    std::exception_ptr responder_error;
    std::thread responder([&]() {
        try {
            EXPECT_EQ(read_master_request(pty.master()), request);
            write_master_response(pty.master(), response);
        }
        catch(...) {
            responder_error = std::current_exception();
        }
    });

    std::array<std::uint8_t, 2> received{};
    remaining_client->transaction([&](SerialTransaction& transaction) {
        EXPECT_EQ(transaction.write(request.data(), request.size()), request.size());
        EXPECT_EQ(transaction.read_exact(received.data(), received.size()), received.size());
    });

    responder.join();
    if(responder_error) std::rethrow_exception(responder_error);
    EXPECT_EQ(received, response);
}
