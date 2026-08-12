#include "serial_arm/transport/serial_bus.hpp"
#include "serial_arm/transport/serial_port.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using serial_arm::transport::SerialPort;
using serial_arm::transport::SerialBus;
using serial_arm::transport::BusRegistry;
using serial_arm::transport::BusRegistryErr;

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

TEST(SerialBusTests, OpensSerialPortOnceAndClosesWithBusLifetime) {
    Pty pty;
    SerialBus bus(bus_config(pty));

    bus.open();
    const int fd = bus.transaction([](SerialPort& serial) {
        return serial.native_handle();
    });
    bus.open();
    const int same_fd = bus.transaction([](SerialPort& serial) {
        return serial.native_handle();
    });

    EXPECT_GE(fd, 0);
    EXPECT_EQ(same_fd, fd);
    EXPECT_TRUE(bus.is_open());

    bus.close();
    EXPECT_FALSE(bus.is_open());
}

TEST(SerialBusTests, DestructorClosesOwnedSerialPort) {
    int fd = -1;
    {
        Pty pty;
        SerialBus bus(bus_config(pty));
        bus.open();
        fd = bus.transaction([](SerialPort& serial) {
            return serial.native_handle();
        });
        ASSERT_GE(fd, 0);
    }

    errno = 0;
    EXPECT_EQ(::fcntl(fd, F_GETFD), -1);
    EXPECT_EQ(errno, EBADF);
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
        bus.transaction([&](SerialPort& serial) {
            first_started = true;
            EXPECT_EQ(serial.write(first.data(), first.size()), first.size());
            while(!first_can_finish) std::this_thread::yield();
        });
    });
    while(!first_started) std::this_thread::yield();

    std::thread second_thread([&]() {
        bus.transaction([&](SerialPort& serial) {
            EXPECT_EQ(serial.write(second.data(), second.size()), second.size());
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

    EXPECT_THROW(bus.transaction([](SerialPort&) {
        throw std::runtime_error("transaction failed");
    }), std::runtime_error);

    EXPECT_NO_THROW(bus.transaction([](SerialPort& serial) {
        EXPECT_TRUE(serial.is_open());
    }));
}

TEST(SerialBusTests, TimeoutDoesNotBlockFollowingTransaction) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    const std::size_t received = bus.transaction([](SerialPort& serial) {
        std::array<std::uint8_t, 1> value{};
        return serial.read(value.data(), value.size());
    });
    EXPECT_EQ(received, 0u);

    std::array<std::uint8_t, 1> input{ 9 };
    ASSERT_EQ(::write(pty.master(), input.data(), input.size()), 1);
    const std::size_t recovered = bus.transaction([](SerialPort& serial) {
        std::array<std::uint8_t, 1> value{};
        return serial.read(value.data(), value.size());
    });
    EXPECT_EQ(recovered, 1u);
}

TEST(SerialBusTests, FlushIsIsolatedInsideTransaction) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    std::array<std::uint8_t, 1> stale{ 7 };
    ASSERT_EQ(::write(pty.master(), stale.data(), stale.size()), 1);
    bus.transaction([](SerialPort& serial) {
        serial.flush(SerialPort::FlushDirection::Input);
    });

    const std::size_t received = bus.transaction([](SerialPort& serial) {
        std::array<std::uint8_t, 1> value{};
        return serial.read(value.data(), value.size());
    });
    EXPECT_EQ(received, 0u);
}

TEST(SerialBusTests, OneWayWriteUsesTransactionArbitration) {
    Pty pty;
    SerialBus bus(bus_config(pty));
    bus.open();

    bus.transaction([](SerialPort& serial) {
        EXPECT_EQ(serial.write({ 4, 5, 6 }), 3u);
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
