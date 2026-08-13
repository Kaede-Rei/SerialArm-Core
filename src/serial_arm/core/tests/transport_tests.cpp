#include "serial_arm/transport/bus.hpp"
#include "serial_arm/transport/serial_bus.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace {

using serial_arm::transport::CanBus;
using serial_arm::transport::CanErr;
using serial_arm::transport::CanFilter;
using serial_arm::transport::CanFrame;
using serial_arm::transport::BusRegistryErr;
using serial_arm::transport::BusResourceDescriptor;
using serial_arm::transport::BusResourceKind;
using serial_arm::transport::SerialBusConfig;
using serial_arm::transport::acquire_can_channel;
using serial_arm::transport::acquire_serial_bus_client;

class FakeCanDriver final {
public:
    explicit FakeCanDriver(std::shared_ptr<serial_arm::transport::CanChannel> channel)
        : channel_(std::move(channel)) {
    }

    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout) {
        return channel_->receive(timeout);
    }

    tl::expected<void, CanErr> send(const CanFrame& value) {
        return channel_->send(value);
    }

    serial_arm::transport::CanChannelDiagnostics diagnostics() const noexcept {
        return channel_->diagnostics();
    }

private:
    std::shared_ptr<serial_arm::transport::CanChannel> channel_;
};

class MockCanBus final : public CanBus {
public:
    tl::expected<void, CanErr> open() override {
        open_ = true;
        return {};
    }

    void close() noexcept override {
        open_ = false;
    }

    bool is_open() const noexcept override {
        return open_;
    }

    tl::expected<void, CanErr> send(const CanFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.push_back(frame);
        return {};
    }

    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if(rx_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
        CanFrame frame = rx_.front();
        rx_.pop_front();
        return frame;
    }

    void flush() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
    }

    void push_rx(const CanFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.push_back(frame);
    }

    std::size_t tx_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_.size();
    }

private:
    bool open_{ false };
    mutable std::mutex mutex_;
    std::deque<CanFrame> rx_;
    std::deque<CanFrame> tx_;
};

class BlockingMockCanBus final : public CanBus {
public:
    tl::expected<void, CanErr> open() override {
        open_ = true;
        return {};
    }

    void close() noexcept override {
        open_ = false;
    }

    bool is_open() const noexcept override {
        return open_;
    }

    tl::expected<void, CanErr> send(const CanFrame& frame) override {
        std::lock_guard<std::mutex> lock(mutex_);
        tx_.push_back(frame);
        return {};
    }

    tl::expected<CanFrame, CanErr> receive(std::chrono::milliseconds timeout) override {
        std::this_thread::sleep_for(timeout);
        std::lock_guard<std::mutex> lock(mutex_);
        if(rx_.empty()) return tl::make_unexpected(CanErr::TIMEOUT);
        CanFrame frame = rx_.front();
        rx_.pop_front();
        return frame;
    }

    void flush() noexcept override {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.clear();
    }

    void push_rx(const CanFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        rx_.push_back(frame);
    }

    std::size_t tx_size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return tx_.size();
    }

private:
    bool open_{ false };
    mutable std::mutex mutex_;
    std::deque<CanFrame> rx_;
    std::deque<CanFrame> tx_;
};

CanFrame frame(std::uint32_t id) {
    CanFrame value;
    value.id = id;
    value.size = 1;
    value.data[0] = static_cast<std::uint8_t>(id);
    return value;
}

BusResourceDescriptor can_resource(const std::string& physical_id) {
    BusResourceDescriptor resource;
    resource.kind = BusResourceKind::CAN;
    resource.physical_id = physical_id;
    resource.config_signature = "classic-can";
    resource.ownership_key = "can:" + physical_id;
    return resource;
}

} // namespace

TEST(TransportTests, SingleChannelReceivesMatchingFrame) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto channel = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    bus->push_rx(frame(0x01));

    auto received = channel->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(received);
    EXPECT_EQ(received->id, 0x01u);
}

TEST(TransportTests, ChannelDoesNotDropOtherChannelFrame) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });
    bus->push_rx(frame(0x20));
    bus->push_rx(frame(0x01));

    auto arm_frame = arm->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(arm_frame);
    EXPECT_EQ(arm_frame->id, 0x01u);

    auto eef_frame = eef->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(eef_frame);
    EXPECT_EQ(eef_frame->id, 0x20u);
}

TEST(TransportTests, ChannelFlushDoesNotClearOtherChannelPendingFrame) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });
    bus->push_rx(frame(0x20));
    bus->push_rx(frame(0x01));

    auto arm_frame = arm->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(arm_frame);
    arm->flush();

    auto eef_frame = eef->receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(eef_frame);
    EXPECT_EQ(eef_frame->id, 0x20u);
}

TEST(TransportTests, UnknownFrameDoesNotEnterOtherChannels) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    bus->push_rx(frame(0x30));

    auto received = arm->receive(std::chrono::milliseconds(5));
    ASSERT_FALSE(received);
    EXPECT_EQ(received.error(), CanErr::TIMEOUT);
}

TEST(TransportTests, ConcurrentTxRxDoesNotCrossChannels) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });

    for(int i = 0; i < 20; ++i) {
        bus->push_rx(frame(i % 2 == 0 ? 0x01 : 0x20));
    }

    std::size_t arm_count = 0;
    std::size_t eef_count = 0;
    std::thread arm_thread([&]() {
        for(int i = 0; i < 10; ++i) {
            auto received = arm->receive(std::chrono::milliseconds(20));
            if(received && received->id == 0x01) ++arm_count;
            (void)arm->send(frame(0x01));
        }
    });
    std::thread eef_thread([&]() {
        for(int i = 0; i < 10; ++i) {
            auto received = eef->receive(std::chrono::milliseconds(20));
            if(received && received->id == 0x20) ++eef_count;
            (void)eef->send(frame(0x20));
        }
    });

    arm_thread.join();
    eef_thread.join();

    EXPECT_EQ(arm_count, 10u);
    EXPECT_EQ(eef_count, 10u);
    EXPECT_EQ(bus->tx_size(), 20u);
}

TEST(TransportTests, AcquireCanChannelSameLogicalBusReusesPhysicalBus) {
    std::atomic<int> create_count{ 0 };
    auto creator = [&]() -> std::shared_ptr<CanBus> {
        create_count.fetch_add(1);
        auto bus = std::make_shared<MockCanBus>();
        (void)bus->open();
        return bus;
    };

    auto first = acquire_can_channel(
        "registry_same_logical",
        can_resource("can-registry-same"),
        creator,
        { CanFilter{ 0x01, 0x7FF } });
    auto second = acquire_can_channel(
        "registry_same_logical",
        can_resource("can-registry-same"),
        creator,
        { CanFilter{ 0x02, 0x7FF } });

    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first->get(), second->get());
    EXPECT_EQ(create_count.load(), 1);
}

TEST(TransportTests, BusRegistryRejectsLogicalBusTypeMismatchThroughConsumerApis) {
    auto first = acquire_can_channel(
        "registry_type_mismatch",
        can_resource("can-registry-type-mismatch"),
        []() -> std::shared_ptr<CanBus> {
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });
    ASSERT_TRUE(first);

    SerialBusConfig serial;
    serial.serial_port = "/dev/ttyACM-registry-type-mismatch";
    auto second = acquire_serial_bus_client("registry_type_mismatch", serial);

    ASSERT_FALSE(second);
    EXPECT_EQ(second.error(), BusRegistryErr::TYPE_MISMATCH);
}

TEST(TransportTests, BusRegistryRejectsCrossKindOwnershipOfSamePhysicalResource) {
    auto can = can_resource("/dev/ttyACM-registry-shared");
    can.ownership_key = "tty:/dev/ttyACM-registry-shared";
    auto first = acquire_can_channel(
        "registry_tty_can_owner",
        can,
        []() -> std::shared_ptr<CanBus> {
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });
    ASSERT_TRUE(first);

    SerialBusConfig serial;
    serial.serial_port = "/dev/ttyACM-registry-shared";
    auto second = acquire_serial_bus_client("registry_tty_serial_owner", serial);

    ASSERT_FALSE(second);
    EXPECT_EQ(second.error(), BusRegistryErr::PHYSICAL_RESOURCE_CONFLICT);
}

TEST(TransportTests, BusRegistryDifferentBusesAreIndependent) {
    std::atomic<int> create_count{ 0 };
    auto first = acquire_can_channel(
        "registry_independent_a",
        can_resource("can-registry-a"),
        [&]() -> std::shared_ptr<CanBus> {
            create_count.fetch_add(1);
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });
    auto second = acquire_can_channel(
        "registry_independent_b",
        can_resource("can-registry-b"),
        [&]() -> std::shared_ptr<CanBus> {
            create_count.fetch_add(1);
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });

    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first->get(), second->get());
    EXPECT_EQ(create_count.load(), 2);
}

TEST(TransportTests, BusRegistryRejectsPhysicalResourceConflict) {
    auto first = acquire_can_channel(
        "registry_physical_owner_a",
        can_resource("can-registry-conflict"),
        []() -> std::shared_ptr<CanBus> {
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });
    auto second = acquire_can_channel(
        "registry_physical_owner_b",
        can_resource("can-registry-conflict"),
        []() -> std::shared_ptr<CanBus> {
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });

    ASSERT_TRUE(first);
    ASSERT_FALSE(second);
    EXPECT_EQ(second.error(), BusRegistryErr::PHYSICAL_RESOURCE_CONFLICT);
}

TEST(TransportTests, BusRegistryErrorMessageContainsLogicalAndPhysicalContext) {
    const auto resource = can_resource("can-registry-diagnostic");
    const std::string message = serial_arm::transport::bus_registry_error_message(
        BusRegistryErr::PHYSICAL_RESOURCE_CONFLICT,
        "registry_diagnostic_bus",
        resource);

    EXPECT_NE(message.find("physical_resource_conflict"), std::string::npos);
    EXPECT_NE(message.find("registry_diagnostic_bus"), std::string::npos);
    EXPECT_NE(message.find("resource_kind=can"), std::string::npos);
    EXPECT_NE(message.find("can-registry-diagnostic"), std::string::npos);
    EXPECT_NE(message.find("classic-can"), std::string::npos);
}

TEST(TransportTests, BusRegistryConcurrentAcquireDoesNotDuplicateOwnership) {
    constexpr int thread_count = 20;
    std::atomic<int> create_count{ 0 };
    std::vector<tl::expected<std::shared_ptr<serial_arm::transport::CanChannel>, BusRegistryErr>> channels(thread_count);
    std::vector<std::thread> threads;
    threads.reserve(thread_count);

    for(int i = 0; i < thread_count; ++i) {
        threads.emplace_back([&, i]() {
            channels[i] = acquire_can_channel(
                "registry_concurrent",
                can_resource("can-registry-concurrent"),
                [&]() -> std::shared_ptr<CanBus> {
                    create_count.fetch_add(1);
                    auto bus = std::make_shared<MockCanBus>();
                    (void)bus->open();
                    return bus;
                });
        });
    }

    for(auto& thread : threads) thread.join();

    for(const auto& channel : channels) ASSERT_TRUE(channel);
    EXPECT_EQ(create_count.load(), 1);
}

TEST(TransportTests, BusRegistryReleasesResourceAfterLastCanChannelEnds) {
    {
        auto first = acquire_can_channel(
            "registry_release_a",
            can_resource("can-registry-release"),
            []() -> std::shared_ptr<CanBus> {
                auto bus = std::make_shared<MockCanBus>();
                (void)bus->open();
                return bus;
            });
        ASSERT_TRUE(first);
    }

    auto second = acquire_can_channel(
        "registry_release_b",
        can_resource("can-registry-release"),
        []() -> std::shared_ptr<CanBus> {
            auto bus = std::make_shared<MockCanBus>();
            (void)bus->open();
            return bus;
        });

    ASSERT_TRUE(second);
}

TEST(TransportTests, ReleasingOneCanChannelDoesNotCloseSharedRegisteredBus) {
    std::weak_ptr<MockCanBus> physical_bus;
    auto creator = [&]() -> std::shared_ptr<CanBus> {
        auto bus = std::make_shared<MockCanBus>();
        (void)bus->open();
        physical_bus = bus;
        return bus;
    };

    auto first_channel = acquire_can_channel(
        "registry_channel_owner_lifetime",
        can_resource("can-registry-channel-owner-lifetime"),
        creator,
        { CanFilter{ 0x01, 0x7FF } });
    auto second_channel = acquire_can_channel(
        "registry_channel_owner_lifetime",
        can_resource("can-registry-channel-owner-lifetime"),
        creator,
        { CanFilter{ 0x02, 0x7FF } });
    ASSERT_TRUE(first_channel);
    ASSERT_TRUE(second_channel);

    first_channel->reset();
    auto bus = physical_bus.lock();
    ASSERT_TRUE(bus);
    EXPECT_TRUE(bus->is_open());
    ASSERT_TRUE((*second_channel)->send(frame(0x02)));
    EXPECT_EQ(bus->tx_size(), 1u);
}

TEST(TransportTests, TwoFakeDriversShareRegisteredCanBusWithIndependentChannels) {
    std::atomic<int> create_count{ 0 };
    std::shared_ptr<MockCanBus> physical_bus;
    auto creator = [&]() -> std::shared_ptr<CanBus> {
        create_count.fetch_add(1);
        physical_bus = std::make_shared<MockCanBus>();
        (void)physical_bus->open();
        return physical_bus;
    };

    auto arm_channel = acquire_can_channel(
        "registry_fake_drivers",
        can_resource("can-registry-fake-drivers"),
        creator,
        { CanFilter{ 0x01, 0x7FF } },
        8);
    auto tool_channel = acquire_can_channel(
        "registry_fake_drivers",
        can_resource("can-registry-fake-drivers"),
        creator,
        { CanFilter{ 0x20, 0x7FF } },
        2);
    ASSERT_TRUE(arm_channel);
    ASSERT_TRUE(tool_channel);

    FakeCanDriver arm(*arm_channel);
    FakeCanDriver tool(*tool_channel);

    ASSERT_EQ(create_count.load(), 1);
    ASSERT_TRUE(physical_bus);

    for(std::uint8_t i = 1; i <= 4; ++i) {
        CanFrame tool_frame = frame(0x20);
        tool_frame.data[0] = i;
        physical_bus->push_rx(tool_frame);
    }
    physical_bus->push_rx(frame(0x01));

    auto arm_frame = arm.receive(std::chrono::milliseconds(20));
    ASSERT_TRUE(arm_frame);
    EXPECT_EQ(arm_frame->id, 0x01u);

    const auto arm_stats = arm.diagnostics();
    const auto tool_stats = tool.diagnostics();
    EXPECT_EQ(arm_stats.received_frames, 1u);
    EXPECT_EQ(arm_stats.dropped_frames, 0u);
    EXPECT_EQ(tool_stats.pending_frames, 2u);
    EXPECT_EQ(tool_stats.received_frames, 4u);
    EXPECT_EQ(tool_stats.dropped_frames, 2u);

    auto first_tool_frame = tool.receive(std::chrono::milliseconds(5));
    auto second_tool_frame = tool.receive(std::chrono::milliseconds(5));
    ASSERT_TRUE(first_tool_frame);
    ASSERT_TRUE(second_tool_frame);
    EXPECT_EQ(first_tool_frame->id, 0x20u);
    EXPECT_EQ(second_tool_frame->id, 0x20u);
    EXPECT_EQ(first_tool_frame->data[0], 3u);
    EXPECT_EQ(second_tool_frame->data[0], 4u);

    ASSERT_TRUE(arm.send(frame(0x01)));
    ASSERT_TRUE(tool.send(frame(0x20)));
    EXPECT_EQ(physical_bus->tx_size(), 2u);
}

TEST(TransportTests, LongReceiveDoesNotBlockSend) {
    auto bus = std::make_shared<BlockingMockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });

    std::atomic<bool> receive_started{ false };
    std::thread receiver([&]() {
        receive_started = true;
        (void)eef->receive(std::chrono::milliseconds(100));
    });

    while(!receive_started) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const auto start = std::chrono::steady_clock::now();
    auto sent = arm->send(frame(0x01));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    receiver.join();
    ASSERT_TRUE(sent);
    EXPECT_LT(elapsed.count(), 10);
    EXPECT_EQ(bus->tx_size(), 1u);
}

TEST(TransportTests, LongEefReceiveDoesNotMonopolizeArmReceive) {
    auto bus = std::make_shared<BlockingMockCanBus>();
    ASSERT_TRUE(bus->open());
    auto arm = bus->create_channel({ CanFilter{ 0x01, 0x7FF } });
    auto eef = bus->create_channel({ CanFilter{ 0x20, 0x7FF } });

    std::atomic<bool> eef_started{ false };
    std::thread eef_thread([&]() {
        eef_started = true;
        (void)eef->receive(std::chrono::milliseconds(100));
    });

    while(!eef_started) std::this_thread::yield();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    bus->push_rx(frame(0x01));

    const auto start = std::chrono::steady_clock::now();
    auto received = arm->receive(std::chrono::milliseconds(80));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    eef_thread.join();
    ASSERT_TRUE(received);
    EXPECT_EQ(received->id, 0x01u);
    EXPECT_LT(elapsed.count(), 20);
}

TEST(TransportTests, BoundedChannelDropsOldestAndReportsDiagnostics) {
    auto bus = std::make_shared<MockCanBus>();
    ASSERT_TRUE(bus->open());
    auto pump = bus->create_channel({ CanFilter{ 0x00, 0x7FF } });
    auto bounded = bus->create_channel({ CanFilter{ 0x00, 0x7FF } }, 3);

    for(std::uint8_t i = 1; i <= 5; ++i) {
        CanFrame value = frame(0x00);
        value.data[0] = i;
        bus->push_rx(value);
        auto pumped = pump->receive(std::chrono::milliseconds(5));
        ASSERT_TRUE(pumped);
    }

    const auto stats = bounded->diagnostics();
    EXPECT_EQ(stats.pending_frames, 3u);
    EXPECT_EQ(stats.max_pending_frames, 3u);
    EXPECT_EQ(stats.received_frames, 5u);
    EXPECT_EQ(stats.dropped_frames, 2u);

    for(std::uint8_t expected = 3; expected <= 5; ++expected) {
        auto received = bounded->receive(std::chrono::milliseconds(5));
        ASSERT_TRUE(received);
        EXPECT_EQ(received->data[0], expected);
    }
}
