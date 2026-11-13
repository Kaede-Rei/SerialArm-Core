#!/usr/bin/env python3
"""Check real Loader/BusRegistry teardown with an entirely in-memory backend.

Builds a temporary DSO whose shared transport control block outlives its backend
through the Core registry's weak reference. No real hardware plugin is loaded,
and the test backend never opens a device or creates a receiver thread.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

PLUGIN = r'''
#include "serial_arm/hardware/motor_bus.hpp"
#include "serial_arm/transport/bus.hpp"
#include <new>

extern "C" void lifetime_test_event(unsigned);
namespace {
template<class T> struct LifetimeAllocator {
    using value_type=T;
    LifetimeAllocator()=default;
    template<class U> LifetimeAllocator(const LifetimeAllocator<U>&) noexcept {}
    T* allocate(std::size_t n){return static_cast<T*>(::operator new(n*sizeof(T)));}
    void deallocate(T* p,std::size_t) noexcept {lifetime_test_event(3);::operator delete(p);}
};
class MemoryTransport final:public serial_arm::transport::CanBus {
public:
    ~MemoryTransport() override {lifetime_test_event(2);}
    tl::expected<void,serial_arm::transport::CanErr> open() override{return {};}
    void close() noexcept override{}
    bool is_open() const noexcept override{return true;}
    tl::expected<void,serial_arm::transport::CanErr> send(const serial_arm::transport::CanFrame&) override{return {};}
    tl::expected<serial_arm::transport::CanFrame,serial_arm::transport::CanErr> receive(std::chrono::milliseconds) override {
        return tl::make_unexpected(serial_arm::transport::CanErr::TIMEOUT);
    }
    void flush() noexcept override{}
};
class MemoryBackend final:public serial_arm::MotorBus {
    std::shared_ptr<serial_arm::transport::CanChannel> channel_;
    serial_arm::HardwareCapabilities capabilities_;
public:
    MemoryBackend(){lifetime_test_event(0);}
    ~MemoryBackend() override {cleanup();lifetime_test_event(1);}
    tl::expected<void,serial_arm::MotorBusErr> configure(const std::string&) override{return {};}
    tl::expected<void,serial_arm::MotorBusErr> connect() override {
        using namespace serial_arm::transport;
        const auto channel=acquire_can_channel("loader-lifetime",
            {BusResourceKind::CAN,"in-memory","lifetime-test","in-memory:loader-lifetime"},
            []{return std::allocate_shared<MemoryTransport>(LifetimeAllocator<MemoryTransport>{});});
        if(!channel)return tl::make_unexpected(serial_arm::MotorBusErr::OPEN_FAILED);
        channel_=*channel;return {};
    }
    tl::expected<serial_arm::ActuatorState,serial_arm::MotorBusErr> read() override{return serial_arm::ActuatorState{};}
    tl::expected<void,serial_arm::MotorBusErr> activate() override{return {};}
    tl::expected<void,serial_arm::MotorBusErr> write(const serial_arm::ActuatorCtrlCmd&) override{return {};}
    tl::expected<void,serial_arm::MotorBusErr> stop() override{return {};}
    tl::expected<void,serial_arm::MotorBusErr> deactivate() override{return {};}
    tl::expected<void,serial_arm::MotorBusErr> recover() override{return {};}
    const serial_arm::HardwareCapabilities& capabilities() const noexcept override{return capabilities_;}
    void cleanup() noexcept override{channel_.reset();}
    std::size_t size() const noexcept override{return 0;}
};
}
extern "C" std::uint64_t serial_arm_motor_bus_abi(){return serial_arm::motor_bus_abi_fingerprint;}
extern "C" serial_arm::MotorBus* create_motor_bus(){return new MemoryBackend;}
extern "C" void destroy_motor_bus(serial_arm::MotorBus* bus){delete bus;}
'''

HOST = r'''
#include "serial_arm/hardware/hardware_loader.hpp"
#include <dlfcn.h>
#include <cstdlib>
#include <iostream>
namespace {unsigned events[4]{};}
extern "C" void lifetime_test_event(unsigned event){if(event<4)++events[event];}
int main(int argc,char** argv) {
    if(argc!=3)return 1;
    // Registered before the registry's first use, so this observes its FINAL
    // expired weak control block being destroyed during normal process exit.
    std::atexit([]{if(events[3]!=2)std::_Exit(10);});
    serial_arm::HardwareLoader loader;
    for(unsigned round=0;round<2;++round) {
        auto loaded=loader.load(argv[1],argv[2]);if(!loaded)return 2;
        auto bus=std::move(*loaded);if(!bus->connect())return 3;
        if(events[3]!=round)return 4;
        bus->cleanup();bus.reset();
        if(events[0]!=round+1 || events[1]!=round+1 || events[2]!=round+1)return 5;
        // Closing the live backend must close its resource, while the DSO
        // stays mapped for the registry's still-existing weak control block.
        void* mapped=dlopen(argv[1],RTLD_NOW|RTLD_NOLOAD);
        if(!mapped)return 6;
        dlclose(mapped);
    }
    std::cout<<"backend/resource cleanup, reload and delayed weak-block teardown passed\n";
    return 0;
}
'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cxx', default='c++', help='Existing C++ compiler')
    args = parser.parse_args()
    include = ['-I', str(ROOT / 'src/serial_arm/core/include'),
               '-I', str(ROOT / 'src/serial_arm/core/third_lib/tl/include')]
    flags = ['-std=c++17', '-O1', '-Wall', '-Wextra', '-Wpedantic', *include]
    with tempfile.TemporaryDirectory(prefix='serialarm-loader-lifetime-') as folder:
        temporary = Path(folder)
        plugin_source = temporary / 'memory_backend.cpp'
        host_source = temporary / 'host.cpp'
        plugin_source.write_text(PLUGIN)
        host_source.write_text(HOST)
        config = temporary / 'memory.yaml'
        config.write_text('backend: in_memory_only\n')
        transport = temporary / 'liblifetime_transport.so'
        plugin = temporary / 'libmemory_backend.so'
        host = temporary / 'loader_lifetime_test'
        shared = ['-shared', '-fPIC', '-fno-gnu-unique']
        subprocess.run([args.cxx, *flags, *shared,
                        str(ROOT / 'src/serial_arm/core/src/transport/bus.cpp'),
                        '-pthread', '-o', str(transport)], check=True)
        subprocess.run([args.cxx, *flags, *shared, str(plugin_source), str(transport),
                        f'-Wl,-rpath,{temporary}', '-pthread', '-o', str(plugin)], check=True)
        subprocess.run([args.cxx, *flags, '-rdynamic', str(host_source),
                        str(ROOT / 'src/serial_arm/core/src/hardware/hardware_loader.cpp'),
                        str(transport), f'-Wl,-rpath,{temporary}', '-lyaml-cpp', '-ldl',
                        '-pthread', '-o', str(host)], check=True)
        subprocess.run([str(host), str(plugin), str(config)], check=True, timeout=15)


if __name__ == '__main__':
    main()
