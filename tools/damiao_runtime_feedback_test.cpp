#include "serial_arm_hardware_damiao/damiao_motor_bus.hpp"
#include "serial_arm/core/joint_actuator_mapper.hpp"

#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>

namespace {
using namespace serial_arm;
using namespace serial_arm::transport;

void require(bool ok,const char* message) { if(!ok)throw std::runtime_error(message); }

std::uint64_t clock_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// The production registry/channel/MotorControl/DamiaoMotorBus chain is used;
// only the physical receiver is replaced. No tty is opened or robot commanded.
class FakeCanBus final : public CanBus {
public:
    tl::expected<void,CanErr> open() override { return {}; }
    void close() noexcept override {}
    bool is_open() const noexcept override { return true; }
    tl::expected<void,CanErr> send(const CanFrame& frame) override {
        if(frame.id==0x7ff && frame.data[2]==damiao::PARAM_READ_CMD) {
            auto reply=frame;reply.id=frame.data[0];
            const float value=frame.data[3]==damiao::PMAX?12.5f:
                frame.data[3]==damiao::VMAX?10.0f:torque_limits.at(frame.data[0]);
            std::memcpy(reply.data.data()+4,&value,sizeof(value));
            reply.received_ns=clock_ns();reply.receive_timing_valid=true;
            frames.push_back(reply);
        } else if(frame.id==0x7ff && frame.data[2]==damiao::PARAM_WRITE_CMD) {
            auto reply=frame;reply.id=frame.data[0];reply.received_ns=clock_ns();reply.receive_timing_valid=true;
            frames.push_back(reply);
            if(frame.data[3]==damiao::CTRL_MODE)++mode_writes;
        } else if(frame.id==0x7ff && frame.data[2]==0xcc) {
            ++refresh_requests;feedback(frame.data[0],32768,2048,2048,statuses[frame.data[0]]);
        } else if(frame.data[7]==0xfc || frame.data[7]==0xfd) {
            statuses[frame.id]=frame.data[7]==0xfc?1:0;
            feedback(frame.id,32768,2048,2048,statuses[frame.id]);
        }
        return {};
    }
    tl::expected<CanFrame,CanErr> receive(std::chrono::milliseconds timeout) override {
        ++polls;
        if(timeout.count()>0)++blocking_polls;
        if(frames.empty())return tl::make_unexpected(CanErr::TIMEOUT);
        auto frame=frames.front();frames.pop_front();return frame;
    }
    void flush() noexcept override { frames.clear(); }
    void feedback(std::uint32_t id,std::uint16_t q,std::uint16_t v,std::uint16_t tau,
                  std::uint8_t status=1,std::uint64_t stamp=0,bool timing=true) {
        CanFrame frame;frame.id=0;frame.size=8;
        frame.data={static_cast<std::uint8_t>((status<<4)|id),static_cast<std::uint8_t>(q>>8),static_cast<std::uint8_t>(q),
            static_cast<std::uint8_t>(v>>4),static_cast<std::uint8_t>((v<<4)|(tau>>8)),static_cast<std::uint8_t>(tau),0,0};
        frame.received_ns=stamp?stamp:clock_ns();frame.receive_timing_valid=timing;
        frames.push_back(frame);
    }
    void reset_counts() { polls=blocking_polls=0; }
    std::size_t polls=0,blocking_polls=0;
    std::size_t mode_writes=0,refresh_requests=0;
    std::deque<CanFrame> frames;
    std::map<std::uint32_t,std::uint8_t> statuses;
    std::map<std::uint32_t,float> torque_limits;
};

void check_joint_quantum_mapping(const HardwareCapabilities& capabilities) {
    std::vector<std::string> names;
    for(auto it=capabilities.rbegin();it!=capabilities.rend();++it)names.push_back(it->joint_name);
    const auto ordered=capabilities_for_joint_order(capabilities,names);
    require(bool(ordered),"joint capability reordering failed");
    JointActuatorMapCfg map;map.joints_count=ordered->size();
    JointCtrlCmd command;
    command.pos.assign(map.joints_count,0.0);command.vel.assign(map.joints_count,0.0);
    command.tor.assign(map.joints_count,0.0);command.kp.assign(map.joints_count,0.0);
    command.kd.assign(map.joints_count,0.0);
    for(std::size_t i=0;i<map.joints_count;++i) {
        const auto& source=capabilities[capabilities.size()-1-i];
        require((*ordered)[i].effort_resolution==source.effort_resolution &&
            (*ordered)[i].actuator_name==source.actuator_name,
            "torque quantum lost its joint identity during capability reordering");
        map.pos_ratio.push_back(2.5+i);
        map.tor_ratio.push_back(1.5+0.5*i);
        map.direction.push_back(i%2==0?-1:1);
        map.joint_zero_offset.push_back(0.0);map.actuator_zero_offset.push_back(0.0);
        command.tor[i]=(*ordered)[i].effort_resolution*map.tor_ratio[i];
    }
    JointActuatorMapper mapper;
    require(bool(mapper.configure(map)),"nonunit torque mapping configuration failed");
    const auto actuator=mapper.to_actuator_cmd(command);
    require(bool(actuator),"joint torque quantum mapping failed");
    for(std::size_t i=0;i<map.joints_count;++i)
        require(std::abs(actuator->tor[i]-map.direction[i]*(*ordered)[i].effort_resolution)<1e-14,
            "one joint torque code must map to one motor torque code independent of pos_ratio");
}

void check_axis_count(std::size_t count) {
    DamiaoBusCfg cfg;cfg.bus="offline-runtime-feedback-"+std::to_string(count);
    cfg.serial_port="/offline/damiao-runtime-"+std::to_string(count);
    cfg.refresh_state_in_read=false;
    for(std::size_t i=0;i<count;++i)
        cfg.actuators.push_back({"motor"+std::to_string(i),"joint"+std::to_string(i),
            static_cast<std::uint32_t>(i+1),0,i%2==0?"DM4340":"DM4310"});
    auto physical=std::make_shared<FakeCanBus>();
    for(std::size_t i=0;i<count;++i)physical->torque_limits[i+1]=10.0f+static_cast<float>(i);
    BusResourceDescriptor resource{BusResourceKind::CAN,cfg.serial_port,
        "damiao_usb2can|baudrate="+std::to_string(cfg.baudrate),tty_ownership_key(cfg.serial_port)};
    auto registration=acquire_can_channel(cfg.bus,resource,[&]{return physical;});
    require(bool(registration),"fake physical bus registration failed");
    DamiaoMotorBus bus;
    require(bool(bus.configure(cfg)),"hardware configuration failed");
    for(std::size_t i=0;i<count;++i) {
        const double configured_tau=i%2==0?28.0:10.0;
        require(std::abs(bus.capabilities()[i].effort_resolution-2.0*configured_tau/4095.0)<1e-14,
            "configured model torque quantum was not exposed");
    }
    require(bool(bus.connect()),"hardware parameter handshake failed");
    require(bus.capabilities().front().max_vel==10.0,"confirmed register range was not used");
    for(std::size_t i=0;i<count;++i) {
        const auto& cap=bus.capabilities()[i];const double confirmed_tau=10.0+i;
        require(cap.max_effort==confirmed_tau &&
            std::abs(cap.effort_resolution-2.0*confirmed_tau/4095.0)<1e-14,
            "confirmed TMAX did not refresh the torque quantum");
        require(cap.kp_resolution==500.0/4095.0 && cap.kd_resolution==5.0/4095.0,
            "torque resolution changed existing PD protocol resolutions");
    }
    check_joint_quantum_mapping(bus.capabilities());

    physical->reset_counts();
    for(std::size_t i=0;i<count;++i)physical->feedback(i+1,33000+i,2200+i,2100+i);
    const auto latest=clock_ns();
    physical->feedback(1,34000,2250,2150,1,latest,false);
    auto state=bus.read();
    require(bool(state),"runtime read failed");
    require(physical->blocking_polls==0,"runtime read waited for motor replies");
    require(physical->frames.empty(),"runtime read did not drain queued feedback");
    require(physical->polls==count+2,"runtime drain did not stop at first empty poll");
    require(state->feedback_received_ns.front()==latest,"latest real feedback timestamp was not retained");
    require(state->feedback_timing_valid.front()==0,"invalid feedback timing was promoted to valid");
    require(std::abs(state->pos.front()-(34000.0*25.0/65535.0-12.5))<2e-6,"position codec or latest-frame selection changed");
    require(std::abs(state->vel.front()-(2250.0*20.0/4095.0-10.0))<2e-6,"confirmed velocity codec changed");
    for(auto online:state->online)require(online==1,"fresh motor was marked offline");

    const auto before=*state;
    physical->reset_counts();
    auto empty=bus.read();
    require(bool(empty),"empty runtime poll failed");
    require(physical->blocking_polls==0 && physical->polls==1,"empty runtime poll blocked or busy-looped");
    require(empty->feedback_received_ns==before.feedback_received_ns && empty->pos==before.pos &&
        empty->feedback_timing_valid==before.feedback_timing_valid,"cached feedback was rewritten as fresh");
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    auto stale=bus.read();
    require(bool(stale),"stale read must preserve timestamps for Safety");
    for(auto online:stale->online)require(online==0,"original feedback timeout was bypassed");
    require(stale->feedback_received_ns==before.feedback_received_ns,"stale timestamp was refreshed");
    std::cout<<"PASS "<<count<<" axes: nonblocking drain, real timing, empty cache and unchanged stale timeout\n";
    std::cout<<"PASS "<<count<<" axes: configured/confirmed torque quantum, joint order and nonunit torque mapping\n";
    if(count==1) {
        require(bool(bus.activate()),"explicit activation or MIT mode handshake changed");
        require(physical->mode_writes==1,"activation skipped MIT mode confirmation");
        physical->reset_counts();
        require(bool(bus.read()) && physical->blocking_polls==0,"first read after startup blocked");
        require(bool(bus.deactivate()),"explicit disable/status confirmation changed");
        cfg.refresh_state_in_read=true;
        require(bool(bus.configure(cfg)) && bool(bus.connect()),"explicit-refresh configuration failed");
        const auto requests=physical->refresh_requests;
        auto refreshed=bus.read();
        require(bool(refreshed) && physical->refresh_requests==requests+1 && refreshed->online.front(),
            "explicit refresh no longer performs its target-specific query");
        std::cout<<"PASS explicit activation, confirmed MIT switch, disable and requested synchronous refresh\n";
    }
}
}

int main() {
    try {
        require(ActuatorCapability{}.effort_resolution==0.0,"undeclared torque resolution must remain unknown");
        check_axis_count(1);check_axis_count(6);
    }
    catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
