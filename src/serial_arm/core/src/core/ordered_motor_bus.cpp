#include "serial_arm/hardware/motor_bus.hpp"
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <unordered_set>

namespace serial_arm {
namespace {
tl::expected<std::vector<std::size_t>,MotorBusErr> ordering(const HardwareCapabilities& caps,const std::vector<std::string>& names) {
    if(caps.size()!=names.size())return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    std::vector<std::size_t> order(names.size());std::iota(order.begin(),order.end(),0);
    if(std::all_of(caps.begin(),caps.end(),[](const auto& cap){return cap.joint_name.empty();}))return order;
    std::unordered_map<std::string,std::size_t> indices;
    for(std::size_t i=0;i<caps.size();++i)
        if(caps[i].joint_name.empty()||!indices.emplace(caps[i].joint_name,i).second)return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    std::unordered_set<std::string> expected;
    for(std::size_t i=0;i<names.size();++i){
        const auto found=indices.find(names[i]);
        if(found==indices.end()||!expected.insert(names[i]).second)return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        order[i]=found->second;
    }
    return order;
}
class OrderedMotorBus final:public MotorBus {
    std::unique_ptr<MotorBus> bus_;
    HardwareCapabilities caps_;
    std::vector<std::size_t> order_;
    std::vector<std::vector<std::size_t>> cycles_;
    ActuatorCtrlCmd command_;
    tl::expected<void,MotorBusErr> refresh_capabilities() {
        const auto& current=bus_->capabilities();
        if(current.size()!=order_.size())return tl::make_unexpected(MotorBusErr::INVALID_CFG);
        HardwareCapabilities next;next.reserve(order_.size());
        for(std::size_t i=0;i<order_.size();++i){
            const auto& capability=current[order_[i]];
            // The command/feedback permutation was fixed at configure time.
            // A capability refresh must not silently change joint identity
            if(capability.joint_name!=caps_[i].joint_name)
                return tl::make_unexpected(MotorBusErr::INVALID_CFG);
            next.push_back(capability);
        }
        caps_=std::move(next);
        return {};
    }
    template<typename T> void reorder(std::vector<T>& values) const {
        for(const auto& cycle:cycles_){
            const T saved=values[cycle.front()];
            for(std::size_t i=0;i+1<cycle.size();++i)values[cycle[i]]=values[cycle[i+1]];
            values[cycle.back()]=saved;
        }
    }
public:
    OrderedMotorBus(std::unique_ptr<MotorBus> bus,HardwareCapabilities caps,std::vector<std::size_t> order)
        :bus_(std::move(bus)),caps_(std::move(caps)),order_(std::move(order)){
        std::vector<bool> seen(order_.size(),false);
        for(std::size_t i=0;i<order_.size();++i){
            if(seen[i])continue;
            std::vector<std::size_t> cycle;
            for(std::size_t j=i;!seen[j];j=order_[j]){seen[j]=true;cycle.push_back(j);}
            if(cycle.size()>1)cycles_.push_back(std::move(cycle));
        }
        command_.pos.resize(size());command_.vel.resize(size());command_.tor.resize(size());command_.kp.resize(size());command_.kd.resize(size());
    }
    tl::expected<void,MotorBusErr> configure(const std::string&) override {return tl::make_unexpected(MotorBusErr::INVALID_CFG);}
    tl::expected<void,MotorBusErr> connect() override {
        const auto result=bus_->connect();if(!result)return result;
        return refresh_capabilities();
    }
    tl::expected<void,MotorBusErr> select_control_path(ControlPath path, const ActuatorVector& speed_limit = {}) override {
        if(path!=ControlPath::CSP) return bus_->select_control_path(path,{});
        if(speed_limit.size()!=size()) return tl::make_unexpected(MotorBusErr::INVALID_CMD);
        ActuatorVector reordered(size());
        for(std::size_t i=0;i<size();++i) reordered[order_[i]]=speed_limit[i];
        return bus_->select_control_path(path,reordered);
    }
    tl::expected<void,MotorBusErr> activate() override {return bus_->activate();}
    tl::expected<ActuatorState,MotorBusErr> read() override {
        auto result=bus_->read();if(!result)return result;
        auto& state=*result;
        if(state.pos.size()!=size()||state.vel.size()!=size()||state.tor.size()!=size()||state.online.size()!=size()||state.enabled.size()!=size()||state.err_code.size()!=size())
            return tl::make_unexpected(MotorBusErr::READ_FAILED);
        if(!state.feedback_timing_valid.empty()) {
            if(state.feedback_timing_valid.size()!=size()) return tl::make_unexpected(MotorBusErr::READ_FAILED);
            reorder(state.feedback_timing_valid);
        }
        if(!state.feedback_received_ns.empty()) {
            if(state.feedback_received_ns.size()!=size()) return tl::make_unexpected(MotorBusErr::READ_FAILED);
            reorder(state.feedback_received_ns);
        }
        reorder(state.pos);reorder(state.vel);reorder(state.tor);reorder(state.online);reorder(state.enabled);reorder(state.err_code);
        return result;
    }
    tl::expected<void,MotorBusErr> write(const ActuatorCtrlCmd& command) override {
        if(command.pos.size()!=size()||command.vel.size()!=size()||command.tor.size()!=size()||command.kp.size()!=size()||command.kd.size()!=size())
            return tl::make_unexpected(MotorBusErr::INVALID_CMD);
        for(std::size_t i=0;i<size();++i){const auto j=order_[i];command_.pos[j]=command.pos[i];command_.vel[j]=command.vel[i];command_.tor[j]=command.tor[i];command_.kp[j]=command.kp[i];command_.kd[j]=command.kd[i];}
        command_.path=command.path;
        if(command.path==ControlPath::CSP) {
            if(command.position_speed_limit.size()!=size()) return tl::make_unexpected(MotorBusErr::INVALID_CMD);
            command_.position_speed_limit.resize(size());
            for(std::size_t i=0;i<size();++i)command_.position_speed_limit[order_[i]]=command.position_speed_limit[i];
        } else command_.position_speed_limit.clear();
        return bus_->write(command_);
    }
    tl::expected<void,MotorBusErr> stop() override {return bus_->stop();}
    tl::expected<void,MotorBusErr> deactivate() override {return bus_->deactivate();}
    tl::expected<void,MotorBusErr> recover() override {
        const auto result=bus_->recover();if(!result)return result;
        return refresh_capabilities();
    }
    void cleanup() noexcept override {bus_->cleanup();}
    const HardwareCapabilities& capabilities() const noexcept override {return caps_;}
    std::size_t size() const noexcept override {return order_.size();}
};
}
tl::expected<HardwareCapabilities,MotorBusErr> capabilities_for_joint_order(const HardwareCapabilities& caps,const std::vector<std::string>& names){
    const auto order=ordering(caps,names);if(!order)return tl::make_unexpected(order.error());
    HardwareCapabilities result;result.reserve(caps.size());for(const auto i:*order)result.push_back(caps[i]);return result;
}
tl::expected<std::unique_ptr<MotorBus>,MotorBusErr> align_motor_bus(std::unique_ptr<MotorBus> bus,const std::vector<std::string>& names){
    if(!bus)return tl::make_unexpected(MotorBusErr::INVALID_CFG);
    const auto order=ordering(bus->capabilities(),names);if(!order)return tl::make_unexpected(order.error());
    bool aligned=true;for(std::size_t i=0;i<order->size();++i)aligned&=(*order)[i]==i;
    if(aligned)return bus;
    auto caps=capabilities_for_joint_order(bus->capabilities(),names);
    return std::unique_ptr<MotorBus>(new OrderedMotorBus(std::move(bus),std::move(*caps),*order));
}
} // namespace serial_arm
