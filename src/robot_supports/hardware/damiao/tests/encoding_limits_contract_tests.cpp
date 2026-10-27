#include "serial_arm_hardware_damiao/damiao_motor_bus.hpp"
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace serial_arm;
using namespace std::chrono_literals;
namespace {
void check(bool value, const char* message) { if(!value) throw std::runtime_error(message); }
bool close(float a, float b, float tolerance=1e-5f) { return std::abs(a-b)<=tolerance; }
struct Request { unsigned id; std::array<unsigned char,8> data; };

// Virtual serial/CAN adapter only; never opens a physical device.
class Adapter {
public:
    enum Fault { None, Missing, WrongOnly, Zero, Negative, Nan, Infinity, Overflow };
    const std::array<damiao::LimitParam,2> limits{{{12.5f,10.0f,28.0f},{6.25f,16.0f,12.0f}}};
    std::atomic<int> fault{None};
    std::atomic<bool> interleave{false};
    int master;
    Adapter() : master(::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK)) {
        check(master>=0 && !::grantpt(master) && !::unlockpt(master), "create virtual serial adapter");
        worker=std::thread([this]{loop();});
    }
    ~Adapter() { running=false; if(worker.joinable())worker.join(); ::close(master); }
    std::string device() const { return ::ptsname(master); }
    std::vector<Request> requests() { std::lock_guard<std::mutex> guard(lock); return sent; }
private:
    std::atomic<bool> running{true};
    std::thread worker;
    std::mutex lock;
    std::vector<Request> sent;
    std::array<unsigned char,3> status{};
    std::array<std::array<unsigned,3>,3> state{{{{32768,2048,2048}},{{32768,2048,2048}},{{32768,2048,2048}}}};
    void reply(unsigned id,const std::array<unsigned char,8>& data) {
        std::array<unsigned char,16> packet{0xaa,0x11,8,0,0,0,0};
        packet[3]=id&255;packet[4]=(id>>8)&255;
        std::copy(data.begin(),data.end(),packet.begin()+7);packet[15]=0x55;
        if(::write(master,packet.data(),packet.size())!=static_cast<ssize_t>(packet.size()))running=false;
    }
    void feedback(unsigned id) {
        const auto& s=state[id];
        reply(0,{{static_cast<unsigned char>((status[id]<<4)|id),static_cast<unsigned char>(s[0]>>8),
                  static_cast<unsigned char>(s[0]),static_cast<unsigned char>(s[1]>>4),
                  static_cast<unsigned char>((s[1]<<4)|(s[2]>>8)),static_cast<unsigned char>(s[2]),0,0}});
    }
    void respond(const Request& request) {
        const auto& data=request.data;
        if(request.id==0x7ff) {
            const unsigned id=data[0]|(data[1]<<8);
            if(id<1||id>2)return;
            if(data[2]==0xcc) { feedback(id);return; }
            if(data[2]==damiao::PARAM_WRITE_CMD) { reply(id,data);return; }
            if(data[2]!=damiao::PARAM_READ_CMD)return;
            auto response=data;
            const int problem=(id==2 && data[3]==damiao::VMAX)?fault.load():None;
            if(problem==Missing)return;
            if(interleave||problem==WrongOnly) {
                auto wrong=response;wrong[2]=damiao::PARAM_WRITE_CMD;reply(id,wrong);
                wrong=response;wrong[3]=damiao::UV_Value;reply(id,wrong);
                wrong=response;wrong[0]=id==1?2:1;reply(wrong[0],wrong);
                feedback(id);
            }
            if(problem==WrongOnly)return;
            const auto& range=limits[id-1];
            float value=data[3]==damiao::PMAX?range.q_max:
                        data[3]==damiao::VMAX?range.dq_max:range.tau_max;
            if(problem==Zero)value=0;
            if(problem==Negative)value=-1;
            if(problem==Nan)value=std::numeric_limits<float>::quiet_NaN();
            if(problem==Infinity)value=std::numeric_limits<float>::infinity();
            if(problem==Overflow)value=std::numeric_limits<float>::max();
            std::memcpy(response.data()+4,&value,sizeof(value));reply(id,response);
        } else if(request.id>=1&&request.id<=2) {
            const auto id=request.id;
            bool management=true;
            for(unsigned i=0;i<7;++i)management&=data[i]==0xff;
            if(management) {
                if(data[7]==0xfc)status[id]=1;
                if(data[7]==0xfd)status[id]=0;
            } else {
                state[id]={{unsigned(data[0]<<8)|data[1],unsigned(data[2]<<4)|(data[3]>>4),
                            unsigned((data[6]&0xf)<<8)|data[7]}};
            }
            feedback(id);
        }
    }
    void loop() {
        std::vector<unsigned char> pending;
        while(running) {
            pollfd p{master,POLLIN,0};if(::poll(&p,1,2)<=0)continue;
            unsigned char bytes[4096];const auto n=::read(master,bytes,sizeof(bytes));
            if(n<=0)continue;pending.insert(pending.end(),bytes,bytes+n);
            while(pending.size()>=30) {
                if(pending[0]!=0x55||pending[1]!=0xaa){pending.erase(pending.begin());continue;}
                Request request{unsigned(pending[13])|(unsigned(pending[14])<<8),{}};
                std::copy(pending.begin()+21,pending.begin()+29,request.data.begin());
                {std::lock_guard<std::mutex> guard(lock);sent.push_back(request);}
                respond(request);pending.erase(pending.begin(),pending.begin()+30);
            }
        }
    }
};

DamiaoBusCfg config(const Adapter& adapter) {
    DamiaoBusCfg cfg;
    cfg.bus="limits-"+adapter.device();cfg.serial_port=adapter.device();cfg.baudrate=115200;
    cfg.feedback_timeout_s=.05;cfg.activation_retries=1;cfg.startup_read_cycles=1;
    cfg.actuators={{"motor-a","joint-a",1,0,"DM4340"},{"motor-b","joint-b",2,0,"DM4340"}};
    return cfg;
}
void only_read_requests(Adapter& adapter) {
    for(const auto& request:adapter.requests())
        check(request.id==0x7ff && request.data[2]==damiao::PARAM_READ_CMD &&
              request.data[3]>=damiao::PMAX && request.data[3]<=damiao::TMAX,
              "connect sent an enable/control/register-write command");
}
void check_ranges(const DamiaoMotorBus& bus,const Adapter& adapter) {
    for(unsigned i=0;i<2;++i) {
        const auto& limit=adapter.limits[i];const auto& info=bus.get_actuator_info()[i];
        const auto& cap=bus.capabilities()[i];
        check(close(info.q_max,limit.q_max)&&close(info.dq_max,limit.dq_max)&&close(info.tau_max,limit.tau_max),"info not updated per axis");
        check(close(cap.min_pos,-limit.q_max)&&close(cap.max_pos,limit.q_max)&&close(cap.max_vel,limit.dq_max)&&close(cap.max_effort,limit.tau_max),"capability not updated per axis");
    }
}
void local_limits_contract() {
    damiao::Motor a(damiao::DM4340,1,0),b(damiao::DM4340,2,0);
    a.receive_data(1,2,3,1,123456,true);
    damiao::MotorControl::change_motor_limit(a,6.25f,10,20);
    check(a.get_limit_param().dq_max==10 && a.get_limit_param().q_max==6.25f,"existing Motor kept old limit");
    check(a.get_state_seq()==0 && !a.timing_valid() && a.get_velocity()==0,"old decoded feedback survived scale change");
    damiao::Motor c(damiao::DM4340,3,0);
    check(b.get_limit_param().dq_max==8 && c.get_limit_param().dq_max==8 && damiao::limit_param[damiao::DM4340].dq_max==8,"same-type motor/default was contaminated");
    for(float invalid:{0.0f,-1.0f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity(),std::numeric_limits<float>::max()}) {
        bool rejected=false;try { damiao::MotorControl::change_motor_limit(a,6.25f,invalid,20); }
        catch(const std::invalid_argument&) { rejected=true; }
        check(rejected&&a.get_limit_param().dq_max==10,"invalid limit changed instance");
    }
}
void connect_and_codec_contract() {
    Adapter adapter;adapter.interleave=true;
    DamiaoMotorBus bus;check(bool(bus.configure(config(adapter))),"configure");
    check(bus.capabilities()[0].max_vel==8 && bus.capabilities()[1].max_vel==8,"nominal defaults changed");
    check(bool(bus.connect()),"valid per-axis readback rejected");
    only_read_requests(adapter);check_ranges(bus,adapter);
    check(adapter.requests().size()==6,"connect did not read all three registers per axis");
    auto initial=bus.read();check(bool(initial),"connected read failed");
    check(initial->online==std::vector<std::uint8_t>({0,0})&&initial->feedback_received_ns==std::vector<std::uint64_t>({0,0}),"old default-scale feedback visible after commit");
    check(bool(bus.connect()) && adapter.requests().size()==6,"connected no-op reread registers");
    check(bool(bus.activate()),"virtual activation failed");
    ActuatorCtrlCmd cmd;cmd.pos={5,3};cmd.vel={9,12};cmd.tor={22,8};cmd.kp={0,0};cmd.kd={0,0};
    check(bool(bus.write(cmd)),"confirmed velocity/effort within range rejected");
    // Drain the virtual adapter's command responses (read need not send 0xCC).
    auto state=bus.read();check(bool(state),"read encoded echo");
    const auto deadline=std::chrono::steady_clock::now()+100ms;
    while((!close(state->vel[0],cmd.vel[0],.02f)||!close(state->vel[1],cmd.vel[1],.02f)) &&
          std::chrono::steady_clock::now()<deadline) {
        state=bus.read();check(bool(state),"read pending encoded echo");
    }
    const auto requests=adapter.requests();
    for(unsigned i=0;i<2;++i) {
        const auto& range=adapter.limits[i];
        const auto found=std::find_if(requests.rbegin(),requests.rend(),[i](const Request& request) {
            return request.id==i+1 && !(request.data[0]==0xff&&request.data[1]==0xff);
        });
        check(found!=requests.rend(),"missing outgoing MIT command");
        const auto& data=found->data;
        const unsigned q_code=unsigned(data[0]<<8)|data[1];
        const unsigned v_code=unsigned(data[2]<<4)|(data[3]>>4);
        const unsigned t_code=unsigned((data[6]&0xf)<<8)|data[7];
        const auto expected=[](float value,float maximum,unsigned bits) {
            return static_cast<unsigned>(((value+maximum)/(2.0f*maximum))*((1u<<bits)-1));
        };
        check(q_code==expected(cmd.pos[i],range.q_max,16),"outgoing position range differs from device readback");
        check(v_code==expected(cmd.vel[i],range.dq_max,12),"outgoing velocity range differs from device readback");
        check(t_code==expected(cmd.tor[i],range.tau_max,12),"outgoing effort range differs from device readback");
        check(close(state->pos[i],cmd.pos[i],2*range.q_max/65535+1e-5f),"position send/decode scales disagree");
        check(close(state->vel[i],cmd.vel[i],2*range.dq_max/4095+1e-5f),"velocity send/decode scales disagree");
        check(close(state->tor[i],cmd.tor[i],2*range.tau_max/4095+1e-5f),"torque send/decode scales disagree");
    }
    cmd.vel[0]=10.1;const auto invalid=bus.write(cmd);
    check(!invalid&&invalid.error()==MotorBusErr::INVALID_CMD,"command above confirmed limit not rejected");
    check(bool(bus.deactivate()),"virtual deactivation failed");
}
void rejected_readback_contract() {
    for(const auto fault:{Adapter::Missing,Adapter::WrongOnly,Adapter::Zero,Adapter::Negative,Adapter::Nan,Adapter::Infinity,Adapter::Overflow}) {
        Adapter adapter;adapter.fault=fault;
        DamiaoMotorBus bus;check(bool(bus.configure(config(adapter))),"configure failure fixture");
        const auto start=std::chrono::steady_clock::now();
        const auto result=bus.connect();
        check(!result&&result.error()==MotorBusErr::READ_FAILED,"unconfirmed/invalid readback reported connected");
        check(std::chrono::steady_clock::now()-start<1500ms,"parameter failure did not time out");
        check(bus.capabilities()[0].max_vel==8 && bus.capabilities()[1].max_vel==8,"partially committed per-axis capabilities");
        check(bus.get_actuator_info()[0].dq_max==8 && bus.get_actuator_info()[1].dq_max==8,"partially committed per-axis info");
        const auto active=bus.activate();check(!active&&active.error()==MotorBusErr::NOT_CONNECTED,"failed connect permitted activate");
        const auto read=bus.read();check(!read&&read.error()==MotorBusErr::NOT_CONNECTED,"failed connect permitted read");
        only_read_requests(adapter);
        adapter.fault=Adapter::None;
        check(bool(bus.connect()),"failed connect could not retry");only_read_requests(adapter);check_ranges(bus,adapter);
    }
}
} // namespace
int main() {
    try {
        local_limits_contract();connect_and_codec_contract();rejected_readback_contract();
        std::cout<<"ENCODING_LIMITS_READBACK_ATOMICITY_AND_CODEC_PASS\n";return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
