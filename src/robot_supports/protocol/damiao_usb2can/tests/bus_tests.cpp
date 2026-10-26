#include "serial_arm_protocol_damiao_usb2can/bus.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace serial_arm;
using namespace std::chrono_literals;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
std::array<std::uint8_t,16> packet(unsigned index){
    std::array<std::uint8_t,16> b{0xaa,0x11,8,1,0,0,0};
    b[7]=index&255;b[8]=(index>>8)&255;b[15]=0x55;return b;
}
int main(){
    int master=::posix_openpt(O_RDWR|O_NOCTTY);
    try{
        check(master>=0&&::grantpt(master)==0&&::unlockpt(master)==0,"create virtual serial");
        auto bus=std::make_shared<protocol::damiao_usb2can::DamiaoUsbCanBus>(protocol::damiao_usb2can::Config{::ptsname(master),115200});
        check(bool(bus->open()),"open virtual transport");auto channel=bus->create_channel();
        auto b=packet(1);check(::write(master,b.data(),b.size())==16,"send virtual feedback");
        std::this_thread::sleep_for(40ms);
        auto first=channel->receive(10ms);check(bool(first)&&first->received_ns!=0,"receive timestamp present");
        const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        check(now-first->received_ns>=25000000,"delayed decode retains transport age");
        b=packet(2);check(::write(master,b.data(),8)==8,"send fragment");std::this_thread::sleep_for(12ms);
        check(::write(master,b.data()+8,8)==8,"complete fragment");
        auto split=channel->receive(20ms);check(bool(split),"receive fragmented packet");
        const auto completed=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        check(completed-split->received_ns>=8000000,"partial packet keeps earliest byte age");
        std::vector<std::uint8_t> burst;
        for(unsigned i=0;i<320;++i){auto bytes=packet(i);burst.insert(burst.end(),bytes.begin(),bytes.end());}
        check(::write(master,burst.data(),burst.size())==static_cast<ssize_t>(burst.size()),"send backlog");
        std::this_thread::sleep_for(40ms);
        unsigned count=0,last=0;bool untrusted=false;
        while(auto frame=channel->receive(0ms)){
            ++count;last=frame->data[0]+(frame->data[1]<<8);untrusted|=!frame->receive_timing_valid;
        }
        check(count<=transport::DEFAULT_CAN_CHANNEL_MAX_PENDING_FRAMES&&last==319,"bounded receiver drains to latest frame");
        check(untrusted,"burst and queue loss invalidate differentiation");
        bus->flush();check(!channel->receive(0ms),"flush receiver queue");bus->close();
        ::close(master);std::cout<<"transport timing contracts passed\n";return 0;
    }catch(const std::exception& e){::close(master);std::cerr<<e.what()<<'\n';return 1;}
}
