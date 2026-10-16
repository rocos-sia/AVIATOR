#include "rokae_session.hpp"
#include <rokae/robot.h>
#include <iostream>
#include <algorithm>
void check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
int main() {
    using aviator::calibration::RokaeSessionArm;
    try {
        {
            RokaeSessionArm arm("test");
            arm.connect(); arm.connect();
            check(fake_sdk::calls==std::vector<std::string>{"connect","toolset_load","rt","setLoad","idle"}, "load setup order/repeat wrong");
            check(fake_sdk::load==std::array<double,7>{.71,.02134,-.02368,.11188,0,0,0},"wrong payload");
            arm.startHand();
            arm.startDrag();
            check(arm.status().dragging && arm.status().power_state=="off","drag power wrong");
            arm.handTransfer({1,2,3},9);
            arm.stopDrag();
            check(arm.status().power_state=="on","hand power not restored");
            arm.startDrag();
            arm.disconnect();
            check(!arm.status().connected,"not disconnected");
        }
        const auto& calls=fake_sdk::calls;
        check(std::count(calls.begin(),calls.end(),"power_on")==2,"unexpected power-on during disconnect");
        check(calls[calls.size()-3]=="drag_off" && calls[calls.size()-2]=="power_off" && calls.back()=="disconnect","cleanup order wrong");
        fake_sdk::calls.clear(); fake_sdk::fail_load=true;
        {
            RokaeSessionArm arm("fail");
            bool failed=false;
            try {arm.connect();} catch(const std::exception&) {failed=true;}
            check(failed,"load failure ignored");
        }
        check(fake_sdk::calls==std::vector<std::string>{"connect","toolset_load","rt","setLoad","idle","disconnect"},"failed-load cleanup wrong");
        fake_sdk::calls.clear(); fake_sdk::fail_load=false; fake_sdk::fail_idle=true;
        {
            RokaeSessionArm arm("fail-idle");
            bool failed=false;
            try {arm.connect();} catch(const std::exception&) {failed=true;}
            check(failed,"Idle transition failure ignored");
            fake_sdk::fail_idle=false;
        }
        check(fake_sdk::calls==std::vector<std::string>{"connect","toolset_load","rt","setLoad","idle","idle","disconnect"},"mode cleanup not retried");
        std::cout<<"SDK payload, hand power and drag lifecycle passed\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
