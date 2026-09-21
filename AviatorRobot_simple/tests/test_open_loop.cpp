#include "../src/OpenLoopGrasp.hpp"
#include <iostream>
#include <stdexcept>
using namespace aviator;
void check(bool condition, const char *message) { if (!condition) throw std::runtime_error(message); }
int main() {
    try {
        OpenLoopGrasp g;
        double p[2]{}, r[2]{};
        g.update(0, 0, false, p, r, 0);
        g.command(GraspCommand::Lock);
        check(g.state().result == GraspResult::NotEnabled, "disabled lock accepted");
        g.update(1, 1, true, p, r, 0);
        g.command(GraspCommand::Lock);
        check(g.state().result == GraspResult::NotAligned, "stable interval bypassed");
        for (int n = 0; n < 1000; ++n) g.update(1, 1, true, p, r, 0);
        check(!g.state().ready, "poll count advanced stable time");
        g.update(1.11, 1.11, true, p, r, 0);
        g.command(GraspCommand::Lock);
        check(g.state().locked == 3 && g.state().open_loop, "software lock failed");
        g.reference(0.5, -0.1);
        g.update(1.12, 1.12, true, p, r, 0.1);
        check(g.state().angle == 0.5 && g.state().displacement == -0.1 && !g.state().fault,
              "command reference not preserved during motion");
        p[0] = 0.03;
        g.update(2, 2, true, p, r, 0);
        g.update(2.11, 2.11, true, p, r, 0);
        check(g.state().fault, "sustained TCP loss not detected");
        g.command(GraspCommand::ResetFault);
        check(g.state().result == GraspResult::Fault, "reset while locked accepted");
        g.command(GraspCommand::Unlock);
        g.command(GraspCommand::ResetFault);
        check(!g.state().locked && !g.state().fault, "unlock/reset failed");
        g.update(3, 2.8, true, p, r, 0);
        check(g.state().fault, "stale arm feedback not detected");
        std::cout << "Open-loop phase, references, timing and arm feedback faults passed\n";
    } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
