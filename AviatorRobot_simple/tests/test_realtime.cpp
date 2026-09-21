#include "aviator/backend.hpp"
#include <mujoco/mujoco.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

int main() {
    try {
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const int key = mj_name2id(model.get(), mjOBJ_KEY, "aviator_home");
        if (key < 0 || !data) throw std::runtime_error("Missing home/data");
        mj_resetDataKeyframe(model.get(), data.get(), key);
        auto link = aviator::makeMuJoCoDirectDataLink(model.get(), data.get(),
                                                     AVIATOR_CONFIG_DIR "/aviator_control.urdf");
        bool passed = true;
        auto paced = [&](const char *label) {
            const auto start = std::chrono::steady_clock::now();
            const double sim_start = link->time();
            // Active controller consumption wakes the same CV as realtime pacing.
            while (link->time() - sim_start < 0.6) link->waitTick();
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            const double sim = link->time() - sim_start;
            std::cout << label << ": simulation=" << sim << "s wall=" << wall
                      << "s speed=" << sim / wall << "x\n";
            passed = passed && wall >= sim * 0.95;
        };
        paced("Realtime active controller");
        link->setRealTime(false);
        for (int i = 0; i < 20; ++i) link->waitTick();
        // Lockstep may finish its one outstanding step, then must stay paused.
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        const double paused = link->time();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (link->time() != paused) throw std::runtime_error("Lockstep advanced without consumption");
        link->setRealTime(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        if (link->time() <= paused) throw std::runtime_error("Realtime did not resume without a controller tick");
        paced("Realtime after mode switch");
        // Also check model timesteps other than 1 ms: one physical step needs its own wall duration.
        link.reset();
        model->opt.timestep = 0.002;
        mj_resetDataKeyframe(model.get(), data.get(), key);
        link = aviator::makeMuJoCoDirectDataLink(model.get(), data.get(),
                                                AVIATOR_CONFIG_DIR "/aviator_control.urdf");
        paced("Realtime with 2ms model timestep");
        if (!passed) throw std::runtime_error("Realtime simulation ran faster than wall time");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
