#include "msgbus.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>
#include <array>

int main()
{
    msgbus::Topic<int> A;
    msgbus::Topic<float> B;
    msgbus::Topic<std::string> C;
    std::thread publish_A_thd([&](){
        for (int i=0; true; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "publishing on A: " << i << std::endl;
            A.publish(i);
        }
    });

    std::thread publish_B_thd([&](){
        for (int i=0; true; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            float f = i*1.1;
            std::cout << "publishing on B: " << f << std::endl;
            B.publish(f);
        }
    });

    std::thread publish_C_thd([&](){
        for (int i=0; true; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(6000));
            auto str = "hello world " + std::to_string(i);
            std::cout << "publishing on C: " << str << std::endl;
            C.publish(str);
        }
    });

    auto subA = msgbus::subscribe(A);
    auto subB = msgbus::subscribe(B);
    auto subC = msgbus::subscribe(C);
    std::array<msgbus::SubscriberBase*, 3> subs = {{&subA, &subB, &subC}};
    while (msgbus::wait_for_update_on_any(subs.begin(), subs.end())) {
        std::cout << "wait for update returned" << std::endl;
        if (subA.has_update()) {
            std::cout << "received on A: " << subA.get_value() << std::endl;
        }
        if (subB.has_update()) {
            std::cout << "received on B: " << subB.get_value() << std::endl;
        }
        if (subC.has_update()) {
            std::cout << "received on C: " << subC.get_value() << std::endl;
        }
    }
}
