#include "msgbus.hpp"
#include <thread>
#include <chrono>
#include <iostream>

int main()
{
    msgbus::Topic<int> counter;
    std::thread publish_thd([&](){
        for (int i=0; true; i++) {
            std::cout << "publishing: " << i << std::endl;
            counter.publish(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });

    auto sub = msgbus::subscribe(counter);
    while (true) {
        if (sub.wait_for_update(.1)) {
            std::cout << "received: " << sub.get_value() << std::endl;
        } else {
            std::cout << "wait timed out" << std::endl;
        }
    }
    publish_thd.join();
}
