#include "linkerhand/linkerhand.hpp"

#include <chrono>
#include <iostream>
#include <optional>

using namespace std::chrono_literals;
using namespace linkerhand;
using namespace linkerhand::hand::l6;

static_assert(static_cast<int>(Finger::Thumb) == 0, "Thumb index");
static_assert(static_cast<int>(Finger::Pinky) == 4, "Pinky index");
static_assert(kFingerCount == 5, "5 fingers");

void test_types() {
    CanMessage msg{};
    msg.arbitration_id = 0x28;
    msg.dlc = 7;
    msg.data[0] = 0x01;

    AngleData ad{};
    ad.angles = {1, 2, 3, 4, 5, 6};
    ad.timestamp = 0.0;

    ForceSensorData fsd{};
    fsd.values.fill(0);
    fsd.timestamp = 0.0;

    AllFingersData afd{};
    afd[Finger::Thumb] = fsd;
    afd[Finger::Index] = fsd;
    const auto& t = afd[Finger::Thumb];
    (void)t;

    FingerArray<std::optional<ForceSensorData>> latest{};
    latest[0] = fsd;
    if (latest[0].has_value()) {
        (void)latest[0]->timestamp;
    }
}

void test_queue() {
    IterableQueue<int> q(10);
    q.put(1);
    q.put(2);

    bool ok = q.try_put(3);
    (void)ok;

    ok = q.force_put(4);
    (void)ok;

    auto val = q.try_get();
    if (val.has_value()) {
        (void)*val;
    }

    int v = q.get();
    (void)v;

    auto timed = q.get_for(100ms);
    if (timed.has_value()) {
        (void)*timed;
    }

    q.close();
}

void test_lifecycle() {
    Lifecycle lc("test");
    lc.ensure_open();
    auto count = lc.notification_count();
    lc.wait_for_notification(count, 10ms);
    lc.close();
}

int main() {
    test_types();
    test_queue();
    test_lifecycle();

    std::cout << "All compile-time and runtime type tests passed.\n";
    return 0;
}
