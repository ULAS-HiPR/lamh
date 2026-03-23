#include <IMU/IMU.h>
#include <data.h>

class MockIMU : public IMU {
public:
    std::vector<imu_data> samples;
    size_t idx = 0;

    bool init() override { return true; }  // <--- added
    bool update(imu_data* out) override {
        if (idx >= samples.size()) return false;
        *out = samples[idx++];
        return true;
    }
};
