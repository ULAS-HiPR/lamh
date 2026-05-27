#include <Servo/servo.h>
#include <data.h>

class MockServo : public Servo {
public:
    float last_angle = 0;

    bool init() override {return true;}
    bool set_position(int16_t position) override {
        last_angle = position;
    }
};