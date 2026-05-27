#ifndef TEST_FSM_H
#define TEST_FSM_H
#include <unity.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>

#include <IMU/IMU.h>
#include <data.h>


#include <unity.h>

#include "mocks/mock_servo.h"
#include "mocks/mock_hal.h"

// lowkey bad practice but whatever
#define private public
#include  "tasks/canards_controller.h"
#undef private

using namespace task;

void test_safety_check_in_range_accel(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    imu_data imu_in_range{};
    imu_in_range.acceleration.x = 0.0f;

       
    TEST_ASSERT_FALSE(controller.safety_check(State::CALIBRATING, imu_in_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::READY, imu_in_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::POWERED, imu_in_range));
    TEST_ASSERT_TRUE(controller.safety_check(State::COASTING, imu_in_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::DROUGE, imu_in_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::MAIN, imu_in_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::LANDED, imu_in_range));
}
void test_safety_check_out_range_accel(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    imu_data imu_out_range{};
    imu_out_range.acceleration.x = 100.0f; // Exceeds threshold

    TEST_ASSERT_FALSE(controller.safety_check(State::CALIBRATING, imu_out_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::READY, imu_out_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::POWERED, imu_out_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::COASTING, imu_out_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::DROUGE, imu_out_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::MAIN, imu_out_range));
    TEST_ASSERT_FALSE(controller.safety_check(State::LANDED, imu_out_range));
}
void test_zero_angular_velocity(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.1f;
    controller.prev_roll_rate = 0.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = 0.0f;

    fake_tick = 1100;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, theta);
}

void test_constant_angular_velocity(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.0f;
    controller.prev_roll_rate = 1.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = 1.0f;

    fake_tick = 1100;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, theta);
}

void test_increasing_angular_velocity(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.0f;
    controller.prev_roll_rate = 1.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = 3.0f;

    fake_tick = 1200;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.349066f, theta);
}

void test_negative_angular_velocity(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.2f;
    controller.prev_roll_rate = -2.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = -2.0f;

    fake_tick = 1100;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, theta);
}

void test_positive_clamp(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.34f;
    controller.prev_roll_rate = 5.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = 5.0f;

    fake_tick = 1100;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.349066f, theta);
}

void test_negative_clamp(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = -0.34f;
    controller.prev_roll_rate = -5.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = -5.0f;

    fake_tick = 1100;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.349066f, theta);
}

void test_zero_delta_time(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.15f;
    controller.prev_roll_rate = 2.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = 5.0f;

    fake_tick = 1000;

    float theta = controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.15f, theta);
}

void test_prev_roll_rate_updated(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.0f;
    controller.prev_roll_rate = 1.0f;
    controller.last_time_ms = 1000;

    imu_data imu = {0};
    imu.gyro.x = 4.0f;

    fake_tick = 1100;

    controller.get_rocket_angle(imu);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        4.0f,
        controller.prev_roll_rate);
}

void test_run_controller_zero_moment_returns_neutral_servo(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    imu_data imu = {0};

    baro_data baro = {0};
    baro.pressure = 0.0f;
    baro.temperature = 300.0f;

    prediction_data pred = {0};
    pred.velocity = 0.0f;

    canards_raw result =
        controller.run_canards_controller(imu, baro, pred);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, result.kp);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, result.kd);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 90.0f, result.servo_angle);
}

void test_run_controller_positive_output(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.1f;
    controller.prev_roll_rate = 1.0f;
    controller.last_time_ms = 1000;

    fake_tick = 1100;

    imu_data imu = {0};
    imu.gyro.x = 1.0f;

    baro_data baro = {0};
    baro.pressure = 101325.0f;
    baro.temperature = 300.0f;

    prediction_data pred = {0};
    pred.velocity = 100.0f;

    canards_raw result =
        controller.run_canards_controller(imu, baro, pred);

    TEST_ASSERT_TRUE(result.kp > 0.0f);
    TEST_ASSERT_TRUE(result.kd > 0.0f);

    // Positive correction should move servo above 90
    TEST_ASSERT_TRUE(result.servo_angle > 90.0f);
}

// need to get correct values
void test_run_controller_upper_servo_clamp(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.34f;
    controller.prev_roll_rate = 20.0f;
    controller.last_time_ms = 1000;

    fake_tick = 1100;

    imu_data imu = {0};
    imu.gyro.x = 20.0f;

    baro_data baro = {0};
    baro.pressure = 101325.0f;
    baro.temperature = 300.0f;

    prediction_data pred = {0};
    pred.velocity = 300.0f;

    canards_raw result =
        controller.run_canards_controller(imu, baro, pred);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        98.0f,
        result.servo_angle);
}

// need to get correct values
void test_run_controller_lower_servo_clamp(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = -0.34f;
    controller.prev_roll_rate = -20.0f;
    controller.last_time_ms = 1000;

    fake_tick = 1100;

    imu_data imu = {0};
    imu.gyro.x = -20.0f;

    baro_data baro = {0};
    baro.pressure = 101325.0f;
    baro.temperature = 300.0f;

    prediction_data pred = {0};
    pred.velocity = 300.0f;

    canards_raw result =
        controller.run_canards_controller(imu, baro, pred);

    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        82.0f,
        result.servo_angle);
}

void test_run_controller_updates_rocket_angle(void)
{
    MockServo servo;
    Canards_Controller controller(servo, nullptr, nullptr);

    controller.rocket_angle = 0.0f;
    controller.prev_roll_rate = 1.0f;
    controller.last_time_ms = 1000;

    fake_tick = 1100;

    imu_data imu = {0};
    imu.gyro.x = 1.0f;

    baro_data baro = {0};
    baro.pressure = 101325.0f;
    baro.temperature = 300.0f;

    prediction_data pred = {0};
    pred.velocity = 100.0f;

    controller.run_canards_controller(imu, baro, pred);

    // rocket_angle should now contain integrated angle
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f,
        0.1f,
        controller.rocket_angle);
}

void setUp(void)
{
    fake_tick = 1000;
}

void tearDown(void)
{
    fake_tick = 0;
}


int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_safety_check_in_range_accel);
    RUN_TEST(test_safety_check_out_range_accel);
    RUN_TEST(test_zero_angular_velocity);
    RUN_TEST(test_constant_angular_velocity);
    RUN_TEST(test_increasing_angular_velocity);
    RUN_TEST(test_negative_angular_velocity);
    RUN_TEST(test_positive_clamp);
    RUN_TEST(test_negative_clamp);
    RUN_TEST(test_zero_delta_time);
    RUN_TEST(test_prev_roll_rate_updated);
    RUN_TEST(test_run_controller_zero_moment_returns_neutral_servo);
    RUN_TEST(test_run_controller_positive_output);
    //RUN_TEST(test_run_controller_upper_servo_clamp);
    //RUN_TEST(test_run_controller_lower_servo_clamp);
    RUN_TEST(test_run_controller_updates_rocket_angle);
    UNITY_END();
}

#endif // TEST_FSM_H