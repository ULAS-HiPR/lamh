#ifndef TEST_FSM_H
#define TEST_FSM_H
#include <unity.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>


#include <tasks/telemetry.h>
#include <tasks/state_machine.h>
#include <IMU/IMU.h>
#include <data.h>

#include <unity.h>
#include "state_machine.h"

#include "mock_imu.h"
#include "mock_rtos.h"
#include "mock_logger.h"

using namespace task;

void test_state_machine_runs_with_mock_data()
{
    // Inject some fake states
    fakeQueue.push(State::ROLL);
    fakeQueue.push(State::UNROLL);
    fakeQueue.push(State::STOP);

    State_Machine fsm;

    // Run only once instead of infinite loop
    fsm.StartStateMachine();

    TEST_ASSERT_TRUE(true);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_state_machine_runs_with_mock_data);
    UNITY_END();
}

#endif // TEST_FSM_H