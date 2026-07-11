#include <unity.h>
#include <CAN/CAN_Frames.h>

void test_canonical_critical_can_ids() {
    TEST_ASSERT_EQUAL_HEX16(0x040U, CAN_ID_KALMANN);
    TEST_ASSERT_EQUAL_HEX16(0x050U, CAN_ID_PYRO_ARM);
    TEST_ASSERT_EQUAL_HEX16(0x060U, CAN_ID_PYRO_FIRE);
}

void test_heartbeat_node_id_and_payload_round_trip() {
    HEARTBEAT_Payload sent{NODE_LAMH, 1U, 0U, 42U};
    CAN_Frame frame = pack_frame(CAN_ID_HEARTBEAT_NODE(NODE_LAMH), sent);
    HEARTBEAT_Payload received{};

    TEST_ASSERT_TRUE(try_unpack_frame(frame, received));
    TEST_ASSERT_EQUAL_UINT8(NODE_LAMH, received.node_id);
    TEST_ASSERT_EQUAL_UINT8(42U, received.uptime_s);
}

void test_actuator_command_addresses_one_output() {
    ActuatorCommandPayload sent{2U, ACTUATOR_COMMAND_FLAG_ACTIVE, 3750, 14U, 0U};
    CAN_Frame frame = pack_frame(CAN_ID_ACTUATOR_COMMAND, sent);
    ActuatorCommandPayload received{};

    TEST_ASSERT_TRUE(try_unpack_frame(frame, received));
    TEST_ASSERT_EQUAL_UINT8(2U, received.output_index);
    TEST_ASSERT_EQUAL_INT16(3750, received.angle_cdeg);
    TEST_ASSERT_EQUAL_UINT16(14U, received.sequence);
}

void test_gps_24bit_codec_covers_global_coordinates() {
    const double latitude = 53.3498;
    const double longitude = -6.2603;
    CAN_Frame frame = pack_gps(
        CAN_ID_GPS,
        gps_encode(latitude),
        gps_encode(longitude),
        8U,
        1U
    );
    GPS_Payload payload{};

    TEST_ASSERT_TRUE(unpack_gps(frame, payload));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, static_cast<float>(latitude), static_cast<float>(gps_decode(payload.latitude)));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, static_cast<float>(longitude), static_cast<float>(gps_decode(payload.longitude)));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_canonical_critical_can_ids);
    RUN_TEST(test_heartbeat_node_id_and_payload_round_trip);
    RUN_TEST(test_actuator_command_addresses_one_output);
    RUN_TEST(test_gps_24bit_codec_covers_global_coordinates);
    return UNITY_END();
}
