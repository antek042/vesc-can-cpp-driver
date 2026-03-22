#include <stdint.h>
#include <cstring>
#include <optional>
#include <linux/can.h>

enum class VESC_ID : uint8_t {
	/*
	For example:
	VESC_RIGHT_1 = 0,
	VESC_RIGHT_2 = 1,
	VESC_RIGHT_3 = 2,
	VESC_LEFT_1 = 3,
	VESC_LEFT_2 = 4,
	VESC_LEFT_3 = 5
	*/
};

typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_STATUS = 9,
	CAN_PACKET_SET_CURRENT_REL = 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2 = 14,
	CAN_PACKET_STATUS_3 = 15,
	CAN_PACKET_STATUS_4 = 16,
	CAN_PACKET_STATUS_5 = 27,
	CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

/* Buffer append functions */

void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t *buffer, float number, float scale,
			   int32_t *index)
{
	buffer_append_int16(buffer, static_cast<int16_t>(number * scale),
			    index);
}

void buffer_append_float32(uint8_t *buffer, float number, float scale,
			   int32_t *index)
{
	buffer_append_int32(buffer, static_cast<int32_t>(number * scale),
			    index);
}

/* Buffer read functions */

int16_t buffer_read_int16(const uint8_t *buffer, int32_t index)
{
	return static_cast<int16_t>(((uint16_t)buffer[index + 1] << 8) | buffer[index]);
}

int32_t buffer_read_int32(const uint8_t *buffer, int32_t index)
{
	return static_cast<int32_t>(
		((uint32_t)buffer[index + 3] << 24) |
		((uint32_t)buffer[index + 2] << 16) |
		((uint32_t)buffer[index + 1] << 8) |
		(uint32_t)buffer[index]
	);
}

float buffer_read_float16(const uint8_t *buffer, int32_t index, float scale)
{
	return buffer_read_int16(buffer, index) / scale;
}

float buffer_read_float32(const uint8_t *buffer, int32_t index, float scale)
{
	return buffer_read_int32(buffer, index) / scale;
}

can_frame generate_frame(VESC_ID controller_id, CAN_PACKET_ID command,
			 uint8_t buffer[], int size)
{
	can_frame frame;
	frame.can_id = (static_cast<uint8_t>(controller_id) |
			(static_cast<uint32_t>(command) << 8)) |
		       CAN_EFF_FLAG;
	frame.can_dlc = size;
	memcpy(frame.data, buffer, size);
	return frame;
}

/* CAN frame creation functions */

can_frame comm_can_set_duty(VESC_ID controller_id, float duty)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, static_cast<int32_t>(duty * 100000.0),
			    &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_DUTY, buffer, 4);
}

can_frame comm_can_set_current(VESC_ID controller_id, float current)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, static_cast<int32_t>(current * 1000.0),
			    &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT, buffer, 4);
}

can_frame comm_can_set_current_off_delay(VESC_ID controller_id, float current,
					 float off_delay)
{
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_int32(buffer, static_cast<int32_t>(current * 1000.0),
			    &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT, buffer, 6);
}

can_frame comm_can_set_current_brake(VESC_ID controller_id, float current)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, static_cast<int32_t>(current * 1000.0),
			    &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT_BRAKE,
			      buffer, 4);
}

can_frame comm_can_set_rpm(VESC_ID controller_id, float rpm)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, static_cast<int32_t>(rpm), &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_RPM, buffer, 4);
}

can_frame comm_can_set_pos(VESC_ID controller_id, float pos)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, static_cast<int32_t>(pos * 1000000.0),
			    &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_POS, buffer, 4);
}

can_frame comm_can_set_current_rel(VESC_ID controller_id, float current_rel)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT_REL, buffer,
			      4);
}

can_frame comm_can_set_current_rel_off_delay(VESC_ID controller_id,
					     float current_rel, float off_delay)
{
	int32_t send_index = 0;
	uint8_t buffer[6];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	buffer_append_float16(buffer, off_delay, 1e3, &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT_REL, buffer,
			      6);
}

can_frame comm_can_set_current_brake_rel(VESC_ID controller_id,
					 float current_rel)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT_BRAKE_REL,
			      buffer, 4);
}

can_frame comm_can_set_handbrake(VESC_ID controller_id, float current)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current, 1e3, &send_index);
	return generate_frame(controller_id, CAN_PACKET_SET_CURRENT_HANDBRAKE,
			      buffer, 4);
}

can_frame comm_can_set_handbrake_rel(VESC_ID controller_id, float current_rel)
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_float32(buffer, current_rel, 1e5, &send_index);
	return generate_frame(controller_id,
			      CAN_PACKET_SET_CURRENT_HANDBRAKE_REL, buffer, 4);
}

struct status_message_1 {
	uint8_t vesc_id;
	float duty_cycle;
	float total_current;
	int32_t rpm;
};

std::optional<status_message_1> get_can_status_message_1(can_frame frame) {
	uint32_t can_id = frame.can_id & CAN_EFF_MASK;
	uint8_t packet_type = (can_id >> 8) & 0xFF;
	
	if (packet_type != CAN_PACKET_STATUS)
		return {};
	
	status_message_1 status_message;
	status_message.vesc_id = can_id & 0xFF;
	status_message.rpm = buffer_read_int32(frame.data, 0);
	status_message.total_current = buffer_read_float16(frame.data, 4, 10.0f);
	status_message.duty_cycle = buffer_read_float16(frame.data, 6, 1000.0f);
	
	return status_message;
}

struct status_message_2 {
	uint8_t vesc_id;
	float amp_hours;
	float amp_hours_charged;
};

std::optional<status_message_2> get_can_status_message_2(can_frame frame) {
	uint32_t can_id = frame.can_id & CAN_EFF_MASK;
	uint8_t packet_type = (can_id >> 8) & 0xFF;
	
	if (packet_type != CAN_PACKET_STATUS_2)
		return {};
	
	status_message_2 status_message;
	status_message.vesc_id = can_id & 0xFF;
	status_message.amp_hours = buffer_read_float32(frame.data, 0, 10000.0f);
	status_message.amp_hours_charged = buffer_read_float32(frame.data, 4, 10000.0f);
	
	return status_message;
}

struct status_message_3 {
	uint8_t vesc_id;
	float watt_hours_charged;
	float watt_hours;
};

std::optional<status_message_3> get_can_status_message_3(can_frame frame) {
	uint32_t can_id = frame.can_id & CAN_EFF_MASK;
	uint8_t packet_type = (can_id >> 8) & 0xFF;
	
	if (packet_type != CAN_PACKET_STATUS_3)
		return {};
	
	status_message_3 status_message;
	status_message.vesc_id = can_id & 0xFF;
	status_message.watt_hours = buffer_read_float32(frame.data, 0, 10000.0f);
	status_message.watt_hours_charged = buffer_read_float32(frame.data, 4, 10000.0f);
	
	return status_message;
}

struct status_message_4 {
	uint8_t vesc_id;
	float fet_temp;
	float motor_temp;
	float toal_current_in;
	float pid_pos;

};

std::optional<status_message_4> get_can_status_message_4(can_frame frame) {
	uint32_t can_id = frame.can_id & CAN_EFF_MASK;
	uint8_t packet_type = (can_id >> 8) & 0xFF;
	
	if (packet_type != CAN_PACKET_STATUS_4)
		return {};
	
	status_message_4 status_message;
	status_message.vesc_id = can_id & 0xFF;
	status_message.fet_temp = buffer_read_float16(frame.data, 0, 10.0f);
	status_message.motor_temp = buffer_read_float16(frame.data, 2, 10.0f);
	status_message.toal_current_in = buffer_read_float16(frame.data, 4, 10.0f);
	status_message.pid_pos = buffer_read_float16(frame.data, 6, 50.0f);
	
	return status_message;
}

struct status_message_5 {
	uint8_t vesc_id;
	int32_t tachometer_value;
	float input_voltage;
	int16_t reserved;
};

std::optional<status_message_5> get_can_status_message_5(can_frame frame) {
	uint32_t can_id = frame.can_id & CAN_EFF_MASK;
	uint8_t packet_type = (can_id >> 8) & 0xFF;
	
	if (packet_type != CAN_PACKET_STATUS_5)
		return {};
	
	status_message_5 status_message;
	status_message.vesc_id = can_id & 0xFF;
	status_message.tachometer_value = buffer_read_int32(frame.data, 0);
	status_message.input_voltage = buffer_read_float16(frame.data, 4, 10.0f);
	status_message.reserved = buffer_read_int16(frame.data, 6);
	
	return status_message;
}