// RS06 电机 CAN 协议实现
#include <rs06_ros/rs06.hpp>

#include <cstring>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {
}

RS06::RS06(SocketCanDriver& can_driver, uint8_t id,
		 float p_safe_min, float p_safe_max,
		 float v_safe_min, float v_safe_max)
	: can_driver_(can_driver), id_(id),
	  p_safe_min_(p_safe_min), p_safe_max_(p_safe_max),
	  v_safe_min_(v_safe_min), v_safe_max_(v_safe_max)
{
	if (p_safe_min_ > p_safe_max_) std::swap(p_safe_min_, p_safe_max_);
	if (v_safe_min_ > v_safe_max_) std::swap(v_safe_min_, v_safe_max_);
}

RS06::~RS06() {
	(void)disable();
}

bool RS06::enable()
{
	struct can_frame frame{};
	packEnable(frame);
	bool ok = can_driver_.send(frame);
	if (ok)
	{
		enabled_.store(true);
	}
	return ok;
}

bool RS06::disable()
{
	struct can_frame frame{};
	packDisable(frame);
	bool ok = can_driver_.send(frame);
	if (ok)
	{
		enabled_.store(false);
	}
	return ok;
}

bool RS06::setZero()
{
	if (enabled_.load())
	{
		std::cerr << "RS06 motor (id=" << static_cast<int>(id_)
			  << ") is enabled, disable before setting zero point." << std::endl;
		return false;
	}

	struct can_frame frame{};
	packSetZero(frame);
	return can_driver_.send(frame);
}

bool RS06::setPositionMode()
{
	struct can_frame frame{};
	packSetPositionMode(frame);
	bool ok = can_driver_.send(frame);
	if (ok)
	{
		position_mode_enabled_.store(true);
	}
	return ok;
}

bool RS06::setPosition(double position_rad, double velocity_rad_s) {
	if (!position_mode_enabled_.load())
	{
		std::cerr << "RS06 motor (id=" << static_cast<int>(id_)
			  << ") position mode not enabled, call setPositionMode() first." << std::endl;
		return false;
	}

	double pos = position_rad;
	double vel = velocity_rad_s;

	if (vel <= 0.0)  vel = 0.0;
	if (vel >= 50.0) vel = 50.0;

	if (pos < static_cast<double>(p_safe_min_)) pos = static_cast<double>(p_safe_min_);
	if (pos > static_cast<double>(p_safe_max_)) pos = static_cast<double>(p_safe_max_);
	if (vel < static_cast<double>(v_safe_min_)) vel = static_cast<double>(v_safe_min_);
	if (vel > static_cast<double>(v_safe_max_)) vel = static_cast<double>(v_safe_max_);

	struct can_frame frame{};
	float vel_f = static_cast<float>(vel);
	uint32_t vel_bits = 0;
	std::memcpy(&vel_bits, &vel_f, sizeof(float));
	packIndexedCommand(frame, 0x7017, vel_bits, 4);
	if (!can_driver_.send(frame)) {
		return false;
	}

	float pos_f = static_cast<float>(pos);
	uint32_t pos_bits = 0;
	std::memcpy(&pos_bits, &pos_f, sizeof(float));
	packIndexedCommand(frame, 0x7016, pos_bits, 4);
	return can_driver_.send(frame);
}

bool RS06::requestPosition() {
	struct can_frame frame{};
	packRequestPosition(frame);
	return can_driver_.send(frame);
}

void RS06::handleCanFrame(const struct can_frame& frame) {
	if (!(frame.can_id & CAN_EFF_FLAG)) {
		return;
	}

	uint32_t raw_id = frame.can_id & CAN_EFF_MASK;
	uint8_t motor_id  = static_cast<uint8_t>((raw_id >> 8) & 0xFF);
	if (motor_id != id_) {
		return;
	}

	unpackFeedback(frame);
}

void RS06::packEnable(struct can_frame& frame) const
{
	std::memset(&frame, 0, sizeof(frame));
	uint32_t can_id = 0;
	can_id |= (0x3u << 24);
	can_id |= (static_cast<uint32_t>(CAN_MASTER_ID) & 0xFFFFu) << 8;
	can_id |= static_cast<uint32_t>(id_);
	frame.can_id  = can_id | CAN_EFF_FLAG;
	frame.can_dlc = 8;
}

void RS06::packDisable(struct can_frame& frame) const
{
	std::memset(&frame, 0, sizeof(frame));
	uint32_t can_id = 0;
	can_id |= (0x4u << 24);
	can_id |= (static_cast<uint32_t>(CAN_MASTER_ID) & 0xFFFFu) << 8;
	can_id |= static_cast<uint32_t>(id_);
	frame.can_id  = can_id | CAN_EFF_FLAG;
	frame.can_dlc = 8;
}

void RS06::packSetZero(struct can_frame& frame) const
{
	std::memset(&frame, 0, sizeof(frame));
	uint32_t can_id = 0;
	can_id |= (0x6u << 24);
	can_id |= (static_cast<uint32_t>(CAN_MASTER_ID) & 0xFFFFu) << 8;
	can_id |= static_cast<uint32_t>(id_);
	frame.can_id  = can_id | CAN_EFF_FLAG;
	frame.can_dlc = 8;
	frame.data[0] = 0x01;
}

void RS06::packIndexedCommand(struct can_frame& frame,
							 uint16_t index,
							 uint32_t data,
							 uint8_t data_len) const
{
	std::memset(&frame, 0, sizeof(frame));

	if (data_len > 4) {
		data_len = 4;
	}

	uint32_t can_id = 0;
	can_id |= (0x12u << 24);
	can_id |= (static_cast<uint32_t>(CAN_MASTER_ID) & 0xFFFFu) << 8;
	can_id |= static_cast<uint32_t>(id_);
	frame.can_id  = can_id | CAN_EFF_FLAG;
	frame.can_dlc = 8;

	frame.data[0] = static_cast<uint8_t>(index & 0xFF);
	frame.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;

	for (uint8_t i = 0; i < data_len; ++i) {
		frame.data[4 + i] = static_cast<uint8_t>((data >> (8 * i)) & 0xFF);
	}
}

void RS06::packSetPositionMode(struct can_frame& frame) const
{
	packIndexedCommand(frame, 0x7005, 0x05u, 1);
}

void RS06::packRequestPosition(struct can_frame& frame) const {
	std::memset(&frame, 0, sizeof(frame));

	uint32_t can_id = 0;
	can_id |= (0x11u << 24);
	can_id |= (static_cast<uint32_t>(CAN_MASTER_ID) & 0xFFFFu) << 8;
	can_id |= static_cast<uint32_t>(id_);
	frame.can_id  = can_id | CAN_EFF_FLAG;
	frame.can_dlc = 8;

	frame.data[0] = 0x19;
	frame.data[1] = 0x70;
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
}

void RS06::unpackFeedback(const struct can_frame& frame) {
	if (frame.can_dlc < 8) {
		return;
	}

	uint16_t index = static_cast<uint16_t>(frame.data[0]) |
			 (static_cast<uint16_t>(frame.data[1]) << 8);
	if (index != 0x7019) {
		return;
	}

	uint32_t bits = 0;
	bits |= static_cast<uint32_t>(frame.data[4]);
	bits |= static_cast<uint32_t>(frame.data[5]) << 8;
	bits |= static_cast<uint32_t>(frame.data[6]) << 16;
	bits |= static_cast<uint32_t>(frame.data[7]) << 24;
	float pos_f = 0.0f;
	std::memcpy(&pos_f, &bits, sizeof(float));

	last_position_rad_.store(static_cast<double>(pos_f));
	has_position_.store(true);
}
