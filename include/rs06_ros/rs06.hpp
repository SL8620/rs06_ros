// RS06 电机 CAN 协议封装
#ifndef RS06_ROS_RS06_HPP
#define RS06_ROS_RS06_HPP

#include <cstdint>
#include <atomic>
#include <linux/can.h>

#include <rs06_ros/socketcan_driver.hpp>

// 位置 / 速度默认安全限制（单位：rad, rad/s）
// 可通过构造函数自定义
// 位置：默认 -3.14 ~ 3.14 rad
// 速度：物理允许范围 (0, 50) rad/s，安全范围默认 0 ~ 5 rad/s
#define P_SAFE_MIN  (-3.14f)
#define P_SAFE_MAX  ( 3.14f)
#define V_SAFE_MIN  ( 0.0f)
#define V_SAFE_MAX  ( 5.0f)

// 主机 ID 定义
#define CAN_MASTER_ID 0x02

class RS06 {
public:
	RS06(SocketCanDriver& can_driver, uint8_t id,
		 float p_safe_min = P_SAFE_MIN,
		 float p_safe_max = P_SAFE_MAX,
		 float v_safe_min = V_SAFE_MIN,
		 float v_safe_max = V_SAFE_MAX);
	~RS06();

	bool enable();
	bool disable();
	bool setZero();
	bool setPositionMode();
	bool setPosition(double position_rad, double velocity_rad_s);
	bool requestPosition();
	void handleCanFrame(const struct can_frame& frame);
	bool hasPosition() const { return has_position_.load(); }
	double position() const { return last_position_rad_.load(); }
	uint8_t id() const { return id_; }

private:
	void packEnable(struct can_frame& frame) const;
	void packDisable(struct can_frame& frame) const;
	void packSetZero(struct can_frame& frame) const;
	void packIndexedCommand(struct can_frame& frame,
							 uint16_t index,
							 uint32_t data,
							 uint8_t data_len = 1) const;
	void packSetPositionMode(struct can_frame& frame) const;
	void packRequestPosition(struct can_frame& frame) const;
	void unpackFeedback(const struct can_frame& frame);

	SocketCanDriver& can_driver_;
	uint8_t id_;
	float p_safe_min_;
	float p_safe_max_;
	float v_safe_min_;
	float v_safe_max_;
	std::atomic<bool> position_mode_enabled_{false};
	std::atomic<bool> enabled_{false};
	std::atomic<bool> has_position_{false};
	std::atomic<double> last_position_rad_{0.0};
};

#endif  // RS06_ROS_RS06_HPP
