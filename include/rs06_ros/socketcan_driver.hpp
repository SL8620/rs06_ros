// socketcan_driver.hpp
#ifndef RS06_ROS_SOCKETCAN_DRIVER_HPP
#define RS06_ROS_SOCKETCAN_DRIVER_HPP

#include <string>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// Callback type for received CAN frames
using CanFrameCallback = std::function<void(const struct can_frame&)>;

class SocketCanDriver {
public:
	SocketCanDriver(const std::string& interface_name, CanFrameCallback recv_callback);
	~SocketCanDriver();

	// 禁止拷贝和赋值，避免误用
	SocketCanDriver(const SocketCanDriver&) = delete;
	SocketCanDriver& operator=(const SocketCanDriver&) = delete;

	bool start();
	void stop();
	bool send(const struct can_frame& frame);

private:
	void recvThread();
	void sendThread();

	std::string interface_name_;
	CanFrameCallback recv_callback_;
	int socket_fd_ = -1;
	std::atomic<bool> running_;

	std::queue<struct can_frame> send_queue_;
	std::mutex send_mutex_;
	std::condition_variable send_cv_;
	static constexpr size_t MAX_QUEUE_SIZE = 10000;

	std::thread recv_thread_;
	std::thread send_thread_;
};

#endif  // RS06_ROS_SOCKETCAN_DRIVER_HPP
