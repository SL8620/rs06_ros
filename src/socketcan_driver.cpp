// socketcan_driver.cpp
#include <rs06_ros/socketcan_driver.hpp>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

SocketCanDriver::SocketCanDriver(const std::string& interface_name, CanFrameCallback recv_callback)
	: interface_name_(interface_name), recv_callback_(recv_callback), running_(false) {}

SocketCanDriver::~SocketCanDriver() {
	stop();
	if (socket_fd_ != -1) {
		close(socket_fd_);
	}
}

bool SocketCanDriver::start() {
	if (running_) {
		return true;
	}

	socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (socket_fd_ < 0) {
		std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
		return false;
	}

	struct ifreq ifr;
	strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
		std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
		close(socket_fd_);
		socket_fd_ = -1;
		return false;
	}

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
		close(socket_fd_);
		socket_fd_ = -1;
		return false;
	}

	int flags = fcntl(socket_fd_, F_GETFL, 0);
	if (flags == -1 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
		std::cerr << "Error setting non-blocking: " << strerror(errno) << std::endl;
		close(socket_fd_);
		socket_fd_ = -1;
		return false;
	}

	running_ = true;
	recv_thread_ = std::thread(&SocketCanDriver::recvThread, this);
	send_thread_ = std::thread(&SocketCanDriver::sendThread, this);

	return true;
}

void SocketCanDriver::stop() {
	running_ = false;
	send_cv_.notify_all();
	if (recv_thread_.joinable()) {
		recv_thread_.join();
	}
	if (send_thread_.joinable()) {
		send_thread_.join();
	}
}

bool SocketCanDriver::send(const struct can_frame& frame) {
	std::unique_lock<std::mutex> lock(send_mutex_);
	if (!running_ || send_queue_.size() >= MAX_QUEUE_SIZE) {
		return false;
	}
	send_queue_.push(frame);
	send_cv_.notify_one();
	return true;
}

void SocketCanDriver::recvThread() {
	struct pollfd pfd;
	pfd.fd = socket_fd_;
	pfd.events = POLLIN;

	while (running_) {
		int ret = poll(&pfd, 1, 10);
		if (ret > 0 && (pfd.revents & POLLIN)) {
			struct can_frame frame;
			ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
			if (nbytes == sizeof(frame)) {
				if (recv_callback_) {
					recv_callback_(frame);
				}
			} else if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
				std::cerr << "Error reading frame: " << strerror(errno) << std::endl;
			}
		} else if (ret < 0) {
			std::cerr << "Poll error: " << strerror(errno) << std::endl;
		}
	}
}

void SocketCanDriver::sendThread() {
	while (running_) {
		std::unique_lock<std::mutex> lock(send_mutex_);
		send_cv_.wait(lock, [this] { return !send_queue_.empty() || !running_; });

		while (!send_queue_.empty() && running_) {
			struct can_frame frame = send_queue_.front();
			send_queue_.pop();
			lock.unlock();

			ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
			if (nbytes != sizeof(frame)) {
				std::cerr << "Error sending frame: " << strerror(errno) << std::endl;
			}

			lock.lock();
		}
	}
}
