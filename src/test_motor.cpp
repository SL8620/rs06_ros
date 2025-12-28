#include <rs06_ros/socketcan_driver.hpp>
#include <rs06_ros/rs06.hpp>

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// 全局退出标志
static std::atomic<bool> g_shutdown{false};

void signalHandler(int signum)
{
	std::cout << "\nInterrupt signal (" << signum << ") received. Shutting down..." << std::endl;
	g_shutdown.store(true);
}

int main()
{
	// 注册 Ctrl+C 信号处理
	signal(SIGINT, signalHandler);

	// 创建电机对象，ID = 101，使用默认安全限位
	RS06* motor_ptr = nullptr; // 先声明指针，供回调捕获

	// CAN 接收回调：把所有帧转给 RS06 对象处理
	CanFrameCallback cb = [&motor_ptr](const struct can_frame& frame) {
		RS06* m = motor_ptr;
		if (m)
		{
			m->handleCanFrame(frame);
		}
	};

	// 创建并启动 SocketCanDriver（根据实际接口修改，如 can0/can1）
	SocketCanDriver driver("can1", cb);
	if (!driver.start())
	{
		std::cerr << "Failed to start SocketCanDriver on can1" << std::endl;
		return 1;
	}

	RS06 motor(driver, 101);
	motor_ptr = &motor;

	// 位置刷新线程：周期性请求位置（不直接打印，避免和交互输出打架）
	std::thread pos_thread([&motor]() {
		using namespace std::chrono_literals;
		while (!g_shutdown.load())
		{
			(void)motor.requestPosition();
			std::this_thread::sleep_for(200ms);
		}
	});

	std::cout << "RS06 motor test started (ID=101)." << std::endl;
	std::cout << "Commands:" << std::endl;
	std::cout << "  e - enable" << std::endl;
	std::cout << "  d - disable" << std::endl;
	std::cout << "  s - position mode + set target" << std::endl;
	std::cout << "  q - quit" << std::endl;

	while (!g_shutdown.load())
	{
		// 每次等待输入前打印一次当前最新位置，保持界面整洁
		if (motor.hasPosition())
		{
			double pos = motor.position();
			std::cout << "\nCurrent position: " << pos << " rad" << std::endl;
		}

		std::cout << "\n\nEnter command (e/d/s/q): " << std::flush;
		char cmd = 0;
		if (!(std::cin >> cmd))
		{
			break;
		}

		if (cmd == 'q')
		{
			g_shutdown.store(true);
			break;
		}
		else if (cmd == 'e')
		{
			if (!motor.enable())
			{
				std::cerr << "Enable command failed" << std::endl;
			}
			else
			{
				std::cout << "Enabled motor 101" << std::endl;
			}
		}
		else if (cmd == 'd')
		{
			if (!motor.disable())
			{
				std::cerr << "Disable command failed" << std::endl;
			}
			else
			{
				std::cout << "Disabled motor 101" << std::endl;
			}
		}
		else if (cmd == 's')
		{
			// 先进入位置模式
			if (!motor.setPositionMode())
			{
				std::cerr << "Set position mode failed" << std::endl;
				continue;
			}
			std::cout << "Position mode enabled." << std::endl;

			// 获取目标速度和位置
			double vel = 0.0;
			double pos = 0.0;
			std::cout << "Input target velocity (rad/s, 0~50): " << std::flush;
			if (!(std::cin >> vel))
			{
				std::cin.clear();
				std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				std::cerr << "Invalid velocity input" << std::endl;
				continue;
			}
			std::cout << "Input target position (rad): " << std::flush;
			if (!(std::cin >> pos))
			{
				std::cin.clear();
				std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
				std::cerr << "Invalid position input" << std::endl;
				continue;
			}

			if (!motor.setPosition(pos, vel))
			{
				std::cerr << "Set position failed (check mode/limits)" << std::endl;
			}
			else
			{
				std::cout << "Commanded position=" << pos << " rad, velocity=" << vel << " rad/s" << std::endl;
			}
		}
		else
		{
			std::cout << "Unknown command: " << cmd << std::endl;
		}
	}

	g_shutdown.store(true);
	if (pos_thread.joinable())
	{
		pos_thread.join();
	}

	driver.stop();
	std::cout << "RS06 motor test stopped." << std::endl;
	return 0;
}

