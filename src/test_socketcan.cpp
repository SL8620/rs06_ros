// test_socketcan.cpp
#include <rs06_ros/socketcan_driver.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>  // For signal handling

// Global flag for shutdown
std::atomic<bool> shutdown_flag(false);

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    shutdown_flag = true;
}

// Callback for received frames
void recvCallback(const struct can_frame& frame) {
    std::cout << "Received CAN frame: ID=0x" << std::hex << frame.can_id
              << ", DLC=" << std::dec << static_cast<int>(frame.can_dlc);
    std::cout << ", Data=[";
    for (int i = 0; i < frame.can_dlc; ++i) {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << (i < frame.can_dlc - 1 ? " " : "");
    }
    std::cout << "]" << std::endl;
}

int main() {
    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    // Initialize driver with interface "can0" (change if needed)
    SocketCanDriver driver("can1", recvCallback);

    // Start the driver
    if (!driver.start()) {
        std::cerr << "Failed to start SocketCanDriver." << std::endl;
        return 1;
    }

    std::cout << "SocketCanDriver started. Press Ctrl+C to stop." << std::endl;

    // Test sending frames at high rate
    for (int i = 0; i < 1000 && !shutdown_flag; ++i) {  // Send 1000 test frames
        struct can_frame frame;
        frame.can_id = 0x123 + (i % 10);  // Varying ID
        frame.can_dlc = 8;
        for (uint8_t j = 0; j < 8; ++j) {
            frame.data[j] = static_cast<uint8_t>(i + j);
        }

        if (!driver.send(frame)) {
            std::cerr << "Failed to send frame " << i << std::endl;
        }

        // Simulate high-rate: sleep 1ms between sends (adjust for real test)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Wait for shutdown or additional time to receive frames
    while (!shutdown_flag) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Stop the driver
    driver.stop();
    std::cout << "SocketCanDriver stopped." << std::endl;

    return 0;
}