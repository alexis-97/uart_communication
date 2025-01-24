#include <boost/asio.hpp>
#include <iostream>
#include <cstdlib>
#include <thread>
//#include <chrono>

void send_data(boost::asio::serial_port& serial) {
    uint8_t device_id = 0x01;
    uint8_t message_type = 0x01; // Type of message (1 = acceleration, 2 = angular velocity)
    uint8_t data[3] = {rand() % 100, rand() % 100, rand() % 100}; // 3 random values representing acceleration.
    
    message_type = 1+ (rand() % 2);
    
    // Calculate checksum
    uint8_t checksum = device_id ^ message_type ^ data[0] ^ data[1] ^ data[2];

    // CCreate packet
    uint8_t packet[] = {0x7E, device_id, message_type, data[0], data[1], data[2], checksum, 0x7E};

    // Send packet
    boost::asio::write(serial, boost::asio::buffer(packet, sizeof(packet)));

    
    if(message_type == 1)
    	std::cout << "Acceleration sent: ";
    else
    	std::cout << "Angular velocity sent: ";
    for (auto byte : packet) {
        std::cout << std::hex << +byte << " ";
    }
    std::cout << std::endl;
}

int main() {
    try {
        boost::asio::io_service io_service;
        boost::asio::serial_port serial(io_service, "/dev/ttyV0");
	
	//Configure serla port
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));


        //while (true) {
        int i = 0;
        while (i<10) {
            send_data(serial);
            sleep(3);
            i++;
        }

    } catch (const boost::system::system_error& e) {
        std::cerr << "\033[31mError: " << e.what() << std::endl;
    }

    return 0;
}
