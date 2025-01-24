#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <thread>

boost::asio::io_service io_service;  // Create I/O service
boost::asio::serial_port serial(io_service, "/dev/ttyV1");
std::vector<uint8_t> buffer(8);


void receive_data(const boost::system::error_code& error, size_t bytes_transferred) {
    size_t bytes_read = bytes_transferred;
    if (!error){
        //std::cout << "--------------------------------------------" << std::endl;
        std::cout << "[second thread] Package received: ";
        for (size_t i = 0; i < bytes_read; ++i) {
            std::cout << std::hex << +(buffer[i]) << " "; //Use "+" for the char->int conversion
        }
        std::cout << std::endl;

        // Check the first and last bytes
        if (buffer[0] == 0x7E && buffer[bytes_read - 1] == 0x7E) {
            uint8_t checksum = buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
            if (checksum == buffer[6]) {
                	if (buffer[2] == 1)
                		std::cout << "[second thread] Acceleration X: " << (int)buffer[3] << ", Y: " << (int)buffer[4] << ", Z: " << (int)buffer[5] << std::endl;
                	else 
                		std::cout << "[second thread] Angular Velocity X: " << (int)buffer[3] << ", Y: " << (int)buffer[4] << ", Z: " << (int)buffer[5] << std::endl;
            } else {
                std::cerr << "\033[31mError: invalid checksum. Expected checksum = " << std::hex << +checksum << "\033[0m" << std::endl;
            }
        } else {
            std::cerr << "\033[31mInvalid package: it does not start/end with 0x7E.\033[0m" << std::endl;
        }
    } else {
        std::cerr << "\033[31mError in reading data.\033[0m" << std::endl;
    }
    
            
    //set the asynchronous read again
    boost::asio::async_read(serial, boost::asio::buffer(buffer), receive_data);
}

void io_service_run(boost::asio::io_service& io_service) {
    // Execute I/O service in a different thread
    io_service.run();
}

int main() {
    //try {
	//Setting the serial communication configuration
        serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        // Start asynchronous read
        boost::asio::async_read(serial, boost::asio::buffer(buffer), receive_data);

        // Parallel thread waiting for interruption to execute the callback function.
        std::thread io_thread(io_service_run, std::ref(io_service));

        // Other processes while we wait for the interruption of the callback function
        while(true){
        	std::cout << "[main thread] Processing other algorithms on the main thread..." << std::endl;
        	usleep(1000000);
	}
        
        
        io_thread.join();
    /*} catch (const boost::system::system_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }*/

    return 0;
}
