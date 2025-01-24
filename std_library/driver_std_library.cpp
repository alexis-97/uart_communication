#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <thread>


uint8_t buffer[256];

// Serial port configuration
void setup_serial(int& serial_fd, const std::string& port_name) {
	serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (serial_fd == -1) {
		std::cerr << "\033[31mError opening serial port.\033[0m" << std::endl;
		exit(1);
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));


	if (tcgetattr(serial_fd, &tty) != 0) {
		std::cerr << "\033[31mError getting port configuration\033[0m" << std::endl;
		exit(1);
	}

	// Speed configuration
	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	// Serial port configuration
	tty.c_cflag &= ~PARENB;  // no parity
	tty.c_cflag &= ~CSTOPB;  // 1 stop bit
	tty.c_cflag &= ~CSIZE;   // Size bits
	tty.c_cflag |= CS8;      // 8 bits per data
	tty.c_cflag &= ~CRTSCTS; // Disable hw flow control
	tty.c_cflag |= CREAD | CLOCAL; // Enable reading/wrinting

	tty.c_cc[VTIME] = 2; // Timeout of 0.2 seconds 
	tty.c_cc[VMIN] = 1;  // Minimum number of bytes to read

	// Apply configuration
	if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
		std::cerr << "\033[31mError during serial port configuration.\033[0m" << std::endl;
		exit(1);
	}
}

int receive_size_package(int serial_fd){
	read(serial_fd, buffer, 3);
	uint8_t acknowledge[3] = {0x7E, 0xFF, 0x7E};
	if(buffer[0] == 0x7E && buffer[2] == 0x7E){
		write(serial_fd, acknowledge, sizeof(acknowledge));
		return buffer[1];
	}else{
		std::cerr << "\033[31m[second thread] Invalid package: it does not start/end with 0x7E\033[0m" << std::endl;
		acknowledge[1] = 0x00;
		write(serial_fd, acknowledge, sizeof(acknowledge));
		return 0;
	}
	return 0;
}

void print_buffer(int start, int buffer_size){
	for (int i = start; i < buffer_size; i++) {
		std::cout << std::hex << +buffer[i] << " ";
	}
	std::cout << std::endl;
}


// Receive data from serial port
void receive_data(int serial_fd) {
	int buffer_size;
	uint8_t checksum;
	while (true) {
		// Read serial port
		
		buffer_size = receive_size_package(serial_fd);
		if(buffer_size != 0){
			int n = read(serial_fd, buffer, buffer_size);

			if (n > 0 && n == buffer_size) {
				
				if(buffer[0] == 0x7E && buffer[buffer_size-1] == 0x7E){
					// checksum verification
					checksum = buffer[1];
					for(int i = 2; i < buffer_size-2; i++)
						checksum ^= buffer[i];
					
					if (checksum == buffer[buffer_size-2]) {
						//Check the type of message
						if (buffer[2] == 1)
							std::cout << "[second thread] Acceleration (X,Y,Z): ";
	 					else if(buffer[2] == 2)
							std::cout << "[second thread] Angular Velocity (X,Y,Z): ";
						else if(buffer[2] == 3)
							std::cout << "[second thread] Other data: ";
						print_buffer(3, buffer_size-2);
						
					} else 
						std::cerr << "\033[31m[second thread] Error: Invalid checksum\033[0m" << std::endl;
						
				}else
					std::cerr << "\033[31m[second thread] Invalid Packet format (0x7E.)\033[0m" << std::endl;//Check format of the message
			}//Check the size of the message (8 or 20 bytes)
		}
		usleep(2000);
	}// while (true)
}

int main() {
	// Set up serial prot
	int serial_fd;
	setup_serial(serial_fd, "/dev/ttyV1"); 
    
	//Create thread for receiving data
	std::thread serial_port_thread(receive_data, serial_fd); //Execute thread
    
	while (true) {
		std::cout << "[main thread] Other operations..." << std::endl;
		sleep(1);
	}

	serial_port_thread.join();
	close(serial_fd);
    
	return 0;
}
