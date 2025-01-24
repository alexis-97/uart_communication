#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <random>
#include <thread>


// Configuración del puerto serial
void setup_serial(int& serial_fd, const std::string& port_name) {
	serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (serial_fd == -1) {
		std::cerr << "\033[31mError al abrir el puerto serial.\033[0m" << std::endl;
		exit(1);
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(serial_fd, &tty) != 0) {
		std::cerr << "\033[31mError getting port configuration\033[0m" << std::endl;
		exit(1);
	}

	// Speed  configuration
	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	// Serial port configuration
	tty.c_cflag &= ~PARENB;  // no parity
	tty.c_cflag &= ~CSTOPB;  // 1 stop bit
	tty.c_cflag &= ~CSIZE;   // Size bits
	tty.c_cflag |= CS8;      // 8 bits per data
	tty.c_cflag &= ~CRTSCTS; // Disable hw flow control 
	tty.c_cflag |= CREAD | CLOCAL; // Enable reading/wrinting 

	tty.c_cc[VTIME] = 1; // Timeout of 0.1 seconds
	tty.c_cc[VMIN] = 1;  // Minimum number of bytes to read

	// Aplicar configuraciones
	if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
		std::cerr << "\033[31mError al configurar el puerto serial.\033[0m" << std::endl;
		exit(1);
	}
}


int receive_acknowledge(int serial_fd) {
	uint8_t buffer[3];
	int n = read(serial_fd, buffer, sizeof(buffer));
	
	if(buffer[0] == 0x7E && buffer[2] == 0x7E){//Check valid format of the acknowledge message
		if(buffer[1] == 0xFF) // Receiver is prepare to receive the data
			return 1;
		else //Error, receiver is not ready for receiving data
			return 0;
	}
}


// Send data through serial port
void send_data(int serial_fd) {
	// Simulate the generation of the data that is going to be sent
	uint8_t device_id = 0x01;
	uint8_t message_type = 1+rand() % 3;  // Kind of message (1 = acceleration, 2 = angular velocity, 3 = other data)
	uint8_t size_of_packet;
	
	//Size of the message based on the type of message
	if(message_type == 1 || message_type == 2)
		size_of_packet= 8;
	else
		size_of_packet= 20;
				
	//Send the size of the message
	uint8_t packet[] = {0x7E, size_of_packet, 0x7E};
	write(serial_fd, packet, sizeof(packet));
	
	//Wait for acknowledge
	int ack = receive_acknowledge(serial_fd); 
	if(ack == 0){
		std::cerr << "\033[31mAcknowledge = 0. Restarting communication\033[0m" << std::endl;
		return;
	}
    	
	// Create the packet with the data
	//Create the packet and calculate the checksum
	uint8_t packet_2[size_of_packet];// = {0x7E, device_id, message_type};;
	uint8_t checksum = device_id^message_type;
	uint8_t data;
	int i;
	packet_2[0] = 0x7E;
	packet_2[1] = device_id;
	packet_2[2] = message_type;
	for(i = 0; i < size_of_packet-5; i++){
		data = rand() % 100;
		checksum ^= data;
		packet_2[i+3] = data;
	}
	packet_2[i+3] = checksum;
	packet_2[i+4] = 0x7E;
    
	//Debugging
	std::cout << "Sent package (" << sizeof(packet_2) << " bytes): " << std::endl;
	for(auto byte : packet_2)
		std::cout << std::hex << +byte << " ";
	std::cout << std::endl;	
	
	// Send message
	write(serial_fd, packet_2, sizeof(packet_2));

}


int main() {
	//Open serial port
	int serial_fd;
	setup_serial(serial_fd, "/dev/ttyV0"); 
    
	// Send data every 3 seconds
	while (true) {
		send_data(serial_fd);
		sleep(3);//seconds
	}

	close(serial_fd);
	return 0;
}
