#include <ros/ros.h>

#include "serial_connection.h"
#include "crc_calc.h"
#include "util.h"

//#include <stropts.h>
#include <termios.h>
//#include <asm/termios.h>

#define termios asmtermios
#define winsize asmwinsize
#define termio asmtermio

#include <asm/termios.h>
#undef  termios
#undef  winsize
#undef  termio
#include <termios.h>

typedef boost::asio::serial_port_base asio_serial;
using namespace std;

namespace ComLib
{

SerialConnection::SerialConnection()
{  
  expectedSize = 0;
  fileDescription = 0;  
  //serialPort = new boost::asio::serial_port (io_service);
  //serialPort->async_read_some
}

SerialConnection::~SerialConnection()
{
  deviceListString.clear();  
  rxArray.clear();
}

void SerialConnection::addGeneralAnswerType(const uint8_t type)
{
  generalAnswerTypes.push_back(type);
}

/**
 * @brief List the available devices
 *
 * This function is used to list the available devices. The index of a device in the list can later be
 * used to select the port to open. The strings can for example be directly put into a comboBox in the GUI and
 * the index is directly given by the comboBox.
 *
 * @return List of strings containing the names of the available devices
 */
vector<string> SerialConnection::availableDevices()
{
  /*const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
      deviceListString.append(info.portName());*/

  deviceListString.clear();
  string str = "/dev/ttyACM0";
  deviceListString.push_back(str);
  ROS_INFO_STREAM("SerialConnection::availableDevices: "<< str);
  return deviceListString;
}

/**
 * @brief Open the serial port
 *
 * @param id Id of the serial port = index of the list when the function "availableDevices" is called
 * @retval true Port is open
 * @retval false Port is not open
 * @return Port open or not
 */
bool SerialConnection::openPort()
{  
    if(fileDescription > 0) closePort();
	bool bFind = false;
	char path[100];
	for(int i = 0;i<10;i++){
		sprintf(path,"/dev/ttyACM%d", i);
		fileDescription = open(path, O_RDWR | O_NOCTTY | O_SYNC); //ttyUSB0 -old
		
		if(fileDescription > 0)
		{
			bFind = true;
			break;
		}
	}
	if( bFind == false ){
		printf("tty error ----------------------------------\n");
		return false;
	}

    set_interface_attribs(B4000000, 0);  // set speed to 10000000 bps, 8n1 (no parity)
    //set_blocking (1);                    // set no blocking
    rxArray.clear();
    return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closePort()
{      
    fileDescription = close(fileDescription);
    ROS_INFO("SerialConnection::closePort: %d\n", fileDescription);
}

/**
 * @brief Check, if the checksum is correct
 *
 * This function checks the CRC. It extracts the CRC from the received data and calculates the CRC of the received
 * data and then compares them.
 *
 * @param array Pointer to the received data
 * @param expectedSize Expected size of the received data
 * @return Checksum correct or not
 */
bool SerialConnection::checksumIsCorrect(const vector<uint8_t> &array, const unsigned int expectedSize)
{
  //The received CRC is the one in the data
  uint32_t receivedCrc = Util::getUint32LittleEndian(array, (CommunicationConstants::Data::SIZE_HEADER + expectedSize));

  //The wanted CRC is the one calculated out of the payload    
  uint32_t wantedCrc = calculateChecksum((uint8_t *)array.data(), (CommunicationConstants::Data::SIZE_HEADER + expectedSize));

  if (receivedCrc == wantedCrc){
    return true;
  }

  printf("SerialConnection::checksumIsCorrect ERROR!!!\n");
  return false;
}

/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
ssize_t SerialConnection::sendData(uint8_t *data)
{
    if(fileDescription <=0 ){
        ROS_ERROR("Error SerialConnection::sendData fileDescription =0 \n");
        return 0;
    }

    //Add the start buffer at the beginning
    data[0] = CommunicationConstants::Command::START_MARK;

    //Calculate the CRC    
     uint32_t crc = calculateChecksum(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)));

    //Add it to the buffer123
    Util::setUint32LittleEndian(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)), crc);

    //This is just to print out the data
	/*
    std::string str= "SEND DATA: ";
    char buf[4];
    for(int i=0; i<14; i++){
        sprintf(buf, "%x ", data[i]);
        str.append(buf);
    }
	*/
    //ROS_DEBUG_STREAM(str);
	


    return write(fileDescription, (uint8_t *)(data), CommunicationConstants::Command::SIZE_TOTAL);
}


/**
 * @brief Extract the expected size from the received data
 *
 * @param array Pointer to the received data
 * @return Expected size
 */
int SerialConnection::getExpextedSize(const std::vector<uint8_t> &array)
{
  int expectedSize = Util::getUint16LittleEndian(array, CommunicationConstants::Data::INDEX_LENGTH);
  return expectedSize;
}

/**
 * @brief Extract the type from the received data
 *
 * @param array Pointer to the received data
 * @return Received type
 */
uint8_t SerialConnection::getType(const vector<uint8_t> &array)
{
    return array.at(CommunicationConstants::Data::INDEX_TYPE);
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int SerialConnection::readRxData(int size, bool reMain)
{  
  std::ignore = reMain;
  static uint8_t buf[50000] = {0};
  int n = read(fileDescription, buf, size);

  if(n == -1 ){
    printf("Error on  SerialConnection::readRxData\n");
    return -1;
  }

  
  if( n > 0 ) rxArray.insert(std::end(rxArray), buf, buf + n); //Append the new data to the rxArray buffer
  //processData(rxArray, size);
  return n;
}

bool SerialConnection::processRemainData(int rxSize)
{
    static uint8_t buf[4096];
	
    int n = 0;

    for(int i=0; i< rxSize; i+=n)
    {
        memset(buf, 0, sizeof(buf)); //clear buffer

        int buf_size = rxSize;
        if(buf_size > sizeof(buf))
            buf_size = sizeof(buf);

        n = read(fileDescription, buf, buf_size);

        if(n > 0){                        
         rxArray.insert(std::end(rxArray), buf, buf + n); //Append the new data to the rxArray buffer

        }else if(n == -1){
            printf("Error on  SerialConnection::readRxData= -1");
            throw 6;

        }else if(n == 0 && i < rxSize-1){

            printf("serialConnection->readRxData %d bytes from %d received\n", i, rxSize);
			
 
            return true;
		}
	}
	if( rxArray.size() > 0) processData(rxArray, rxSize);
   	return true;
 }

/**
 * @brief Process the received data
 *
 * @param array Pointer to the received byte array
 */
bool SerialConnection::processData(std::vector<uint8_t> &array, int rxSize)
{        
    if (array.size() == 0){
        printf("ERROR SerialConnection::processData array.size() = 0\n");
        return false;
    }

    //Check for the marking byte
    if( array.at(0) != CommunicationConstants::Data::START_MARK){
        array.erase(array.begin(), array.begin() + 1 );
        return true; //--------------------------------->
    }

    //Cancel here if no marking bytes

    //Get the expected size. Cancel here if not enough bytes received
    if (array.size() < (static_cast<int>(CommunicationConstants::Data::SIZE_HEADER)))
        return true; //--------------------------------->

    //Get the expexted size
    expectedSize = getExpextedSize(array);

    //Cancel here if not enough bytes received
    if (array.size() < (static_cast<int>(expectedSize + CommunicationConstants::Data::SIZE_OVERHEAD)) )
        return true; //--------------------------------->

	

    //Check if the end marking is present. Only use the data if this is the case.
    if ( checksumIsCorrect(array, expectedSize) )
    {
        uint8_t type = getType(array);

        //Remove the remaining: thats the checksum
		array.erase(array.begin(), array.begin() + CommunicationConstants::Data::SIZE_HEADER);
		array.erase(array.begin() + expectedSize,  array.begin() + expectedSize + CommunicationConstants::Data::SIZE_CHECKSUM);

//        printf("SerialConnection::processData:  sigReceivedData expectedSize = %d type = %x size = %d\n", expectedSize, type, array.size()); //TODO remove
        sigReceivedData(array, type);
		array.erase(array.begin(), array.begin() + expectedSize );
		if( (type < 3 || type > 8) ){
			if( array.size() > 0 ) printf("after size = %lu rxSize = %d at(0) = %x\n", array.size(), rxSize, array.at(0));
		}

		return true;
    }else{
        printf("-------- >>>> corrupted data <<< ---------\n");
        //array.erase(array.begin(), array.begin() + 1);
		array.clear();
    }

  //array.clear();

  return false;
}

int SerialConnection::set_interface_attribs(int speed, int parity)
{
  /* boost::asio::serial_port_base::baud_rate baud_option;
  //serialPort->set_option(baud_option); // set the baud rate after the port has been opened
  serialPort->set_option( asio_serial::baud_rate(speed));
  serialPort->set_option( asio_serial::flow_control( asio_serial::flow_control::none ) );
  serialPort->set_option( asio_serial::parity( asio_serial::parity::none ) );
  serialPort->set_option( asio_serial::stop_bits( asio_serial::stop_bits::one ) );
  serialPort->set_option( asio_serial::character_size(8));
  //serialPort->baud_rate baud_option;
  serialPort->get_option(baud_option);
  int value = baud_option.value();
  ROS_INFO("SerialConnection::set_interface_attribs boudrate fileDesctription: %d  %d"  , value, fileDescription);
  //ioctl (fd_, IOSSIOSPEED, &new_baud, 1);
  struct termios2 tio;
  ioctl(fileDescription, TCGETS2, &tio);
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = speed;
  tio.c_ospeed = speed;
  int r = ioctl(fileDescription, TCSETS2, &tio);
  if (r == 0) printf("Changed successfully.\n");
  else        perror("ioctl");*/

  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fileDescription, &tty) != 0)
  {
    ROS_ERROR("Error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);


  tty.c_oflag = 0;				  // no remapping, no delays
  tty.c_oflag &= ~(ONLCR | OCRNL); //TODO...
  
  tty.c_lflag = 0;				  // no signaling chars, no echo,
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); //TODO...
  
  tty.c_iflag &= ~IGNBRK;		  // disable break processing
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_iflag &= ~(INLCR | IGNCR | ICRNL); //TODO...
  
  tty.c_cc[VMIN]  = 0;			  // non-blocking read
  tty.c_cc[VTIME] = 1; 		   // 1 second read timeout
    
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;	  // 8-bit chars				
  tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
  tty.c_cflag &= ~CSTOPB;	 //one stop bit
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls	 
  tty.c_cflag |= CRTSCTS;	//data DTR hardware control do not use it
  
  tcflush(fileDescription, TCIOFLUSH);

  if (tcsetattr (fileDescription, TCSANOW, &tty) != 0){
      printf("Error %d from tcsetattr\n", errno);
      return -1;
  }

  return 0;
}


void SerialConnection::set_blocking (int should_block){

 struct termios tty;
  memset (&tty, 0, sizeof tty);

  if (tcgetattr (fileDescription, &tty) != 0){
      ROS_ERROR("Error %d from tggetattr", errno);
      return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

  if (tcsetattr (fileDescription, TCSANOW, &tty) != 0)
      ROS_ERROR ("Error %d setting term attributes", errno);
}


/**
 * @brief Send a command
 *
 * This function sends a command. It adds the marking and the checksum and sends it
 * to the device.
 *
 * Important: The size of the buffer must be big enough to add the markings an the checksum.
 *
 * @param data Pointer to the data to send
 */
bool SerialConnection::sendCommand(uint8_t *data, uint8_t expectedType, bool blocking)
{
  if (fileDescription <= 0)
  {
    return false;
  }

  //If a command is still ongoing, just put the data into the queue to send it later
  if(waitForSpecificDataType)
  {
    Message newMessage(data, expectedType);
    queue.emplace(newMessage);
    return false;
  }

  sendCommandInternal(data, expectedType);

  //If a blocking command is desired, wait until the answer is received
  if(blocking)
  {
    //Process the events. Otherwise everything will block
    while(waitForSpecificDataType)
    {
      //QCoreApplication::processEvents();
    }

    //This flag is set by the timeout timer signal
    /*if (timeout) //TODO...
    {
      return false;
    }*/
  }

  return true;
}

ssize_t SerialConnection::sendCommandInternal(uint8_t *data, uint8_t expectedType_)
{
  this->expectedType = expectedType_;
  this->waitForSpecificDataType = true;
  //timeoutTimer->start(TIMEOUT);
  //timeout = false;

  //Add the start buffer at the beginning
  data[0] = CommunicationConstants::Command::START_MARK;

  //Calculate the CRC
  uint32_t crc = calculateChecksum(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)));

  //Add it to the buffer
  Util::setUint32LittleEndian(data, (CommunicationConstants::Command::SIZE_TOTAL - sizeof(uint32_t)), crc);

  return write(fileDescription, (uint8_t *)(data), CommunicationConstants::Command::SIZE_TOTAL);
}

uint32_t SerialConnection::calculateChecksum(const uint8_t *data, const uint32_t size)
{
  return CrcCalc::calcCrc32_32(data, size);
}

} //end namespace ComLib
