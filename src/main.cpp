#include <iostream>
#include <string.h>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#pragma warning(disable:4996)

libusb_device_handle* devHandle = NULL;

// Vendor ID
#define	VENDOR_ID			0x191A

// Device ID
#define	DEVICE_ID			0x6001

// Command version
#define	COMMAND_VERSION		0x00

// Command ID
#define	COMMAND_ID_CONTROL	0x00
#define	COMMAND_ID_SETTING	0x01
#define	COMMAND_ID_GETSTATE	0x80

// Endpoint address for sending to host -> USB controlled multicolor indicator
#define	ENDPOINT_ADDRESS	1
// Endpoint address for sending to USB -> host controlled multicolor indicator
#define	ENDPOINT_ADDRESS_GET	0x81

// Time-out time when sending a command
#define	SEND_TIMEOUT		1000

// Protocol data area size
#define	SEND_BUFFER_SIZE	8
#define	RECV_BUFFER_SIZE	2

// LED color
#define LED_COLOR_OFF				0		// Off
#define LED_COLOR_RED				1		// Red
#define LED_COLOR_GREEN				2		// green
#define LED_COLOR_YELLOW			3		// yellow
#define LED_COLOR_BLUE				4		// Blue
#define LED_COLOR_PURPLE			5		// purple
#define LED_COLOR_LIGHTBLUE			6		// Sky blue
#define LED_COLOR_WHITE				7		// White
#define LED_COLOR_KEEP				0x0F	// Maintain the setting of the phenomenon

// LED pattern
#define	LED_OFF						0x00	// Off
#define	LED_ON						0x01	// Lit
#define	LED_PATTERN1				0x02	// LED pattern1
#define	LED_PATTERN2				0x03	// LED pattern2
#define	LED_PATTERN3				0x04	// LED pattern3
#define	LED_PATTERN4				0x05	// LED pattern4
#define	LED_PATTERN5				0x06	// LED pattern5
#define	LED_PATTERN6				0x07	// LED pattern6
#define	LED_PATTERN_KEEP			0x0F	// Maintain the setting of the phenomenon

// Number of buzzers
#define	BUZZER_COUNT_CONTINUE		0x00	// Continuous operation
#define	BUZZER_COUNT_KEEP			0x0F	// Maintain the setting of the phenomenon

// Buzzer pattern
#define	BUZZER_OFF					0x00	// Stop
#define	BUZZER_ON					0x01	// Blow (continuous)
#define	BUZZER_SWEEP				0x02	// Sweep sound
#define	BUZZER_INTERMITTENT			0x03	// Intermittent sound
#define	BUZZER_WEEK_ATTENTION		0x04	// Weak caution sound
#define	BUZZER_STRONG_ATTENTION		0x05	// Strong attention sound
#define BUZZER_SHINING_STAR			0x06	// shining star
#define BUZZER_LONDON_BRIDGE		0x07	// London bridge
#define	BUZZER_KEEP					0x0F	// Maintain the setting of the phenomenon

// Buzzer volume
#define	BUZZER_VOLUME_OFF			0x00	// Mute
#define	BUZZER_VOLUME_MAX			0x0A	// Maximum volume
#define	BUZZER_VOLUME_KEEP			0x0F	// Maintain the setting of the phenomenon

// Setting
#define SETTING_OFF					0x00	// OFF
#define SETTING_ON					0x01	// ON

// openings
#define BLANK						0x00	// openings

int UsbOpen();
void UsbClose();
int SendCommand(char* sendData, int sendLength);
int SetLight(unsigned char color, unsigned char state);
int SetBuz(unsigned char buz_state, unsigned char limit);
int SetVol(unsigned char volume);
int SetBuzEx(unsigned char buz_state, unsigned char limit, unsigned char volume);
int SetSetting(unsigned char setting);
int GetTouchSensorState();
int Reset();

/// <summary>
/// Main function
/// </summary>
int main(int argc, char* argv[])
{
	int ret;
	
	// Connect to USB control multi-color indicator via USB communication
	ret = UsbOpen();
    if (ret == -1)
    {
        std::cout << "device not found" << std::endl;
        return -1;
    }

    // Get the command identifier specified by the command line argument
    char* commandId = NULL;
    if (argc > 1) {
        commandId = argv[1];
    	int length = strlen(commandId);
    	if(length != 1){
    		commandId[0] = '0';
    	}
    }

    switch (*commandId)
    {
        case '1':
        {
            // Specify the LED color and LED pattern to turn on and turn on the pattern
            if (argc >= 4)
                SetLight(atoi(argv[2]), atoi(argv[3]));
            break;
        }

        case '2':
        {
            // Specify the buzzer pattern and make the buzzer sound
            if (argc >= 4)
                SetBuz(atoi(argv[2]), atoi(argv[3]));
            break;
        }

        case '3':
        {
            // Change the buzzer volume by specifying the volume
            if (argc >= 3)
                SetVol(atoi(argv[2]));
            break;
        }

        case '4':
        {
            // Sound the buzzer by specifying the buzzer pattern, number of times, and volume.
            if (argc >= 5)
                SetBuzEx(atoi(argv[2]), atoi(argv[3]), atoi(argv[4]));
            break;
        }

        case '5':
        {
            // Change the connection display settings
            if (argc >= 3)
                SetSetting(atoi(argv[2]));
            break;
        }

        case '6':
        {
            // Get touch sensor input status
            int state = GetTouchSensorState();
            if(state == 1){
                puts("touch sensor input = ON");
            }
            else if(state == 0){
                puts("touch sensor input = OFF");
            }
            else{
                puts("USB communication failed");
            }
            break;
        }

        case '7':
        {
            // Turn off the LED and stop the buzzer
            Reset();
            break;
        }

    }
	
    // Close socket
    UsbClose();
	
	return 0;
}

/// <summary>
/// Connect to USB control multi-color indicator via USB communication
/// </summary>
/// <returns>Success: 0, Failure: Other than 0</returns>
int UsbOpen()
{
	int ret;
	
	// initialize
	ret = libusb_init(NULL);
	if (ret < 0) {
		return -1;
	}
	
	// Open device
	devHandle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, DEVICE_ID);
	if (devHandle == NULL) {
		return -1;
	}

    // Kernel driver detachment
	libusb_detach_kernel_driver(devHandle, 0);
	ret = libusb_claim_interface(devHandle, 0);
	if (ret!= LIBUSB_SUCCESS) {
		return ret;
	}
	
	return 0;
}

/// <summary>
/// End USB communication with USB control multi-color indicator
/// </summary>
void UsbClose()
{
	if (devHandle != NULL) {
		libusb_close(devHandle);
	}
}

/// <summary>
/// Send command
/// </summary>
/// <param name="sendData">Transmission data</param>
/// <param name="sendLength">Send data size</param>
/// <returns>Success: 0, Failure: Other than 0</returns>
int SendCommand(unsigned char* sendData, int sendLength)
{
	int ret;
	int length = 0;

	if (devHandle == NULL) {
        std::cout << "device handle is not null" << std::endl;
		return -1;
	}

	// send
	ret = libusb_interrupt_transfer(devHandle, ENDPOINT_ADDRESS, sendData, sendLength, &length, SEND_TIMEOUT);
	if (ret != 0) {
        std::cout << "failed to send" << std::endl;
		return -1;
	}

	return 0;
}

/// <summary>
/// Specify the LED color and LED pattern to turn on the USB control multi-color indicator and turn on the pattern.
/// Buzzer pattern and buzzer volume maintain their current state
/// </summary>
/// <param name="color">LED color to control (off: 0, red: 1, green: 2, yellow: 3, blue: 4, purple: 5, sky blue: 6, white: 7, keep the current settings: 0x8-0xF)</param>
/// <param name="state">LED pattern (off: 0x00, on: 0x01, LED pattern 1: 0x02, LED pattern 2: 0x03, LED pattern 3: 0x04, LED pattern 4: 0x05, LED pattern 5: 0x06, LED pattern 6: 0x07, current settings Maintain: 0x08-0x0F)</param>
/// <returns>Success: 0, Failure: Other than 0</returns>
int SetLight(unsigned char color, unsigned char state)
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    std::memset(sendData, 0, sizeof(sendData));

    // Argument range check
    if((0x0F < color) || (0x0F < state)){
    	return -1;
    }
    
	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_CONTROL;

	// Buzzer control (maintain the status quo)
    sendData[2] = (BUZZER_COUNT_KEEP << 4) | BUZZER_KEEP;

    // Buzzer volume (maintain the status quo)
    sendData[3] = BUZZER_VOLUME_KEEP;

    // LED control
    sendData[4]  = (color << 4) | state;

    // Free (0x00: fixed)
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        std::cout << "failed to send data" << std::endl;
        return -1;
    }

    return 0;
}

/// <summary>
/// Specify the buzzer pattern and make the buzzer sound
/// LED and buzzer volume maintain current state
/// </summary>
/// <param name="buz_state">Buzzer pattern (stop: 0x00, continuous sound: 0x01, sweep sound: 0x02, intermittent sound: 0x03, weak caution sound: 0x04, strong caution sound: 0x05, glitter star: 0x06, London Bridge: 0x07, maintain the current settings: 0x08-0x0F)</param>
/// <param name="limit">Continuous operation: 0, Number of operations: 1 to 14, Maintain current settings: 0x0F</param>
/// <returns>Success: 0, Failure: Other than 0</returns>
int SetBuz(unsigned char buz_state, unsigned char limit)
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    std::memset(sendData, 0, sizeof(sendData));

    // Argument range check
    if((0x0F < buz_state) || (0x0F < limit)){
    	return -1;
    }
    
	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_CONTROL;

	// Buzzer control
    sendData[2] = (limit << 4) | buz_state;

    // Buzzer volume (maintain the status quo)
    sendData[3] = BUZZER_VOLUME_KEEP;

    // LED control (maintaining the status quo)
    sendData[4]  = (LED_COLOR_KEEP << 4) | LED_PATTERN_KEEP;

    // Free (0x00: fixed)
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        std::cout << "failed to send data" << std::endl;
        return -1;
    }

    return 0;
}

/// <summary>
/// Change the buzzer volume by specifying the volume
/// LED and buzzer patterns maintain their current state
/// </summary>
/// <param name="volume">Volume (silence: 0x00, step volume: 0x01 to 0x09, maximum volume: 0x0A, maintain the current settings: 0x0B to 0x0F)</param>
/// <returns>Success: 0, Failure: Other than 0</returns>
int SetVol(unsigned char volume)
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    std::memset(sendData, 0, sizeof(sendData));

    // Argument range check
    if(0x0F < volume){
    	return -1;
    }
    
	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_CONTROL;

	// Buzzer control (maintain the status quo)
    sendData[2] = (BUZZER_COUNT_KEEP << 4) | BUZZER_KEEP;

    // Buzzer volume
    sendData[3] = volume;

    // LED control (maintaining the status quo)
    sendData[4]  = (LED_COLOR_KEEP << 4) | LED_PATTERN_KEEP;

    // Free (0x00: fixed)
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        std::cout << "failed to send data" << std::endl;
        return -1;
    }

    return 0;
}

/// <summary>
/// Sound the buzzer by specifying the buzzer pattern, number of times, and volume.
/// </summary>
/// <param name="buz_state">Buzzer pattern (stop: 0x00, continuous sound: 0x01, sweep sound: 0x02, intermittent sound: 0x03, weak caution sound: 0x04, strong caution sound: 0x05, glitter star: 0x06, London Bridge: 0x07, maintain the current settings: 0x08-0x0F)</param>
/// <param name="limit">Continuous operation: 0, Number of operations: 1 to 14, Maintain current settings: 0x0F</param>
/// <param name="volume">Volume (silence: 0x00, step volume: 0x01 to 0x09, maximum volume: 0x0A, maintain the current settings: 0x0B to 0x0F)</param>
/// <returns>Success: 0, Failure: Other than 0</returns>
int SetBuzEx(unsigned char buz_state, unsigned char limit, unsigned char volume)
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    std::memset(sendData, 0, sizeof(sendData));

    // Argument range check
    if((0x0F < buz_state) || (0x0F < limit) || (0x0F < volume)){
    	return -1;
    }
    
	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_CONTROL;

	// Buzzer control
    sendData[2] = (limit << 4) | buz_state;

    // Buzzer volume
    sendData[3] = volume;

    // LED control (maintaining the status quo)
    sendData[4]  = (LED_COLOR_KEEP << 4) | LED_PATTERN_KEEP;

    // Free (0x00: fixed)
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        std::cout << "failed to send data" << std::endl;
        return -1;
    }

    return 0;
}

/// <summary>
/// Change the connection display settings
/// </summary>
/// <param name="setting">Settings (OFF: 0x00, ON: 0x01)</param>
/// <returns>Success: 0, Failure: Other than 0</returns>
int SetSetting(unsigned char setting)
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    std::memset(sendData, 0, sizeof(sendData));

    // Argument range check
    if((SETTING_OFF != setting) && (SETTING_ON != setting)){
    	return -1;
    }
    
	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_SETTING;

	// Setting
    sendData[2] = setting;

    // Free (0x00: fixed)
    sendData[3] = BLANK;
    sendData[4] = BLANK;
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        std::cout << "failed to send data" << std::endl;
        return -1;
    }

    return 0;
}

/// <summary>
/// Get touch sensor input status
/// </summary>
/// <returns>Acquisition failure:-1, Touch sensor input OFF:0縲ゝouch sensor input ON:1</returns>
int GetTouchSensorState()
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    memset(sendData, 0, sizeof(sendData));

	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_GETSTATE;

    // Free (0x00: fixed)
    sendData[2] = BLANK;
    sendData[3] = BLANK;
    sendData[4] = BLANK;
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        puts("failed to send data");
        return -1;
    }

	// Receive response
    unsigned char getData[RECV_BUFFER_SIZE];
    memset(getData, 0, sizeof(getData));
	int length = 0;
	ret = libusb_interrupt_transfer(devHandle, ENDPOINT_ADDRESS_GET, getData, sizeof(getData), &length, SEND_TIMEOUT);
	if (ret != 0) {
		puts("failed to receive data");
		return -1;
	}
	
	if ((getData[1] & 1) == 1) {
		return 1;
	}
	return 0;
}

/// <summary>
/// Turn off the LED and stop the buzzer
/// </summary>
/// <returns>Success: 0, Failure: Other than 0</returns>
int Reset()
{
	int ret;
    unsigned char sendData[SEND_BUFFER_SIZE];
    std::memset(sendData, 0, sizeof(sendData));

	// Command version (0x00: fixed)
    sendData[0] = COMMAND_VERSION;

    // Command ID
    sendData[1] = COMMAND_ID_CONTROL;

	// Buzzer control(The number of times is maintained as it is)
    sendData[2] = (BUZZER_COUNT_KEEP << 4) | BUZZER_OFF;

    // Buzzer volume (maintain the status quo)
    sendData[3] = BUZZER_VOLUME_KEEP;

    // LED control
    sendData[4]  = (LED_COLOR_OFF << 4) | LED_OFF;

    // Free (0x00: fixed)
    sendData[5] = BLANK;
    sendData[6] = BLANK;
    sendData[7] = BLANK;

    // Send command
    ret = SendCommand(sendData, sizeof(sendData));
    if (ret != 0) {
        std::cout << "failed to send data" << std::endl;
        return -1;
    }

    return 0;
}
