
/**
 * @file cmd.h
 * @brief Orbbec-specific hardware command interface via raw USB control transfers.
 * 
 * This file defines the low-level command interface used to communicate directly
 * with Orbbec camera hardware for tasks not covered by the standard OpenNI API,
 * such as controlling the emitter, flood LED, and LDP, or reading/writing I2C registers.
 */

#pragma once

#ifdef WIN32
#include <XnPlatform.h>
#include <XnOS.h>
#include <XnUSB.h>
#endif
#include <PS1080.h>
#include <OpenNI.h>
#include <vector>
#include <map>

#ifdef LINUX
#include "libusb.h"
#endif

using namespace openni;
using namespace std;

/**
 * @brief Callback function type for command processing.
 */
typedef bool (*cbfunc)(vector<string>& Command);

/**
 * @brief USB Opcodes for Orbbec vendor-specific commands.
 * These opcodes are used in the protocol_header to identify the hardware command.
 */
enum EPsProtocolOpCodes
{
	OPCODE_GET_VERSION = 0,             ///< Retrieve hardware/firmware version
	OPCODE_GET_SERIAL_NUMBER = 37,      ///< Retrieve camera serial number
	OPCODE_ENABLE_EMITTER = 42,         ///< (Standard) Enable/Disable laser emitter
	OPCODE_ENABLE_FLOOD_LED = 43,       ///< Enable/Disable IR flood LED
	OPCODE_GET_FLOOD_LED_STATUS = 44,   ///< Query current IR flood LED status
	OPCODE_GET_LASER_STATUS = 45,       ///< Query current laser emitter status
    
	// Custom Orbbec additions and specific hardware controls
	CMD_IR_EXPOSURE_GET = 78,           ///< Get IR sensor exposure value
	CMD_IR_GAIN_GET = 79,               ///< Get IR sensor gain value
	CMD_ENABLE_LDP1 = 83,               ///< LDP control part 1
	CMD_ENABLE_LDP2 = 84,               ///< LDP control part 2
	CMD_ENABLE_EMITTER = 85,            ///< (Orbbec-specific) Enable/Disable emitter
	CMD_GAIN_SET = 87,                  ///< Set sensor gain
	CMD_EXP_SET = 96,                   ///< Set sensor exposure
	CMD_GET_CAMERA_PARA = 97,           ///< Get internal camera calibration parameters
	OPCODE_KILL = 999,                  ///< Termination command
};

/**
 * @brief Status of the laser emitter.
 */
enum LaserStatus { LASER_OFF = 0, LASER_ON = 1, LASER_UNKNOWN = 0xff };

/**
 * @brief Status of the IR flood LED.
 */
enum IrFloodLedStatus { IR_LED_OFF = 0, IR_LED_ON = 1, IR_LED_UNKNOWN = 0xff };

/**
 * @brief Status of the Laser Detection and Protection (LDP) system.
 */
enum LDPStatus { LDP_OFF = 0, LDP_ON = 1, LDP_UNKNOWN = 0xff };

/**
 * @brief Binary header for Orbbec USB control packets.
 * Every control transfer starts with this 8-byte header.
 */
typedef struct {
	uint16_t magic;  ///< Magic number (0x4d47) identifying the start of a packet
	uint16_t size;   ///< Size of the data following the header (in 16-bit words)
	uint16_t opcode; ///< Command opcode from EPsProtocolOpCodes
	uint16_t id;     ///< Sequence ID for tracking requests and responses
} protocol_header;

#define CMD_HEADER_MAGIC	(0x4d47)
#define CMD_HEADER_LEN		(0x08)

/**
 * @brief Helper to convert string to integer (supports decimal and hex '0x').
 */
bool atoi2(const char* str, int* pOut);

/**
 * @brief Manages hardware-level communication with the Orbbec device.
 * 
 * This class wraps OpenNI device management and provides methods for raw USB
 * control transfers to interact with specific Orbbec hardware features.
 */
class cmd
{
public:
	cmd(void);
	~cmd(void);

	/**
	 * @brief Initializes OpenNI and identifies the low-level USB interface.
	 * Detects whether the device is Astra Pro or other models to handle RGB correctly.
	 * @return 0 on success, non-zero error code otherwise.
	 */
	int init(void);

	/**
	 * @brief Retrieves the firmware version from the device.
	 */
	int get_version(void);

	/**
	 * @brief Retrieves the unique serial number of the camera.
	 */
	int get_sn_number(void);
    
    /**
     * @brief Retrieves internal camera calibration parameters (intrinsics/extrinsics).
     * These are stored in the device's persistent memory.
     */
	int get_cmos_params(void);
	
	/**
	 * @brief Reads an I2C register from the specified sensor (depth or image).
	 * @param Command Vector containing command arguments.
	 * @param I2C Structure to store the register address and returned value.
	 * @return The value read from the register.
	 */
	unsigned short read_i2c(vector<string>& Command, XnControlProcessingData& I2C);

	/**
	 * @brief Writes a value to an I2C register on the specified sensor.
	 */
	bool write_i2c(vector<string>& Command, XnControlProcessingData& I2C);
	
    // Sensor and Peripheral Controls
	int ir_flood_set(bool status);
	int ir_flood_get(enum IrFloodLedStatus &status);
	int ir_exposure_set(uint32_t val);
	int ir_exposure_get(uint32_t &val);
	bool ir_gain_set(const char* val);
	bool ir_gain_get(void);
	int emitter_set(bool emitter_status);
	int emitter_get(enum LaserStatus &status);
	int ldp_set(bool ldp_status);
	int ldp_get(enum LDPStatus &status);

	/**
	 * @brief Main interactive command loop (unused in this specific GUI application).
	 */
	void mainloop(istream& istr, bool prompt);

	/**
	 * @brief Registers a callback function for the command loop.
	 */
	void RegisterCB(string cmd, cbfunc func, const string& strHelp);
	void RegisterMnemonic(string strMnemonic, string strCommand);

	Device device;           ///< The main OpenNI device handle
	unsigned short m_I2CReg; ///< Cache for last I2C register value

	int m_vid;               ///< Vendor ID (usually 0x2bc5)
	int m_pid;               ///< Product ID (identifies model like Astra Pro)

	// Current hardware status caches
	enum IrFloodLedStatus status_IrFlood;
	enum LaserStatus status_Laser;
	enum LDPStatus status_LDP;

private:
	uint16_t seq_num;        ///< Monotonically increasing sequence ID for USB packets
	uint8_t	req_buf[512];    ///< Buffer for outgoing USB requests
	uint8_t	resp_buf[512];   ///< Buffer for incoming USB responses

	// Alias for request buffer as different types
	protocol_header *pheader;
	uint8_t *obuf;
	uint8_t *ibuf;
	
#ifdef WIN32	
	XN_USB_DEV_HANDLE m_hUSBDevice;	
	const XnUSBConnectionString* m_astrDevicePaths;
#else
	libusb_device_handle *handle;
	libusb_device *dev;
	libusb_device **devs;
	const char* m_astrDevicePaths;
#endif

private:
	/**
	 * @brief Prepares the standard 8-byte header in the buffer.
	 */
	int init_header(void *buf, uint16_t cmd, uint16_t data_len);

	/**
	 * @brief Synchronously sends a control transfer and waits for a response.
	 */
	int send(void *cmd_req, uint16_t req_len, void *cmd_resp, uint16_t *resp_len);

	// Internal command registration maps
	map<string, cbfunc> cbs;
	map<string, cbfunc> mnemonics;
	map<string, string> helps;
};

/**
 * @brief Structure containing factory calibration parameters.
 * Includes focal lengths, principal points, rotation, and translation.
 */
typedef struct OBCameraParams
{
	float l_intr_p[4]; ///< Left (IR) intrinsic parameters: [fx, fy, cx, cy]
	float r_intr_p[4]; ///< Right (RGB) intrinsic parameters: [fx, fy, cx, cy]
	float r2l_r[9];    ///< Rotation matrix from Right to Left sensor: [r00, r01, r02; r10, r11, r12; r20, r21, r22]
	float r2l_t[3];    ///< Translation vector from Right to Left sensor: [t1, t2, t3]
	float l_k[5];      ///< Left sensor distortion coefficients: [k1, k2, k3, p1, p2]
	float r_k[5];      ///< Right sensor distortion coefficients: [k1, k2, k3, p1, p2]
	int is_m;          ///< Mirroring status
}OBCameraParams;

#define IsMirroredTrue		1
#define IsMirroredFalse		2
