#include "cmd.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

/**
 * @brief Helper to convert string to integer (supports decimal and hex '0x').
 */
bool atoi2(const char* str, int* pOut)
{
    int output = 0;
    int base = 10;
    int start = 0;

    if (strlen(str) > 1 && str[0] == '0' && str[1] == 'x')
    {
        start = 2;
        base = 16;
    }

    for (size_t i = start; i < (int)strlen(str); i++)
    {
        output *= base;
        if (str[i] >= '0' && str[i] <= '9')
            output += str[i] - '0';
        else if (base == 16 && str[i] >= 'a' && str[i] <= 'f')
            output += 10 + str[i] - 'a';
        else if (base == 16 && str[i] >= 'A' && str[i] <= 'F')
            output += 10 + str[i] - 'A';
        else
            return false;
    }
    *pOut = output;
    return true;
}

cmd::cmd(void)
{
    m_vid = 0x2bc5; // Default Orbbec VID
    m_pid = 0x0401; // Default Astra PID
    seq_num = 0x00;
    status_IrFlood = IR_LED_UNKNOWN;
    status_Laser = LASER_UNKNOWN;
    status_LDP = LDP_UNKNOWN;

    // Buffer Aliasing
    obuf = req_buf;
    ibuf = resp_buf;
    pheader = (protocol_header *)req_buf;

    #ifdef WIN32
    m_hUSBDevice = NULL;
    #endif

    #ifdef LINUX
    handle = NULL;
    #endif
}


cmd::~cmd(void)
{
    device.close(); // Ensure OpenNI device is closed
}

/**
 * @brief Initializes OpenNI and the platform-specific USB communication layer.
 * 
 * This method:
 * 1. Initializes the OpenNI 2 environment.
 * 2. Opens the first available OpenNI device.
 * 3. Initializes the low-level USB driver (xnUSB on Windows, libusb on Linux).
 * 4. Matches the OpenNI device to its raw USB counterpart for control transfers.
 */
int cmd::init(void)
{
    DeviceInfo dInfo;
    uint32_t n = 0;
    
    // Step 1: Initialize OpenNI
    int rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return rc;
    }

#ifdef WIN32
    // Step 2: Open OpenNI Device
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return rc;
    }
    dInfo = device.getDeviceInfo();
    printf("vid %04x, pid %04x\n", dInfo.getUsbVendorId(), dInfo.getUsbProductId());
    m_vid = dInfo.getUsbVendorId();
    m_pid = dInfo.getUsbProductId();
    
    // Step 3: Initialize Windows xnUSB driver
    int nRetVal = xnUSBInit();
    if (nRetVal == XN_STATUS_USB_ALREADY_INIT)
        nRetVal = XN_STATUS_OK;

    // Step 4: Find the matching USB device for control transfers
    rc = xnUSBEnumerateDevices(m_vid, m_pid, &m_astrDevicePaths, &n);
    if(rc != STATUS_OK)
    {
        cout << " Error: failed to enumerate usb devices" << endl;
        return -1;
    }

    rc = xnUSBOpenDeviceByPath(*m_astrDevicePaths, &m_hUSBDevice);
    if (rc != STATUS_OK)
    {
        cout << " Error: failed to open device" << m_vid << m_pid << endl;
        return -1;
    }
#endif

#ifdef LINUX
    // Step 2: Enumerate and open OpenNI device via URI
    openni::Array<DeviceInfo> deviceList;
    char uri[16];
    
    OpenNI::enumerateDevices(&deviceList);
    if (deviceList.getSize() == 0) {
        printf("No devices connected\n");
        return -1;
    }
    dInfo = deviceList[0];

    rc = device.open(dInfo.getUri());
    m_astrDevicePaths = dInfo.getUri();

    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return rc;
    }
    dInfo = device.getDeviceInfo();
    printf("vid %04x, pid %04x\n", dInfo.getUsbVendorId(), dInfo.getUsbProductId());
    m_vid = dInfo.getUsbVendorId();
    m_pid = dInfo.getUsbProductId();

    // Step 3: Initialize libusb
    int nRetVal = libusb_init(NULL);
    if(nRetVal !=0)
        return -1;

    // Step 4: Iterate libusb devices to find the one matching the OpenNI URI
    int cnt = libusb_get_device_list(NULL, &devs);
    if(cnt < 0)
    {
        printf("failed to list device\n");
        return -1;
    }

    int i = 0;
    struct libusb_device_descriptor desc;

    while ((dev = devs[i++]) != NULL)
    {
        int res = libusb_get_device_descriptor(dev, &desc);
        if (res < 0) continue;

        int vid = desc.idVendor;
        int pid = desc.idProduct;
        int bus = libusb_get_bus_number(dev);
        int addr = libusb_get_device_address(dev);

        sprintf(uri,"%04x/%04x@%d/%d", vid, pid, bus, addr);

        if (0 == strncmp(uri, m_astrDevicePaths, 14))
        {
            // Found the matching hardware device
            break;
        }
    }

    rc = libusb_open(dev, &handle);
#endif

    return rc;
}

/**
 * @brief Standard interactive loop for executing commands from a stream.
 */
void cmd::mainloop( istream& istr, bool prompt)
{
    char buf[256];
    string str;
    vector<string> Command;

    while (istr.good())
    {
        if (prompt)
            cout << "> ";
        Command.clear();
        istr.getline(buf, 256);
        str = buf;
        size_t previous = 0, next = 0;

        // Simple space-based tokenizer
        while (1)
        {
            next = str.find(' ', previous);

            if (next != previous && previous != str.size())
                Command.push_back(str.substr(previous, next - previous));

            if (next == str.npos)
                break;

            previous = next + 1;
        }

        if (Command.size() > 0)
        {
            if (Command[0][0] == ';') continue; // Comment support

            for (unsigned int i = 0; i < Command[0].size(); i++)
                Command[0][i] = (char)tolower(Command[0][i]);

            if (cbs.find(Command[0]) != cbs.end())
            {
                if (!(*cbs[Command[0]])(Command)) return;
            }
            else if (mnemonics.find(Command[0]) != mnemonics.end())
            {
                if (!(*mnemonics[Command[0]])(Command)) return;
            }
            else
            {
                cout << "Unknown command \"" << Command[0] << "\"" << endl;
            }
        }
    }
}

void cmd::RegisterCB(string cmd_str, cbfunc func, const string& strHelp)
{
    for (unsigned int i = 0; i < cmd_str.size(); i++)
        cmd_str[i] = (char)tolower(cmd_str[i]);
    cbs[cmd_str] = func;
    helps[cmd_str] = strHelp;
}

void cmd::RegisterMnemonic(string strMnemonic, string strCommand)
{
    for (unsigned int i = 0; i < strCommand.size(); i++)
        strCommand[i] = (char)tolower(strCommand[i]);
    for (unsigned int i = 0; i < strMnemonic.size(); i++)
        strMnemonic[i] = (char)tolower(strMnemonic[i]);

    if (cbs.find(strCommand) != cbs.end())
    {
        mnemonics[strMnemonic] = cbs[strCommand];
    }
}

/**
 * @brief Prepares the Orbbec protocol header at the beginning of the buffer.
 */
int cmd::init_header(void *buf, uint16_t cmd_opcode, uint16_t data_len)
{
    protocol_header *h = (protocol_header *)buf;
    h->magic = CMD_HEADER_MAGIC;
    h->size = data_len / 2; // Size is in 16-bit words
    h->opcode = cmd_opcode;
    h->id = seq_num++;

    return 0;
}

/**
 * @brief Executes a synchronous USB control transfer (Vendor Request 0x00).
 * 
 * Every Orbbec command involves sending a request and immediately waiting for
 * a response via another control transfer.
 */
int cmd::send(void *cmd_req, uint16_t req_len, void *cmd_resp, uint16_t *resp_len)
{
    uint32_t actual_len;
    int rc;

#ifdef WIN32
    // Send Request
    rc = xnUSBSendControl(m_hUSBDevice, 
            XN_USB_CONTROL_TYPE_VENDOR, 
            0x00, 0x0000, 0x0000, 
            (XnUChar*)cmd_req, req_len, 500000);
    
    // Poll for Response
    do 
    {       
        xnUSBReceiveControl(m_hUSBDevice, 
            XN_USB_CONTROL_TYPE_VENDOR, 
            0x00, 0x0000, 0x0000, 
            (XnUChar *)cmd_resp, 0x200 , &actual_len, 500000);
    } while ((actual_len == 0) || (actual_len == 0x200));
#endif

#ifdef LINUX
    // Send Request
    rc = libusb_control_transfer(handle,
         LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR,
            0x00, 0x0000, 0x0000, (unsigned char *)cmd_req, req_len, 1000);

    // Poll for Response
    do
    {
        actual_len = libusb_control_transfer(handle,
             LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                0x00, 0x0000, 0x0000, (unsigned char *)cmd_resp, 0x200, 1000);
    } while((actual_len == 0) || (actual_len == 0x200));
#endif

    *resp_len = actual_len;
    return rc;
}

/**
 * @brief Queries the hardware serial number using a raw USB command.
 */
int cmd::get_sn_number(void)
{
    int ret;
    uint16_t data_len = 0;
    uint16_t resp_len;
    char sn[15];
    
    ret = init_header(req_buf, OPCODE_GET_SERIAL_NUMBER, data_len);
    if (ret) return ret;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret < 0) {
        cout << "send cmd get serial number failed" << endl;
        return ret;
    }

    strncpy(sn, (char *)(resp_buf + 10), sizeof(sn));
    printf("Serial Number: %s\n", sn);
    return ret;
}

/**
 * @brief Queries the firmware version via the OpenNI property interface.
 */
int cmd::get_version(void)
{
    Status rc;
    char strPlatformString[XN_DEVICE_MAX_STRING_LENGTH];
    char strVersion[20];
    int size = sizeof(strPlatformString);

    rc = device.getProperty(XN_MODULE_PROPERTY_SENSOR_PLATFORM_STRING, strPlatformString, &size);
    if (rc != openni::STATUS_OK)
    {
        printf("Error: %s\n", openni::OpenNI::getExtendedError());
        return rc;
    }

    if (strPlatformString[0] != '\0')
    {
        strncpy(strVersion, strPlatformString + 2, sizeof(strVersion));
        printf("Firmware Version: %s\n", strVersion);
    }

    return rc;
}

/**
 * @brief Retrieves factory calibration parameters (intrinsics/extrinsics) from flash.
 */
int cmd::get_cmos_params(void)
{
    int ret;
    uint16_t data_len = 0;
    uint16_t resp_len;
    OBCameraParams param[1];

    memset(param, 0, sizeof(OBCameraParams));

    ret = init_header(req_buf, CMD_GET_CAMERA_PARA, data_len);
    if (ret) return ret;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret < 0) cout << "send cmd get_cmos_params failed" << endl;

    // Parameters start at offset 10 in the response packet
    memcpy((unsigned char *)param, resp_buf + 10, sizeof(OBCameraParams));
    
    printf("Mirroring: %s\n", (param[0].is_m == IsMirroredTrue) ? "Yes" : "No");

    printf("[IR Camera Intrinsic]\n %.3f %.3f %.3f %.3f\n",
        param[0].l_intr_p[0], param[0].l_intr_p[1],
        param[0].l_intr_p[2], param[0].l_intr_p[3]);
    printf("[RGB Camera Intrinsic]\n %.3f %.3f %.3f %.3f\n",
        param[0].r_intr_p[0], param[0].r_intr_p[1],
        param[0].r_intr_p[2], param[0].r_intr_p[3]);
    printf("[Rotate Matrix]\n %.3f %.3f %.3f\n %.3f %.3f %.3f\n %.3f %.3f %.3f\n",
        param[0].r2l_r[0], param[0].r2l_r[1], param[0].r2l_r[2],
        param[0].r2l_r[3], param[0].r2l_r[4], param[0].r2l_r[5],
        param[0].r2l_r[6], param[0].r2l_r[7], param[0].r2l_r[8]);
    printf("[Translate Matrix]\n %.3f %.3f %.3f\n",
        param[0].r2l_t[0], param[0].r2l_t[1], param[0].r2l_t[2]);

    return ret;
}

/**
 * @brief Reads an I2C register from the hardware sensors via the OpenNI property interface.
 */
unsigned short cmd::read_i2c(vector<string>& Command, XnControlProcessingData& I2C)
{
    if (Command.size() != 4)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <0:IR/1:RGB> <register>" << endl;
        return 0;
    }

    int nRegister;
    if (!atoi2(Command[3].c_str(), &nRegister)) return 0;
    I2C.nRegister = (unsigned short)nRegister;

    int nParam = 0;
    int sensorType;
    if (!atoi2(Command[2].c_str(), &sensorType)) return 0;

    if (sensorType == 1) nParam = XN_MODULE_PROPERTY_DEPTH_CONTROL;
    else if (sensorType == 0) nParam = XN_MODULE_PROPERTY_IMAGE_CONTROL;
    else return 0;

    if (device.getProperty(nParam, &I2C) != openni::STATUS_OK)
    {
        cout << "I2C Read failed!" << endl;
        return 0;
    }

    cout << "I2C(" << sensorType << ")[0x" << hex << I2C.nRegister << "] = 0x" << hex << I2C.nValue << endl;
    return I2C.nValue;
}

/**
 * @brief Writes a value to an I2C register via the OpenNI property interface.
 */
bool cmd::write_i2c(vector<string>& Command, XnControlProcessingData& I2C)
{
    if (Command.size() != 5)
    {
        cout << "Usage: " << Command[0] << " " << Command[1] << " <0:IR/1:RGB> <register> <value>" << endl;
        return true;
    }

    int nRegister, nValue;
    if (!atoi2(Command[3].c_str(), &nRegister)) return true;
    if (!atoi2(Command[4].c_str(), &nValue)) return true;
    
    I2C.nRegister = (unsigned short)nRegister;
    I2C.nValue = (unsigned short)nValue;

    int nParam = 0;
    int sensorType;
    if (!atoi2(Command[2].c_str(), &sensorType)) return true;

    if (sensorType == 1) nParam = XN_MODULE_PROPERTY_DEPTH_CONTROL;
    else if (sensorType == 0) nParam = XN_MODULE_PROPERTY_IMAGE_CONTROL;
    else return true;

    openni::Status rc = device.setProperty(nParam, I2C);
    if (rc != openni::STATUS_OK)
    {
        printf("I2C Write Error: %s\n", openni::OpenNI::getExtendedError());
    }

    return true;
}

/**
 * @brief Controls the Laser Detection and Protection (LDP) system.
 * This involves a sequence of specific memory writes to internal registers.
 */
int cmd::ldp_set(bool ldp_status)
{
    int ret;
    uint16_t data_len;
    uint16_t resp_len;
    
    uint32_t a;
    uint32_t *b = (uint32_t *)0x50005000;
    uint32_t r, _d;
    uint32_t v;
    uint8_t buf1[10];
    uint8_t buf2[10];

    _d = ldp_status & 0x01;
    a = (uint32_t)(b + 6);
    if (a % 4 != 0) return -1;

    *(uint32_t*)buf1 = a;
    data_len = 4;
    init_header(obuf, CMD_ENABLE_LDP1, data_len);
    memcpy(obuf + sizeof(*pheader), buf1, data_len);
    ret = send(obuf, data_len + sizeof(*pheader), ibuf, &resp_len);

    v = *(uint32_t*)&buf2[2];
    r = (v & (~(1 << 7))) | ((_d << 7));

    *(uint32_t*)buf1 = (uint32_t)(b + 6);
    *(uint32_t*)&buf1[4] = r;
    data_len = 8;
    init_header(obuf, CMD_ENABLE_LDP2, data_len);
    memcpy(obuf + sizeof(*pheader), buf1, data_len);
    ret = send(obuf, data_len + sizeof(*pheader), ibuf, &resp_len);

    *(uint32_t*)buf1 = (uint32_t)(b + 7);
    *(uint32_t*)&buf1[4] = ~r;
    init_header(obuf, CMD_ENABLE_LDP2, data_len);
    memcpy(obuf + sizeof(*pheader), buf1, data_len);
    ret = send(obuf, data_len + sizeof(*pheader), ibuf, &resp_len);

    status_LDP = ldp_status ? LDP_ON : LDP_OFF;
    return ret;
}

int cmd::ldp_get(enum LDPStatus &status)
{
    const char* s = (status == 0x01) ? "on" : (status == 0x00) ? "off" : "unknown";
    printf("LDP status: %s\n", s);
    return status;
}

/**
 * @brief Controls the IR laser emitter.
 */
int cmd::emitter_set(bool emitter_status)
{
    int ret;
    uint16_t data_len = 2;
    uint16_t resp_len;

    ret = init_header(req_buf, CMD_ENABLE_EMITTER, data_len);
    if (ret) return ret;

    req_buf[8] = emitter_status ? 0x01 : 0x00;
    req_buf[9] = 0x00;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret < 0) cout << "send Emitter failed" << endl;

    return ret;
}

int cmd::emitter_get(enum LaserStatus &status)
{
    int ret;
    uint16_t data_len = 0;
    uint16_t resp_len;

    ret = init_header(req_buf, OPCODE_GET_LASER_STATUS, data_len);
    if (ret) return ret;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret >= 0)
    {
        status = (req_buf[8] == 0x01) ? LASER_ON : (req_buf[8] == 0x00) ? LASER_OFF : LASER_UNKNOWN;
        printf("Emitter is %s.\n", (status == LASER_ON) ? "on" : (status == LASER_OFF) ? "off" : "unknown");
    }
    else cout << "send EmitterGet failed" << endl;

    return ret;
}

/**
 * @brief Controls the IR flood LED (used for passive IR imaging).
 */
int cmd::ir_flood_set(bool status)
{
    int ret;
    uint16_t data_len = 2;
    uint16_t resp_len;

    ret = init_header(req_buf, OPCODE_ENABLE_FLOOD_LED, data_len);
    req_buf[8] = status ? 0x01 : 0x00;
    req_buf[9] = 0x00;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret != 0) cout << "send IrFlood failed" << endl;

    return ret;
}

int cmd::ir_flood_get(enum IrFloodLedStatus &status)
{
    int ret;
    uint16_t data_len = 0;
    uint16_t resp_len;

    ret = init_header(req_buf, OPCODE_GET_FLOOD_LED_STATUS, data_len);
    if (ret) return ret;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret >= 0)
    {
        status = (req_buf[8] == 0x01) ? IR_LED_ON : (req_buf[8] == 0x00) ? IR_LED_OFF : IR_LED_UNKNOWN;
        printf("IR Flood LED is %s.\n", (status == IR_LED_ON) ? "on" : (status == IR_LED_OFF) ? "off" : "unknown");
    }
    else cout << "send IrFloodGet failed" << endl;

    return ret;
}

/**
 * @brief Sets the IR sensor exposure time.
 */
int cmd::ir_exposure_set(uint32_t val)
{
    int ret;
    uint16_t data_len = 2;
    uint16_t resp_len;
    
    ret = init_header(req_buf, CMD_EXP_SET, data_len);
    if (ret) return ret;

    req_buf[8] = val & 0xff;
    req_buf[9] = (val & 0xff00) >> 8;
    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);

    if (ret < 0) cout << "send cmd set_exp failed" << endl;
    return ret;
}

/**
 * @brief Retrieves the current IR sensor exposure time.
 */
int cmd::ir_exposure_get(uint32_t &val)
{
    int ret;
    uint16_t data_len = 0;
    uint16_t resp_len;

    ret = init_header(req_buf, CMD_IR_EXPOSURE_GET, data_len);
    if (ret) return ret;

    ret = send(req_buf, CMD_HEADER_LEN + data_len, resp_buf, &resp_len);
    if (ret >= 0)
    {
        val = resp_buf[10] | (resp_buf[11] << 8);
    }
    else cout << "send ir_exposure_get failed" << endl;

    return ret;
}

/**
 * @brief Retrieves IR sensor gain via I2C register 0x35.
 */
bool cmd::ir_gain_get(void)
{
    vector<string> cmd_r;
    XnControlProcessingData I2C;

    cmd_r.push_back("i2c");
    cmd_r.push_back("read");
    cmd_r.push_back("1");    // 1 indicates Depth/IR sensor
    cmd_r.push_back("0x35"); // IR Gain register
    
    m_I2CReg = read_i2c(cmd_r, I2C);
    return true;
}

/**
 * @brief Sets IR sensor gain via I2C register 0x35.
 */
bool cmd::ir_gain_set(const char* val)
{
    vector<string> cmd_r;
    XnControlProcessingData I2C;

    cmd_r.push_back("i2c");
    cmd_r.push_back("write");
    cmd_r.push_back("1");    // 1 indicates Depth/IR sensor
    cmd_r.push_back("0x35"); // IR Gain register
    cmd_r.push_back(val);

    write_i2c(cmd_r, I2C);
    return true;
}


