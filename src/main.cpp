/**
 * @file main.cpp
 * @brief Entry point for the Orbbec Integration Sample.
 * 
 * This application demonstrates how to:
 * 1. Initialize the Orbbec camera using OpenNI 2.
 * 2. Access low-level hardware features via raw USB (emitter, gain, exposure).
 * 3. Process Depth and IR streams for visualization.
 * 4. Implement a simple vertical-zone gesture analysis for treadmill control.
 */

#include <iostream>
#include <OpenNI.h>
#include <fstream>
#include <string>
#include <stdio.h>

#include "SimpleViewer.h"
#include "cmd.h"
#include <vector>
#include <map>

using namespace std;

/**
 * @brief Global hardware command object.
 * Handles all non-standard OpenNI hardware controls.
 */
class cmd cmd;

/**
 * @brief OpenNI VideoStream objects.
 * These represent the logical data channels from the camera.
 */
openni::VideoStream m_irStream;
openni::VideoStream m_depthStream;
openni::VideoStream m_colorStream;

/**
 * @brief Creates and initializes the Infrared (IR) sensor stream.
 * @param device Reference to the opened OpenNI device.
 * @return true if the stream was successfully created.
 */
bool createIRStream(openni::Device &device)
{
    openni::Status rc = m_irStream.create(device, openni::SENSOR_IR);
    if (rc != STATUS_OK)
    {
        printf("Couldn't find IR stream:\n%s\n", openni::OpenNI::getExtendedError());
        return false;
    }
    return true;
}

/**
 * @brief Creates and initializes the Depth sensor stream.
 */
bool createDepthStream(openni::Device &device)
{
    openni::Status rc = m_depthStream.create(device, openni::SENSOR_DEPTH);
    if (rc != STATUS_OK)
    {
        printf("Couldn't find Depth stream:\n%s\n", openni::OpenNI::getExtendedError());
        return false;
    }
    return true;
}

/**
 * @brief Creates and initializes the RGB Color sensor stream.
 * @note Many Orbbec models (like Astra Pro) handle RGB via standard UVC, 
 *       meaning the RGB sensor won't be visible through OpenNI.
 */
bool createRGBStream(openni::Device &device)
{
    openni::Status rc = m_colorStream.create(device, openni::SENSOR_COLOR);
    if (rc != STATUS_OK)
    {
        printf("Couldn't find RGB stream:\n%s\n", openni::OpenNI::getExtendedError());
        return false;
    }
    return true;
}

/**
 * @brief Application main loop.
 * Initializes hardware, sets up streams, and starts the OpenGL viewer.
 */
int main(int argc, char** argv)
{
    cout << " *********** Orbbec Integrate Simple v1.0 *********" << endl << endl;
    cout << " Keyboard Controls:" << endl;
    cout << "  1/2/3 : Switch between Depth, RGB, and IR streams" << endl;
    cout << "  l/L   : Emitter ON / OFF" << endl;
    cout << "  d/D   : IR Flood LED ON / OFF" << endl;
    cout << "  b/B   : LDP ON / OFF" << endl;
    cout << "  r/R   : Increase / Decrease IR Exposure" << endl;
    cout << "  o/O   : Increase / Decrease IR Gain" << endl;
    cout << "  f/v   : Print Version / Serial Number" << endl;
    cout << "  Esc   : Exit Application" << endl << endl;

   openni::Status rc = openni::STATUS_OK;
   
   // Step 1: Initialize hardware control and identify the device
   if (cmd.init() != 0)
   {
       printf("Hardware initialization failed. Ensure the camera is connected.\n");
       return -1;
   }

    // Step 2: Create required sensor streams
    if (!createDepthStream(cmd.device))
    {
        openni::OpenNI::shutdown();
        return -1;
    }

    if (!createIRStream(cmd.device))
    {
        openni::OpenNI::shutdown();
        return -1;
    }

    // Astra Pro models do not support RGB via OpenNI (they use UVC)
    if(cmd.m_pid != Astra_Pro)
    {
        if (!createRGBStream(cmd.device))
        {
            // Non-Astra Pro devices should have RGB via OpenNI
            printf("Notice: RGB stream not found via OpenNI.\n");
        }
    }
    else
    {
        printf("Astra Pro detected: RGB stream is handled via UVC (OpenNI stream skipped).\n");
    }

    // Step 3: Initialize the visualization engine
    SimpleViewer mSimpleViewer("Orbbec Integrate Simple", cmd.device, m_colorStream, m_irStream, m_depthStream);

    // Link the hardware control object so the viewer can toggle settings via keyboard
    mSimpleViewer.m_cmd = &cmd;

    // Step 4: Start OpenGL and the main processing loop
    rc = mSimpleViewer.init(argc, argv);
    if (rc != openni::STATUS_OK)
    {
        openni::OpenNI::shutdown();
        return -1;
    }

    // The run() method enters the GLUT main loop and never returns until the window is closed
    mSimpleViewer.run();

    return 0;
}
