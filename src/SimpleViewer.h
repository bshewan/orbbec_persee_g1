/**
 * @file SimpleViewer.h
 * @brief Header for the OpenGL-based visualization and gesture analysis engine.
 * 
 * SimpleViewer manages the OpenGL lifecycle (via GLUT), handles keyboard 
 * interaction, converts OpenNI video frames into OpenGL textures, and 
 * performs heuristic-based gesture analysis on depth data.
 */

#ifndef __RGBVIEWER_H__
#define __RGBVIEWER_H__

#include "OpenNI.h"
#include "cmd.h"
#include <iostream>
#include <vector>

#pragma once

#define MAX_DEPTH (10000) ///< Maximum depth value in mm (10 meters)
#define Astra_Pro (0x0403) ///< Product ID for Astra Pro models

class SimpleViewer
{
public:
	/**
	 * @brief Supported camera stream modes for visualization.
	 */
	enum StreamTypes
	{
		STREAM_TYPE_IR_VGA,    ///< Infrared sensor at VGA resolution (640x480)
		STREAM_TYPE_DEPTH_VGA, ///< Depth sensor at VGA resolution (640x480)
		STREAM_TYPE_RGB_VGA,   ///< Color sensor at VGA resolution (640x480)
		STREAM_TYPE_UNKNOWN
	};


	SimpleViewer(const char* strSampleName, openni::Device &device, openni::VideoStream& color,
		openni::VideoStream& ir,openni::VideoStream& depth);

	virtual ~SimpleViewer();

    /**
     * @brief Initializes the viewer state and OpenGL context.
     */
    virtual openni::Status init(int argc, char** argv);

    /**
     * @brief Enters the main GLUT event loop.
     */
    virtual openni::Status run();

    /**
     * @brief Generates a grayscale histogram from depth data for better visualization.
     * Maps the non-linear depth range (800mm-10000mm) into an 8-bit grayscale range.
     */
    void calculateHistogram(float* pHistogram, int histogramSize, const VideoFrameRef& frame);

    // Frame processing methods: Convert OpenNI frames into the m_pTexMap buffer
    void loadFrameToTexture(VideoFrameRef frame);
    void loadIRFrame(VideoFrameRef frame);	
    void loadDepthFrame(VideoFrameRef frame);	
    void loadCOLORFrame(VideoFrameRef frame);	

    /**
     * @brief Detects gestures (Jump, Duck, Walk) using connected-component segmentation
     *        and world-space coordinate analysis.
     *
     * Algorithm:
     * 1. Distance filter: Build a binary mask of pixels in the 800-2000mm user zone.
     * 2. BFS flood fill: Starting from the rough CoM of the mask, isolate the connected
     *    foreground blob belonging to the user, discarding background clutter.
     * 3. Extremity extraction: Locate the head (topmost pixel) and hand candidates
     *    (lateral extremes in the upper 60% of the bounding box).
     * 4. World-space conversion: Use CoordinateConverter::convertDepthToWorld() to get
     *    real-world mm coordinates, making detection distance-invariant.
     * 5. Classification: Compare average hand height to body-proportional thresholds
     *    derived from the head-to-CoM vertical distance.
     * 6. Debouncing: Requires 5 consecutive frames to commit a gesture.
     */
    void analyzeGestures(const openni::VideoFrameRef& frame);

    // Color Camera Settings (Standard OpenNI Properties)
    void toggleImageAutoExposure(int);
    void toggleImageAutoWhiteBalance(int);
    void changeImageExposure(int);
    void changeImageGain(int);
    
    /**
     * @brief Switches the active OpenNI stream and updates the window resolution.
     */
    void switchStream();	
    
    openni::VideoStream* getStream(int index);

    cmd* m_cmd; ///< Pointer to the low-level hardware command interface
	string changeStringValue(unsigned short nReg);
	uint32_t changeExposureValue(int nReg);
	bool changeStringValueFlag;


protected:
    /**
     * @brief Main OpenGL display callback.
     */
    virtual void display();

    /**
     * @brief Keyboard callback for controlling camera settings.
     */
    virtual void onKey(unsigned char key, int x, int y);

    virtual openni::Status initOpenGL(int argc, char** agrv);
    void    initOpenGLHooks();

    // References to OpenNI streams and current frame buffers
    openni::Device&             m_device;
    openni::VideoStream&        m_colorStream;
    openni::VideoFrameRef       m_colorFrame;

	openni::VideoStream&		m_irStream;
	openni::VideoFrameRef       m_irFrame;

	openni::VideoStream&		m_depthStream;
	openni::VideoFrameRef       m_depthFrame;

	openni::VideoStream*		m_stream;		

private:
    static SimpleViewer*        m_self; ///< Singleton-like pointer for GLUT callbacks

    // GLUT Static Callbacks
    static void                 glutIdle();
    static void                 glutDisplay();
    static void                 glutKeyboard(unsigned char key, int x, int y);


    char                        m_strSampleName[ONI_MAX_STR];
    int                         m_width;     ///< Current resolution width
    int                         m_height;    ///< Current resolution height
    unsigned int                m_nTexMapX;  ///< Texture width (must be power of 2)
    unsigned int                m_nTexMapY;  ///< Texture height (must be power of 2)
    openni::RGB888Pixel*        m_pTexMap;   ///< OpenGL texture buffer
    StreamTypes                 m_streamType;
    StreamTypes 	            m_prevStreamType;

    float			m_pDepthHist[MAX_DEPTH]; ///< Pre-calculated depth histogram

   // --- Gesture Tracking Members ---
   float m_userCoMX;        ///< Horizontal Center of Mass of the detected user (pixel space)
   float m_userCoMY;        ///< Vertical Center of Mass of the detected user (pixel space)
   int   m_validPixelCount; ///< Pixel count of the BFS-isolated user blob
   int   m_minX, m_maxX, m_minY, m_maxY; ///< Bounding box of the detected user

   // --- BFS Segmentation Buffers (pre-allocated to avoid per-frame heap churn) ---
   std::vector<uint8_t> m_userMask;  ///< Per-pixel foreground mask: 0=bg, 1=fg, 2=visited
   std::vector<int>     m_bfsBuffer; ///< BFS frontier / result component (pixel indices)

   // --- State Debouncing (Frame counters) ---
   int   m_framesJump;
   int   m_framesDuck;
   int   m_framesWalk;
   int   m_framesDodgeLeft;
   int   m_framesDodgeRight;

   // --- HUD / On-Screen Display ---
   std::string m_gestureText; ///< Current gesture string for the GUI
   void drawText(int x, int y, const char* text);
   void renderGestureOverlay();
};

#endif
