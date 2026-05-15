#include "SimpleViewer.h"
#include <GL/glut.h>
#include <sstream>

// Resolution and Buffer Constants
#define RES_COLOR_WIDTH    640
#define RES_COLOR_HEIGHT   480
#define GL_WIN_SIZE_X      800
#define GL_WIN_SIZE_Y      600
#define TEXTURE_SIZE       512

// Gesture detection depth range (mm).
// Treadmill user sits approximately 800-1000 mm from the sensor.
// 400 mm near-limit gives margin for arms reaching toward the camera;
// 1200 mm far-limit excludes background while still covering the full body depth.
// CLOSE_DIST separates forearms-pointing-at-camera (~400-650mm) from the torso (~800-1000mm).
#define GESTURE_MIN_DIST   400
#define GESTURE_MAX_DIST   1200
#define GESTURE_CLOSE_DIST 650

/**
 * @brief Helpers to ensure texture dimensions are powers of 2 (required for some OpenGL versions).
 */
#define MIN_NUM_CHUNKS(data_size, chunk_size)   ((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)  (MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))


SimpleViewer* SimpleViewer::m_self = NULL;
string getChangeValueString;
uint32_t m_exposureReg, m_exposureRegSet;

// --- GLUT Callbacks (Static wrappers) ---

void SimpleViewer::glutIdle()
{
    glutPostRedisplay();
}

void SimpleViewer::glutDisplay()
{
    SimpleViewer::m_self->display();
}

void SimpleViewer::glutKeyboard(unsigned char key, int x, int y)
{
    SimpleViewer::m_self->onKey(key, x, y);
}

SimpleViewer::SimpleViewer(const char* strSampleName, openni::Device &device, openni::VideoStream& color, openni::VideoStream& ir, openni::VideoStream& depth) :
m_device(device), m_colorStream(color), m_irStream(ir), m_depthStream(depth), m_pTexMap(NULL), m_streamType(STREAM_TYPE_UNKNOWN), m_prevStreamType(STREAM_TYPE_UNKNOWN)
{
    m_self = this;
    strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
    
    // Pre-allocate BFS buffers for the maximum expected frame size (640x480)
    // to avoid per-frame heap allocation inside analyzeGestures().
    m_userMask.reserve(640 * 480);
    m_bfsBuffer.reserve(640 * 480);

    // Initialize gesture tracking state
    m_framesJump = 0;
    m_framesDuck = 0;
    m_framesWalk = 0;
    m_avgPixelCount = 0.0f;
    m_topHalfCount  = 0;
    m_botHalfCount  = 0;
    m_gestureText = "WAITING";
}


SimpleViewer::~SimpleViewer()
{
    delete[] m_pTexMap;
    m_self = NULL;
}


openni::Status SimpleViewer::init(int argc, char** argv)
{
    // Default mode: Start with the Depth stream
    m_streamType = STREAM_TYPE_DEPTH_VGA;  
    switchStream();

    return initOpenGL(argc, argv);
}

/**
 * @brief Dispatches the current OpenNI frame to the correct conversion function based on stream type.
 */
void SimpleViewer::loadFrameToTexture(VideoFrameRef frame)
{
    // Re-allocate texture buffer if resolution changed
    unsigned int textMapX = m_nTexMapX;
    unsigned int textMapY = m_nTexMapY;
    m_nTexMapX = MIN_CHUNKS_SIZE(m_width, TEXTURE_SIZE);
    m_nTexMapY = MIN_CHUNKS_SIZE(m_height, TEXTURE_SIZE);

    if (textMapX != m_nTexMapX || textMapY != m_nTexMapY)
    {
        delete[] m_pTexMap;
        m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
    }

    memset(m_pTexMap, 0, m_nTexMapX * m_nTexMapY * sizeof(openni::RGB888Pixel));

    if (frame.isValid())
    {
        switch(m_streamType)
        {
        case STREAM_TYPE_IR_VGA:
            loadIRFrame(frame);
            break;
        case STREAM_TYPE_DEPTH_VGA:
            loadDepthFrame(frame);
            analyzeGestures(frame); // Run gesture analysis on every depth frame
            break;
        case STREAM_TYPE_RGB_VGA:
            loadCOLORFrame(frame);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Converts a 16-bit Grayscale IR frame to 8-bit RGB for OpenGL.
 * Shifts bits to map the high-dynamic range IR data into visible intensities.
 */
void SimpleViewer::loadIRFrame(VideoFrameRef frame)
{
    openni::RGB888Pixel* pTexRow = m_pTexMap + frame.getCropOriginY() * m_nTexMapX;
    const openni::Grayscale16Pixel* pImageRow = (const openni::Grayscale16Pixel*)frame.getData();
    int rowSize = frame.getStrideInBytes() / sizeof(openni::Grayscale16Pixel);

    for (int y = 0; y < frame.getHeight(); ++y)
    {
        const openni::Grayscale16Pixel* pImage = pImageRow;
        openni::RGB888Pixel* pTex = pTexRow + frame.getCropOriginX();

        for (int x = 0; x < frame.getWidth(); ++x, ++pImage, ++pTex)
        {
            // Map 16-bit IR (typically 10-bit active) to 8-bit
            uint8_t textureValue = (uint8_t)((*pImage) >> 2);
            pTex->r = textureValue;
            pTex->g = textureValue;
            pTex->b = textureValue;
        }

        pImageRow += rowSize;
        pTexRow += m_nTexMapX;
    }
}

/**
 * @brief Converts a 16-bit Depth frame to 8-bit Grayscale, rendering only
 * pixels within the gesture detection range (800–2000 mm).
 * Everything outside that range is rendered black so the display matches
 * exactly what the gesture algorithm evaluates.
 */
void SimpleViewer::loadDepthFrame(VideoFrameRef frame)
{
    calculateHistogram(m_pDepthHist, MAX_DEPTH, frame);

    const DepthPixel* pDepthRow = (const DepthPixel*)frame.getData();
    RGB888Pixel* pTexRow = m_pTexMap + frame.getCropOriginY() * m_nTexMapX;
    int rowSize = frame.getStrideInBytes() / sizeof(DepthPixel);

    for (int y = 0; y < frame.getHeight(); ++y)
    {
        const DepthPixel* pDepth = pDepthRow;
        RGB888Pixel* pTex = pTexRow + frame.getCropOriginX();

        for (int x = 0; x < frame.getWidth(); ++x, ++pDepth, ++pTex)
        {
            if (*pDepth >= GESTURE_MIN_DIST && *pDepth <= GESTURE_MAX_DIST)
            {
                int nHistValue = (int)m_pDepthHist[*pDepth];
                pTex->r = nHistValue;
                pTex->g = nHistValue;
                pTex->b = nHistValue;
            }
            // Pixels outside the gesture range stay black (zeroed by memset in loadFrameToTexture)
        }

        pDepthRow += rowSize;
        pTexRow += m_nTexMapX;
    }
}

/**
 * @brief Copies standard RGB888 pixel data directly into the texture buffer.
 */
void SimpleViewer::loadCOLORFrame(VideoFrameRef frame)
{
    const RGB888Pixel* pImageRow = (const RGB888Pixel*)frame.getData();
    RGB888Pixel* pTexRow = m_pTexMap + frame.getCropOriginY() * m_nTexMapX;
    int rowSize = frame.getStrideInBytes() / sizeof(RGB888Pixel);

    for (int y = 0; y < frame.getHeight(); ++y)
    {
        const RGB888Pixel* pImage = pImageRow;
        RGB888Pixel* pTex = pTexRow + frame.getCropOriginX();

        for (int x = 0; x < frame.getWidth(); ++x, ++pImage, ++pTex)
        {
            *pTex = *pImage;
        }

        pImageRow += rowSize;
        pTexRow += m_nTexMapX;
    }
}

void SimpleViewer::analyzeGestures(const openni::VideoFrameRef& frame)
{
    if (!frame.isValid() || frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1MM)
        return;

    const int W       = frame.getWidth();
    const int H       = frame.getHeight();
    const DepthPixel* pData   = (const DepthPixel*)frame.getData();
    const int         rowSize = frame.getStrideInBytes() / sizeof(DepthPixel);

    const DepthPixel MIN_DIST = 800;
    const DepthPixel MAX_DIST = 2000;

    // --- Phase 1: Distance filter → binary foreground mask + rough CoM ---
    // The rough CoM is only used as the BFS seed in Phase 2; it includes all
    // foreground pixels (furniture, walls, etc.) and is intentionally imprecise.
    m_userMask.assign(W * H, 0);
    long long roughSumX = 0, roughSumY = 0;
    int roughCount = 0;

    for (int y = 0; y < H; ++y)
    {
        for (int x = 0; x < W; ++x)
        {
            DepthPixel d = pData[y * rowSize + x];
            if (d >= MIN_DIST && d <= MAX_DIST)
            {
                m_userMask[y * W + x] = 1;
                roughSumX += x;
                roughSumY += y;
                roughCount++;
            }
        }
    }

    if (roughCount < 1000)
    {
        m_gestureText = "NO USER";
        return;
    }

    // --- Phase 2: BFS flood fill to isolate the connected user blob ---
    // Seeding from the rough CoM means we always grow the blob that is
    // dominant in the scene — ignoring disconnected background clutter that
    // also happens to fall in the 800-2000mm range.
    int seedX = (int)(roughSumX / roughCount);
    int seedY = (int)(roughSumY / roughCount);

    // The CoM may land on a 0-pixel (e.g. in a concave region). Spiral outward
    // on the border of an expanding square until a foreground pixel is found.
    if (!m_userMask[seedY * W + seedX])
    {
        bool found = false;
        for (int r = 1; r < 30 && !found; ++r)
        {
            for (int dy = -r; dy <= r && !found; ++dy)
            {
                for (int dx = -r; dx <= r && !found; ++dx)
                {
                    if (abs(dx) != r && abs(dy) != r) continue; // only the border ring
                    int nx = seedX + dx, ny = seedY + dy;
                    if (nx >= 0 && nx < W && ny >= 0 && ny < H && m_userMask[ny * W + nx])
                    {
                        seedX = nx; seedY = ny; found = true;
                    }
                }
            }
        }
        if (!found) { m_gestureText = "NO USER"; return; }
    }

    // BFS using m_bfsBuffer as both the queue and the final component list.
    // Processing via a head index avoids the overhead of std::queue.
    m_bfsBuffer.clear();
    int seedIdx = seedY * W + seedX;
    m_userMask[seedIdx] = 2; // 2 = visited
    m_bfsBuffer.push_back(seedIdx);

    static const int dx4[] = {-1,  1,  0, 0};
    static const int dy4[] = { 0,  0, -1, 1};

    for (int head = 0; head < (int)m_bfsBuffer.size(); ++head)
    {
        int idx = m_bfsBuffer[head];
        int px  = idx % W, py = idx / W;
        for (int i = 0; i < 4; ++i)
        {
            int nx = px + dx4[i], ny = py + dy4[i];
            if (nx >= 0 && nx < W && ny >= 0 && ny < H)
            {
                int nidx = ny * W + nx;
                if (m_userMask[nidx] == 1)
                {
                    m_userMask[nidx] = 2;
                    m_bfsBuffer.push_back(nidx);
                }
            }
        }
    }

    m_validPixelCount = (int)m_bfsBuffer.size();
    if (m_validPixelCount < 1000)
    {
        m_gestureText = "NO USER";
        return;
    }

    // --- Phase 3: Bounding box, CoM, and extremity pixels from the user blob ---
    m_minX = W; m_maxX = 0; m_minY = H; m_maxY = 0;
    long long sumX = 0, sumY = 0;

    for (int idx : m_bfsBuffer)
    {
        int px = idx % W, py = idx / W;
        if (px < m_minX) m_minX = px;
        if (px > m_maxX) m_maxX = px;
        if (py < m_minY) m_minY = py;
        if (py > m_maxY) m_maxY = py;
        sumX += px;
        sumY += py;
    }

    m_userCoMX = (float)(sumX / m_validPixelCount);
    m_userCoMY = (float)(sumY / m_validPixelCount);

    int userHeight = m_maxY - m_minY;
    if (userHeight == 0) return;

    // Head  = topmost pixel in the blob (smallest screen Y = highest in real world).
    // Hands = leftmost and rightmost pixels in the upper 60% of the bounding box.
    //         Limiting to the upper body prevents feet/knees from being mistaken for hands.
    int armSearchBottom = m_minY + (int)(userHeight * 0.60f);

    int headX = (int)m_userCoMX, headY = H;
    int leftHandX  = W,  leftHandY  = -1;
    int rightHandX = -1, rightHandY = -1;

    for (int idx : m_bfsBuffer)
    {
        int px = idx % W, py = idx / W;
        if (py < headY) { headY = py; headX = px; }
        if (py <= armSearchBottom)
        {
            if (px < leftHandX)  { leftHandX  = px; leftHandY  = py; }
            if (px > rightHandX) { rightHandX = px; rightHandY = py; }
        }
    }

    if (leftHandY < 0 || rightHandY < 0) return; // degenerate blob

    // --- Phase 4: Convert key points to world coordinates (mm) ---
    // convertDepthToWorld maps (pixelX, pixelY, depthZ) → world(X, Y, Z) where
    // Y increases upward. The head therefore has the largest world Y value.
    auto toWorld = [&](int px, int py, float& wx, float& wy, float& wz)
    {
        DepthPixel d = pData[py * rowSize + px];
        if (d == 0) d = MIN_DIST; // guard against invalid (zero) depth pixels
        openni::CoordinateConverter::convertDepthToWorld(m_depthStream, px, py, d, &wx, &wy, &wz);
    };

    float headWX,  headWY,  headWZ;
    float leftWX,  leftWY,  leftWZ;
    float rightWX, rightWY, rightWZ;
    float comWX,   comWY,   comWZ;

    toWorld(headX,           headY,           headWX,  headWY,  headWZ);
    toWorld(leftHandX,       leftHandY,       leftWX,  leftWY,  leftWZ);
    toWorld(rightHandX,      rightHandY,      rightWX, rightWY, rightWZ);
    toWorld((int)m_userCoMX, (int)m_userCoMY, comWX,   comWY,   comWZ);

    // --- Phase 5: Body-proportional gesture classification ---
    // headToCoM is the vertical span from the head down to the body's CoM (~belly
    // button). This scales with both the user's size and their distance from the
    // camera, making every threshold below naturally distance-invariant.
    float headToCoM = headWY - comWY; // mm, positive (head is above CoM in world Y)
    if (headToCoM <= 0) return;       // degenerate: camera upside-down or bad data

    // Shoulder reference: ~40% of head-to-CoM distance below the head.
    // Arms raised above this → JUMP.
    float shoulderThreshold = headWY - headToCoM * 0.40f;

    // Hip reference: ~140% of head-to-CoM distance below the head (below CoM).
    // Arms lowered below this → DUCK.
    float hipThreshold = headWY - headToCoM * 1.40f;

    float avgHandY = (leftWY + rightWY) * 0.5f;

    if (avgHandY > shoulderThreshold)
    {
        m_framesJump++;
        m_framesDuck = 0;
        m_framesWalk = 0;
    }
    else if (avgHandY < hipThreshold)
    {
        m_framesDuck++;
        m_framesJump = 0;
        m_framesWalk = 0;
    }
    else
    {
        m_framesWalk++;
        m_framesJump = 0;
        m_framesDuck = 0;
    }

    if (m_framesJump >= 5) m_gestureText = "JUMP";
    else if (m_framesDuck >= 5) m_gestureText = "DUCK";
    else if (m_framesWalk >= 5) m_gestureText = "WALKING";
}


openni::VideoStream* SimpleViewer::getStream(int index)
{
    switch(index)
    {
    case STREAM_TYPE_IR_VGA:    return &m_irStream;
    case STREAM_TYPE_RGB_VGA:   return &m_colorStream;
    case STREAM_TYPE_DEPTH_VGA: return &m_depthStream;
    default: return NULL;
    }
}

/**
 * @brief Stops the current stream and starts the requested one.
 * Updates the resolution settings for the sensor.
 */
void SimpleViewer::switchStream()
{
    if (m_streamType == m_prevStreamType) return;
    m_prevStreamType = m_streamType;

    // Stop all streams first
    m_colorStream.stop();
    m_depthStream.stop();
    m_irStream.stop();

    openni::VideoStream* active = getStream(m_streamType);
    if (active && active->isValid())
    {
        VideoMode videomode = active->getVideoMode();
        videomode.setResolution(640, 480);
        
        if (m_streamType == STREAM_TYPE_IR_VGA)
            videomode.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
        else if (m_streamType == STREAM_TYPE_DEPTH_VGA)
            videomode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
        
        active->setVideoMode(videomode);
        active->start();
        m_width = 640;
        m_height = 480;
    }
}


openni::Status SimpleViewer::run()
{
    glutMainLoop(); // Hand control over to GLUT
    return openni::STATUS_OK;
}

/**
 * @brief Main rendering loop.
 * 1. Reads the next frame from the camera.
 * 2. Loads it into an OpenGL texture.
 * 3. Draws the texture as a full-screen quad.
 * 4. Renders the gesture HUD overlay.
 */
void SimpleViewer::display()
{
    switchStream();
    VideoStream* pStream = getStream(m_streamType);

    if (NULL == pStream) return;

    int changedIndex = -1;
    openni::Status rc = openni::OpenNI::waitForAnyStream(&pStream, 1, &changedIndex, 1000);
    if (rc != openni::STATUS_OK) return;

    VideoFrameRef frame;
    pStream->readFrame(&frame); 
    loadFrameToTexture(frame);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

    // Update the OpenGL texture with current frame data
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

    glColor4f(1, 1, 1, 1);
    glBegin(GL_QUADS);
    // Map the 640x480 frame onto the power-of-2 (e.g. 1024x512) texture coordinate space
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    glTexCoord2f((float)m_width / (float)m_nTexMapX, 0);
    glVertex2f(GL_WIN_SIZE_X, 0);
    glTexCoord2f((float)m_width / (float)m_nTexMapX, (float)m_height / (float)m_nTexMapY);
    glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glTexCoord2f(0, (float)m_height / (float)m_nTexMapY);
    glVertex2f(0, GL_WIN_SIZE_Y);
    glEnd();

    renderGestureOverlay();
    glutSwapBuffers();
}

/**
 * @brief Helper to render text at specified screen coordinates.
 */
void SimpleViewer::drawText(int x, int y, const char* text)
{
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    }
}

/**
 * @brief Renders the gesture HUD and live detection values for tuning.
 *
 * Layout (top-left):
 *   ACTION: <gesture>
 *   Bot: <bot>  EMA_bot: <avg_bot>  duck_ratio: <ratio>  [threshold 0.50]
 *   Top: <top>  Bot: <bot>  ratio: <jump_ratio>  [threshold 1.80]
 */
void SimpleViewer::renderGestureOverlay()
{
    glDisable(GL_TEXTURE_2D);

    // --- Background box ---
    glColor4f(0.0f, 0.0f, 0.0f, 0.7f);
    glBegin(GL_QUADS);
    glVertex2i(10, 10);
    glVertex2i(460, 10);
    glVertex2i(460, 100);
    glVertex2i(10, 100);
    glEnd();

    char buf[128];

    // Row 1: gesture label
    glColor3f(0.0f, 1.0f, 0.0f);
    drawText(20, 32, ("ACTION: " + m_gestureText).c_str());

    // Row 2: close-pixel ratio (key WALK signal)
    float closeRatio = (m_validPixelCount > 0)
                       ? (float)m_closePixelCount / (float)m_validPixelCount
                       : 0.0f;
    snprintf(buf, sizeof(buf), "Close: %5d  Total: %5d  close_ratio: %.2f  [walk thresh 0.10]  close<%dmm",
             m_closePixelCount,
             m_validPixelCount,
             closeRatio,
             GESTURE_CLOSE_DIST);
    glColor3f(1.0f, 1.0f, 0.0f);
    drawText(20, 57, buf);

    // Row 3: jump/walk detection values
    float jumpRatio = (m_botHalfCount > 0)
                      ? (float)m_topHalfCount / (float)m_botHalfCount
                      : 0.0f;
    snprintf(buf, sizeof(buf), "Top: %5d  Bot: %5d  jump_ratio: %.2f  [thresh 0.30]",
             m_topHalfCount,
             m_botHalfCount,
             jumpRatio);
    glColor3f(0.4f, 0.8f, 1.0f);
    drawText(20, 82, buf);

    glEnable(GL_TEXTURE_2D);
}

/**
 * @brief Handles keyboard input for hardware control.
 * Dispatches to the m_cmd hardware control object or standard OpenNI properties.
 */
void SimpleViewer::onKey(unsigned char key, int x, int y)
{
    switch (key)
    {
    case '1': m_streamType = STREAM_TYPE_DEPTH_VGA; break;
    case '2': 
        if (m_cmd->m_pid == Astra_Pro) printf("Notice: Astra Pro RGB is UVC only.\n");
        else m_streamType = STREAM_TYPE_RGB_VGA; 
        break;
    case '3': m_streamType = STREAM_TYPE_IR_VGA; break;

    case 'l': m_cmd->emitter_set(true); break;
    case 'L': m_cmd->emitter_set(false); break;
    case 'd': m_cmd->ir_flood_set(true); break; 
    case 'D': m_cmd->ir_flood_set(false); break;
    case 'b': m_cmd->ldp_set(true); break;
    case 'B': m_cmd->ldp_set(false); break;

    case 'r': // Increase IR Exposure
        changeStringValueFlag = true;
        m_cmd->ir_exposure_get(m_exposureReg);
        m_exposureReg = changeExposureValue(256);
        m_cmd->ir_exposure_set(m_exposureReg);
        break;  
    case 'R': // Decrease IR Exposure
        changeStringValueFlag = false;
        m_cmd->ir_exposure_get(m_exposureReg);
        m_exposureReg = changeExposureValue(256);
        m_cmd->ir_exposure_set(m_exposureReg);
        break;

    case 'o': // Increase IR Gain
        changeStringValueFlag = true;
        m_cmd->ir_gain_get();
        getChangeValueString = changeStringValue(m_cmd->m_I2CReg);
        m_cmd->ir_gain_set(getChangeValueString.c_str());
        break;  
    case 'O': // Decrease IR Gain
        changeStringValueFlag = false;
        m_cmd->ir_gain_get();
        getChangeValueString = changeStringValue(m_cmd->m_I2CReg);
        m_cmd->ir_gain_set(getChangeValueString.c_str());
        break;

    case 'f': m_cmd->get_version(); break;
    case 'v': m_cmd->get_sn_number(); break;
    case 'c': m_cmd->get_cmos_params(); break;

    case 27: // ESC key
        m_device.close();
        openni::OpenNI::shutdown();
        exit(0);
    }
}

/**
 * @brief Helper to increment/decrement and format register values as strings.
 */
string SimpleViewer::changeStringValue(unsigned short nReg)
{
    if (changeStringValueFlag) nReg++; else nReg--;
    m_cmd->m_I2CReg = nReg;
    stringstream ss;
    ss << nReg; 
    return ss.str();
}

/**
 * @brief Helper to update the exposure register cache.
 */
uint32_t SimpleViewer::changeExposureValue(int delta)
{
    if (changeStringValueFlag) m_exposureRegSet += delta; else m_exposureRegSet -= delta;
    return m_exposureRegSet;
}

// --- Standard OpenNI Camera Property Toggles ---

void SimpleViewer::toggleImageAutoExposure(int)
{
    if (m_colorStream.getCameraSettings()) {
        bool enabled = m_colorStream.getCameraSettings()->getAutoExposureEnabled();
        m_colorStream.getCameraSettings()->setAutoExposureEnabled(!enabled);
        printf("Auto Exposure: %s\n", !enabled ? "ON" : "OFF");
    }
}

void SimpleViewer::toggleImageAutoWhiteBalance(int)
{
    if (m_colorStream.getCameraSettings()) {
        bool enabled = m_colorStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
        m_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(!enabled);
        printf("Auto White Balance: %s\n", !enabled ? "ON" : "OFF");
    }
}

void SimpleViewer::changeImageExposure(int delta)
{
    if (m_colorStream.getCameraSettings()) {
        int exposure = m_colorStream.getCameraSettings()->getExposure();
        m_colorStream.getCameraSettings()->setExposure(exposure + delta);
        printf("Exposure: %d\n", exposure + delta);
    }
}

void SimpleViewer::changeImageGain(int delta)
{
    if (m_colorStream.getCameraSettings()) {
        int gain = m_colorStream.getCameraSettings()->getGain();
        m_colorStream.getCameraSettings()->setGain(gain + delta);
        printf("Gain: %d\n", gain + delta);
    }
}


openni::Status SimpleViewer::initOpenGL(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow(m_strSampleName);
    glutSetCursor(GLUT_CURSOR_NONE);

    initOpenGLHooks();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    return openni::STATUS_OK;
}


void SimpleViewer::initOpenGLHooks()
{
    glutKeyboardFunc(glutKeyboard);
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);
}

/**
 * @brief Calculates an accumulative histogram of depth values.
 * This maps depth values into a [0, 255] grayscale range where closer 
 * objects are brighter.
 */
void SimpleViewer::calculateHistogram(float* pHistogram, int histogramSize, const VideoFrameRef& frame)
{
    const openni::DepthPixel* pDepth = (const openni::DepthPixel*)frame.getData();
    memset(pHistogram, 0, histogramSize*sizeof(float));
    
    int height = frame.getHeight();
    int width = frame.getWidth();
    unsigned int nNumberOfPoints = 0;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x, ++pDepth)
        {
            if (*pDepth != 0)
            {
                pHistogram[*pDepth]++;
                nNumberOfPoints++;
            }
        }
    }

    for (int i=1; i<histogramSize; i++) pHistogram[i] += pHistogram[i-1];
    
    if (nNumberOfPoints)
    {
        for (int i=1; i<histogramSize; i++)
        {
            pHistogram[i] = (255 * (1.0f - (pHistogram[i] / nNumberOfPoints)));
        }
    }
}
