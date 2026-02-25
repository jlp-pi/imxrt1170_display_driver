/*
 * Copyright 2019-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "lvgl_support.h"
#include "lvgl.h"
#if defined(SDK_OS_FREE_RTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif
#include "board.h"
#include "pin_mux.h"

#include "fsl_gpio.h"
#include "fsl_cache.h"

#include "fsl_gt911.h"

#if LV_USE_GPU_NXP_VG_LITE
#include "vg_lite.h"
#include "vglite_support.h"
#endif

#if LV_USE_GPU_NXP_PXP
#include "draw/nxp/pxp/lv_draw_pxp.h"
#endif

#include "py/runtime.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Ratate panel or not. */
#ifndef DEMO_USE_ROTATE
#if LV_USE_GPU_NXP_PXP
#define DEMO_USE_ROTATE 1
#else
#define DEMO_USE_ROTATE 0
#endif
#endif

/* Cache line size. */
#ifndef FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#define FSL_FEATURE_L2CACHE_LINESIZE_BYTE 0
#endif
#ifndef FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#define FSL_FEATURE_L1DCACHE_LINESIZE_BYTE 0
#endif

#if (FSL_FEATURE_L2CACHE_LINESIZE_BYTE > FSL_FEATURE_L1DCACHE_LINESIZE_BYTE)
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L2CACHE_LINESIZE_BYTE
#else
#define DEMO_CACHE_LINE_SIZE FSL_FEATURE_L1DCACHE_LINESIZE_BYTE
#endif

#if (DEMO_CACHE_LINE_SIZE > FRAME_BUFFER_ALIGN)
#define DEMO_FB_ALIGN DEMO_CACHE_LINE_SIZE
#else
#define DEMO_FB_ALIGN FRAME_BUFFER_ALIGN
#endif

#if (LV_ATTRIBUTE_MEM_ALIGN_SIZE > DEMO_FB_ALIGN)
#undef DEMO_FB_ALIGN
#define DEMO_FB_ALIGN LV_ATTRIBUTE_MEM_ALIGN_SIZE
#endif

#define DEMO_FB_SIZE \
    (((DEMO_BUFFER_WIDTH * DEMO_BUFFER_HEIGHT * LCD_FB_BYTE_PER_PIXEL) + DEMO_FB_ALIGN - 1) & ~(DEMO_FB_ALIGN - 1))

#if DEMO_USE_ROTATE
#define LVGL_BUFFER_WIDTH  DEMO_BUFFER_HEIGHT
#define LVGL_BUFFER_HEIGHT DEMO_BUFFER_WIDTH
#else
#define LVGL_BUFFER_WIDTH  DEMO_BUFFER_WIDTH
#define LVGL_BUFFER_HEIGHT DEMO_BUFFER_HEIGHT
#endif

#if __CORTEX_M == 4
#define DEMO_FLUSH_DCACHE() L1CACHE_CleanInvalidateSystemCache()
#else
#define DEMO_FLUSH_DCACHE() SCB_CleanInvalidateDCache()
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DEMO_FlushDisplay(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
static void DEMO_CleanInvalidateCache(lv_display_t * disp);
#endif

static void DEMO_InitTouch(void);

static void DEMO_ReadTouch(lv_indev_t * indev_drv, lv_indev_data_t * data);

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer);

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp);

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode);

static void DEMO_WaitBufferSwitchOff(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define DYNAMIC_FB_ALLOC 1

#if DYNAMIC_FB_ALLOC
MP_REGISTER_ROOT_POINTER(uint8_t *s_frameBuffer_alloc);  // malloc output goes here
uint8_t (*s_frameBuffer)[DEMO_FB_SIZE];  // this holds aligned framebuffer pointer for use

#if DEMO_USE_ROTATE
MP_REGISTER_ROOT_POINTER(uint8_t *s_lvglBuffer_alloc);
uint8_t (*s_lvglBuffer)[DEMO_FB_SIZE];
#endif

#else
SDK_ALIGN(static uint8_t __attribute__((section(".sdram"))) s_frameBuffer[2][DEMO_FB_SIZE], DEMO_FB_ALIGN);
#if DEMO_USE_ROTATE
SDK_ALIGN(static uint8_t __attribute__((section(".sdram"))) s_lvglBuffer[1][DEMO_FB_SIZE], DEMO_FB_ALIGN);
#endif
#endif

#if defined(SDK_OS_FREE_RTOS)
static SemaphoreHandle_t s_transferDone;
#else
static volatile bool s_transferDone;
#endif

#if DEMO_USE_ROTATE
/*
 * When rotate is used, LVGL stack draws in one buffer (s_lvglBuffer), and LCD
 * driver uses two buffers (s_frameBuffer) to remove tearing effect.
 */
static void *volatile s_inactiveFrameBuffer;
#endif

static gt911_handle_t s_touchHandle;
static const gt911_config_t s_touchConfig = {
    .I2C_SendFunc     = BOARD_MIPIPanelTouch_I2C_Send,
    .I2C_ReceiveFunc  = BOARD_MIPIPanelTouch_I2C_Receive,
    .pullResetPinFunc = BOARD_PullMIPIPanelTouchResetPin,
    .intPinFunc       = BOARD_ConfigMIPIPanelTouchIntPin,
    .timeDelayMsFunc  = VIDEO_DelayMs,
    .touchPointNum    = 1,
    .i2cAddrMode      = kGT911_I2cAddrMode0,
    .intTrigMode      = kGT911_IntRisingEdge,
};
static int s_touchResolutionX;
static int s_touchResolutionY;

/*******************************************************************************
 * Code
 ******************************************************************************/

static bool s_lvgl_initialized = false;
static bool s_disp_initialized = false;

void lv_port_pre_init(void) {
}

void lv_port_disp_init(void) {

    /* Guard against multiple initialization */
    if (s_disp_initialized) {
        PRINTF("Display already initialized, skipping\r\n");
        return;
    }

    PRINTF("Display config: %dx%d, %d BPP, FB size: %d bytes\r\n",
           DEMO_BUFFER_WIDTH, DEMO_BUFFER_HEIGHT, DEMO_BUFFER_BYTE_PER_PIXEL, DEMO_FB_SIZE);

    /* Initialize LVGL library */
    if (!s_lvgl_initialized) {
        lv_init();
        s_lvgl_initialized = true;
        PRINTF("LVGL initialized\r\n");
    }

    BOARD_InitMipiPanelPins();

    #if DYNAMIC_FB_ALLOC
    
    #define align_up(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

    MP_STATE_VM(s_frameBuffer_alloc) = m_new0(uint8_t, 2 * DEMO_FB_SIZE + DEMO_FB_ALIGN);
    s_frameBuffer = (uint8_t(*)[DEMO_FB_SIZE]) align_up((uintptr_t)MP_STATE_VM(s_frameBuffer_alloc), DEMO_FB_ALIGN);
    
    #if DEMO_USE_ROTATE
    MP_STATE_VM(s_lvglBuffer_alloc) = m_new0(uint8_t, DEMO_FB_SIZE + DEMO_FB_ALIGN);
    s_lvglBuffer = (uint8_t(*)[DEMO_FB_SIZE]) align_up((uintptr_t)MP_STATE_VM(s_lvglBuffer_alloc), DEMO_FB_ALIGN);
    #endif
    
	#else // static FB alloc
	
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));
    #if DEMO_USE_ROTATE
    memset(s_lvglBuffer, 0, sizeof(s_lvglBuffer));
    #endif
    #endif

    status_t status;
    dc_fb_info_t fbInfo;

    #if LV_USE_GPU_NXP_VG_LITE
    /* Initialize GPU. */
    BOARD_PrepareVGLiteController();
    #endif

    /*-------------------------
     * Initialize your display
     * -----------------------*/
    BOARD_PrepareDisplayController();

    status = g_dc.ops->init(&g_dc);
    if (kStatus_Success != status) {
        assert(0);
    }

    g_dc.ops->getLayerDefaultConfig(&g_dc, 0, &fbInfo);
    fbInfo.pixelFormat = DEMO_BUFFER_PIXEL_FORMAT;
    fbInfo.width = DEMO_BUFFER_WIDTH;
    fbInfo.height = DEMO_BUFFER_HEIGHT;
    fbInfo.startX = DEMO_BUFFER_START_X;
    fbInfo.startY = DEMO_BUFFER_START_Y;
    fbInfo.strideBytes = DEMO_BUFFER_STRIDE_BYTE;
    g_dc.ops->setLayerConfig(&g_dc, 0, &fbInfo);

    g_dc.ops->setCallback(&g_dc, 0, DEMO_BufferSwitchOffCallback, NULL);

    #if defined(SDK_OS_FREE_RTOS)
    s_transferDone = xSemaphoreCreateBinary();
    if (NULL == s_transferDone) {
        PRINTF("Frame semaphore create failed\r\n");
        assert(0);
    }
    #else
    s_transferDone = false;
    #endif

    #if DEMO_USE_ROTATE
    /* s_frameBuffer[1] is first shown in the panel, s_frameBuffer[0] is inactive. */
    s_inactiveFrameBuffer = (void *)s_frameBuffer[0];
    #endif

    /* lvgl starts render in frame buffer 0, so show frame buffer 1 first. */
    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)s_frameBuffer[1]);

    /* Wait for frame buffer sent to display controller video memory. */
    if ((g_dc.ops->getProperty(&g_dc) & kDC_FB_ReserveFrameBuffer) == 0) {
        DEMO_WaitBufferSwitchOff();
    }

    g_dc.ops->enableLayer(&g_dc, 0);

    /* Start the LCD panel (send Display On command for ILI9881C) */
    BOARD_StartLcdPanel();

    /*-----------------------------------
     * Register the display in LittlevGL
     *----------------------------------*/

    // Changes in master (v9 development) https://github.com/lvgl/lvgl/issues/4011

    #if DEMO_USE_ROTATE
    /* Tell LVGL the display is landscape (LVGL_BUFFER_WIDTH x LVGL_BUFFER_HEIGHT = 1200x720).
     * Do NOT call lv_display_set_rotation(): in LVGL v9 that causes LVGL to internally
     * rotate the rendered output to physical portrait layout in the buffer (stride = 720*2 =
     * 1440), while our C flush callback expects landscape layout (stride = 1200*2 = 2400).
     * Applying both rotations produces garbled banded output.
     * With no set_rotation(), LVGL renders landscape directly; the flush callback rotates
     * once (270°) to portrait for the display controller. */
    lv_display_t * disp = lv_display_create(LVGL_BUFFER_WIDTH, LVGL_BUFFER_HEIGHT);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, (void *)DEMO_FlushDisplay);
    lv_display_set_buffers(disp, s_lvglBuffer[0], NULL, DEMO_BUFFER_WIDTH*DEMO_BUFFER_HEIGHT*DEMO_BUFFER_BYTE_PER_PIXEL, LCD_RENDER_MODE);
    #else
    lv_display_t * disp = lv_display_create(LCD_WIDTH, LCD_HEIGHT);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, (void *)DEMO_FlushDisplay);
    /* No set_rotation: LVGL renders portrait (720x1200) directly into s_frameBuffer.
     * Use FULL render mode (not DIRECT) to avoid double-buffer dirty-area sync complexity.
     * Two buffers kept for tearing-free display switching. */
    PRINTF("s_frameBuffer[0]=%p s_frameBuffer[1]=%p\r\n", s_frameBuffer[0], s_frameBuffer[1]);
    /* Single buffer: LVGL does ONE flush per lv_refr_now(), avoiding the
     * double-flush deadlock where the 2nd xSemaphoreTake never gets a Give.
     * s_frameBuffer[1] is still shown initially by lv_port_pre_init(). */
    lv_display_set_buffers(disp, s_frameBuffer[0], NULL, DEMO_BUFFER_WIDTH*DEMO_BUFFER_HEIGHT*DEMO_BUFFER_BYTE_PER_PIXEL, LV_DISPLAY_RENDER_MODE_FULL);
    #endif

#if LV_USE_GPU_NXP_VG_LITE
    if (vg_lite_init(DEFAULT_VG_LITE_TW_WIDTH, DEFAULT_VG_LITE_TW_HEIGHT) != VG_LITE_SUCCESS)
    {
        PRINTF("VGLite init error. STOP.");
        vg_lite_close();
        while (1)
            ;
    }

    if (vg_lite_set_command_buffer_size(VG_LITE_COMMAND_BUFFER_SIZE) != VG_LITE_SUCCESS)
    {
        PRINTF("VGLite set command buffer. STOP.");
        vg_lite_close();
        while (1)
            ;
    }
#endif

    s_disp_initialized = true;
    PRINTF("Display initialization complete\r\n");
}

void lv_port_disp_deinit(void) {
    BOARD_DeinitLcdPanel();
    s_disp_initialized = false;
}

static void DEMO_BufferSwitchOffCallback(void *param, void *switchOffBuffer) {
    #if defined(SDK_OS_FREE_RTOS)
    BaseType_t taskAwake = pdFALSE;

    xSemaphoreGiveFromISR(s_transferDone, &taskAwake);
    portYIELD_FROM_ISR(taskAwake);
    #else
    s_transferDone = true;
    #endif

    #if DEMO_USE_ROTATE
    s_inactiveFrameBuffer = switchOffBuffer;
    #endif
}

#if (LV_USE_GPU_NXP_VG_LITE || LV_USE_GPU_NXP_PXP)
// static void DEMO_CleanInvalidateCache(lv_disp_drv_t *disp_drv) {
    // DEMO_FLUSH_DCACHE();
// }
#endif

static void DEMO_WaitBufferSwitchOff(void) {
    #if defined(SDK_OS_FREE_RTOS)
    if (xSemaphoreTake(s_transferDone, portMAX_DELAY) != pdTRUE) {
        PRINTF("Display flush failed\r\n");
        assert(0);
    }
    #else
    while (false == s_transferDone) {
    }
    s_transferDone = false;
    #endif
}

void DEMO_FlushDisplay(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * color_p) {

    bool is_last = lv_disp_flush_is_last(disp_drv);

    if (!is_last) {
        lv_disp_flush_ready(disp_drv);
        return;
    }
    #if DEMO_USE_ROTATE

    /*
     * Work flow:
     *
     * 1. Wait for the available inactive frame buffer to draw.
     * 2. Draw the ratated frame to inactive buffer.
     * 3. Pass inactive to LCD controller to show.
     */

    static bool firstFlush = true;
    static int rotateFlushCount = 0;

    /* Only wait for the first time. */
    if (firstFlush) {
        firstFlush = false;
        uint32_t disp_para = LCDIFV2->DISP_PARA;
        PRINTF("DBG ROTATE: DISP_PARA=0x%08X LINE_PATTERN=%u (0=RGB,1=RBG,2=GBR,3=GRB,4=BRG,5=BGR)\r\n",
               disp_para, (unsigned)((disp_para >> 26) & 0x7U));
    } else {
        /* Wait frame buffer. */
        DEMO_WaitBufferSwitchOff();
    }

    /* Copy buffer. */
    void *inactiveFrameBuffer = s_inactiveFrameBuffer;

    #if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)s_inactiveFrameBuffer, DEMO_FB_SIZE);
    #else
    SCB_CleanInvalidateDCache_by_Addr(inactiveFrameBuffer, DEMO_FB_SIZE);
    #endif

    lv_color_t * dest_buf = ((lv_color_t *)inactiveFrameBuffer);

    int32_t w = LVGL_BUFFER_WIDTH; //lv_area_get_width(area);
    int32_t h = LVGL_BUFFER_HEIGHT; //lv_area_get_height(area);
    lv_color_format_t cf = lv_display_get_color_format(disp_drv);
    /* lv_draw_sw_rotate has no case for LV_COLOR_FORMAT_RGB565_SWAPPED — it falls
     * through to default: break and silently does nothing.  Both variants are 2 bytes/pixel
     * so pass plain RGB565; the byte-swap is preserved unchanged through the pixel copy. */
    lv_color_format_t rotate_cf = (cf == LV_COLOR_FORMAT_RGB565_SWAPPED) ? LV_COLOR_FORMAT_RGB565 : cf;
    uint32_t w_stride = lv_draw_buf_width_to_stride(w, rotate_cf);
    uint32_t h_stride = lv_draw_buf_width_to_stride(h, rotate_cf);

    lv_display_rotation_t rotation = LV_DISPLAY_ROTATION_270;

    uint32_t dest_stride = (rotation == LV_DISPLAY_ROTATION_270 || rotation == LV_DISPLAY_ROTATION_90) ? h_stride : w_stride;

    /* Invalidate CPU cache for the source LVGL buffer before SW rotate reads it.
     * PXP writes to SDRAM directly (bypassing CPU cache); if SW rotate reads
     * stale CPU cache from a previous render, it would copy wrong pixel values.
     * This flush+invalidate ensures SW rotate always sees PXP-written SDRAM data. */
#if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)color_p, DEMO_FB_SIZE);
#else
    SCB_CleanInvalidateDCache_by_Addr(color_p, DEMO_FB_SIZE);
#endif

    /* Sample source pixels for diagnostic: check a pixel in the RED rectangle area.
     * LVGL landscape (1200x720): RED rect is centered, ~w=200 h=60, at (500,330) to (700,390).
     * Sample at landscape (600, 360) = center. */
    uint16_t *src_px = (uint16_t *)color_p;
    uint16_t sample = src_px[360 * 1200 + 600];
    PRINTF("ROTATE FLUSH#%d: color_p=%p src[360*1200+600]=0x%04X inactive=%p\r\n",
           rotateFlushCount, color_p, sample, inactiveFrameBuffer);
    rotateFlushCount++;

    /* SW rotation: diagnostic step to bypass PXP rotation and isolate colour issue.
     * If colours are correct here, PXP rotation was byte-swapping; if still wrong,
     * the issue is in PXP fill (need to disable LV_USE_DRAW_PXP). */
    lv_draw_sw_rotate(color_p, dest_buf, w, h, w_stride, dest_stride, rotation, rotate_cf);


#if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)s_inactiveFrameBuffer, DEMO_FB_SIZE);
#else
    SCB_CleanInvalidateDCache_by_Addr(inactiveFrameBuffer, DEMO_FB_SIZE);
#endif

    g_dc.ops->setFrameBuffer(&g_dc, 0, inactiveFrameBuffer);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);

    #else /* DEMO_USE_ROTATE */

    static int flushCount = 0;

    if (flushCount == 0) {
        uint32_t disp_para = LCDIFV2->DISP_PARA;
        PRINTF("DBG: DISP_PARA=0x%08X LINE_PATTERN=%u (0=RGB,1=RBG,2=GBR,3=GRB,4=BRG,5=BGR)\r\n",
               disp_para, (unsigned)((disp_para >> 26) & 0x7U));
    }

    /* Alternate between Y-gradient and X-gradient on successive test() calls.
     * Y-GRADIENT (even flushes): confirms whether buffer Y maps to display Y or X.
     *   Top 1/3 (y<400):  0xF800 (red in RGB565 / blue in BGR565)
     *   Mid 1/3 (y<800):  0x07E0 (green — same in RGB565 and BGR565)
     *   Bot 1/3 (y>=800): 0x001F (blue in RGB565 / red in BGR565)
     * X-GRADIENT (odd flushes): confirms whether buffer X maps to display X or Y.
     *   Left 1/3 (x<240): 0xF800
     *   Mid 1/3 (x<480):  0x07E0
     *   Rgt 1/3 (x>=480): 0x001F
     * Expected if portrait-correct: Y-grad → 3 horizontal bands; X-grad → 3 vertical bands.
     * Expected if 90° rotated:      Y-grad → 3 vertical bands;   X-grad → 3 horizontal bands. */
    uint16_t *px = (uint16_t *)color_p;
    if ((flushCount & 1) == 0) {
        /* Y-gradient */
        for (int y = 0; y < 1200; y++) {
            uint16_t c = (y < 400) ? 0xF800 : (y < 800) ? 0x07E0 : 0x001F;
            for (int x = 0; x < 720; x++)
                px[x + y*720] = c;
        }
    } else {
        /* X-gradient */
        for (int y = 0; y < 1200; y++) {
            for (int x = 0; x < 720; x++) {
                uint16_t c = (x < 240) ? 0xF800 : (x < 480) ? 0x07E0 : 0x001F;
                px[x + y*720] = c;
            }
        }
    }

    PRINTF("FLUSH#%d PATTERN=%s area=(%d,%d,%d,%d) color_p=%p\r\n",
           flushCount, ((flushCount & 1) == 0) ? "Y-GRAD" : "X-GRAD",
           area->x1, area->y1, area->x2, area->y2, color_p);
    if ((flushCount & 1) == 0) {
        PRINTF("  Y-GRAD: @(360,200)=%04x(exp F800) @(360,600)=%04x(exp 07E0) @(360,1000)=%04x(exp 001F)\r\n",
               px[360+200*720], px[360+600*720], px[360+1000*720]);
    } else {
        PRINTF("  X-GRAD: @(120,600)=%04x(exp F800) @(360,600)=%04x(exp 07E0) @(600,600)=%04x(exp 001F)\r\n",
               px[120+600*720], px[360+600*720], px[600+600*720]);
    }

#if __CORTEX_M == 4
    L1CACHE_CleanInvalidateSystemCacheByRange((uint32_t)color_p, DEMO_FB_SIZE);
#else
    SCB_CleanInvalidateDCache_by_Addr(color_p, DEMO_FB_SIZE);
#endif

    g_dc.ops->setFrameBuffer(&g_dc, 0, (void *)color_p);
    /* Wait for the LCDIFV2 vsync ISR to confirm the buffer switch.
     * Safe with single-buffer mode: LVGL calls flush exactly once per
     * lv_refr_now(), so WaitBufferSwitchOff takes exactly one semaphore
     * give and returns — no deadlock.  Without this wait the unreceived
     * portYIELD_FROM_ISR in DEMO_BufferSwitchOffCallback disrupts the
     * FreeRTOS scheduler and causes subsequent time.sleep() calls to hang. */
    DEMO_WaitBufferSwitchOff();

    PRINTF("  setFrameBuffer done (flush#%d)\r\n", flushCount);
    flushCount++;

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
    #endif /* DEMO_USE_ROTATE */
}

void lv_port_indev_init(void) {
	BOARD_MIPIPanelTouch_I2C_Init();
    // static lv_indev_drv_t indev_drv;

    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad */
    DEMO_InitTouch();

    /*Register a touchpad input device*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, DEMO_ReadTouch);
}

static void BOARD_PullMIPIPanelTouchResetPin(bool pullUp)
{
    #ifdef BOARD_MIPI_PANEL_TOUCH_RST_GPIO
    if (pullUp)
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 1);
    }
    else
    {
        GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, 0);
    }
    #endif
}

static void BOARD_ConfigMIPIPanelTouchIntPin(gt911_int_pin_mode_t mode)
{
    #ifdef BOARD_MIPI_PANEL_TOUCH_INT_GPIO
    if (mode == kGT911_IntPinInput)
    {
        BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR &= ~(1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
    }
    else
    {
        if (mode == kGT911_IntPinPullDown)
        {
            GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 0);
        }
        else
        {
            GPIO_PinWrite(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, 1);
        }

        BOARD_MIPI_PANEL_TOUCH_INT_GPIO->GDIR |= (1UL << BOARD_MIPI_PANEL_TOUCH_INT_PIN);
    }
    #endif
}

/*Initialize your touchpad*/
static void DEMO_InitTouch(void)
{
    status_t status;

    const gpio_pin_config_t resetPinConfig = {
        .direction = kGPIO_DigitalOutput, .outputLogic = 0, .interruptMode = kGPIO_NoIntmode
    };
    #ifdef BOARD_MIPI_PANEL_TOUCH_INT_GPIO
    GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_INT_GPIO, BOARD_MIPI_PANEL_TOUCH_INT_PIN, &resetPinConfig);
    #endif
    #ifdef BOARD_MIPI_PANEL_TOUCH_RST_GPIO
    GPIO_PinInit(BOARD_MIPI_PANEL_TOUCH_RST_GPIO, BOARD_MIPI_PANEL_TOUCH_RST_PIN, &resetPinConfig);
    #endif

    status = GT911_Init(&s_touchHandle, &s_touchConfig);

    if (kStatus_Success != status)
    {
        PRINTF("ERROR: Touch IC initialization failed\r\n");
        return;
    }

    GT911_GetResolution(&s_touchHandle, &s_touchResolutionX, &s_touchResolutionY);
}

/* Will be called by the library to read the touchpad */
static void DEMO_ReadTouch(lv_indev_t * drv, lv_indev_data_t * data) {
    static int touch_x = 0;
    static int touch_y = 0;

    if (kStatus_Success == GT911_GetSingleTouch(&s_touchHandle, &touch_x, &touch_y))
    {
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Set the last pressed coordinates*/
    if (DEMO_PANEL_WIDTH != s_touchResolutionX) {
        touch_x = touch_x * DEMO_PANEL_WIDTH / s_touchResolutionX;
    }
    data->point.x = s_touchResolutionX - 1 - touch_x;
    
    if (DEMO_PANEL_HEIGHT != s_touchResolutionY) {
        touch_y = touch_y * DEMO_PANEL_HEIGHT / s_touchResolutionY;
    }
    data->point.y = s_touchResolutionY - 1 - touch_y;
}
