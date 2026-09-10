#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>
#include "arm_math.h"

#define ADC_SAMPLE_CYCLES_NORMAL ADC_SAMPLETIME_15CYCLES
#define ADC_SAMPLE_CYCLES_FAST ADC_SAMPLETIME_3CYCLES

#define SWAP16(c) ((uint16_t)(((c) >> 8) | (((c) & 0xFF) << 8)))

#define RAW_ADC_MODE 0

DMA_HandleTypeDef hdmaSpi1Tx;
SPI_HandleTypeDef *hspi1Handle;

extern "C" void DMA2_Stream3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdmaSpi1Tx); }

void spiDmaSetup() {
    hspi1Handle = SPI.getHandle();

    __HAL_RCC_DMA2_CLK_ENABLE();

    hdmaSpi1Tx.Instance = DMA2_Stream3;
    hdmaSpi1Tx.Init.Channel = DMA_CHANNEL_3;
    hdmaSpi1Tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdmaSpi1Tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdmaSpi1Tx.Init.MemInc = DMA_MINC_ENABLE;
    hdmaSpi1Tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmaSpi1Tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdmaSpi1Tx.Init.Mode = DMA_NORMAL;
    hdmaSpi1Tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdmaSpi1Tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdmaSpi1Tx);
    __HAL_LINKDMA(hspi1Handle, hdmatx, hdmaSpi1Tx);

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

inline void spiDmaWait() {
    while (HAL_SPI_GetState(hspi1Handle) != HAL_SPI_STATE_READY) {}
}

volatile bool needsRedraw = true;

void requestRedraw() { needsRedraw = true; }

#define ADC_CAPTURE_SAMPLES 30000u
#define ADC_MIDPOINT 2048

#define ADC_RAIL_LOW_COUNTS 24
#define ADC_RAIL_HIGH_COUNTS 4071

volatile bool waveformClipped = false;

#define SCREEN_W 320
#define SCREEN_H 240

#define ARROW_MARGIN 16
#define PLOT_X ARROW_MARGIN
#define PLOT_Y 20
#define PLOT_W (SCREEN_W - 2 * ARROW_MARGIN)
#define PLOT_H 191

#define PLOT_ROW_BYTES (PLOT_W / 8)

#define GRID_COLS 10
#define GRID_ROWS 8

inline int16_t clampOffset(int32_t raw);
void updateTriggerArrow();
void markSettingsDirty();
extern volatile bool paused;
extern volatile bool pauseRequested;
void drawScrollbar();
extern char voltsPerDivLabel[16];

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdmaAdc1;
uint16_t adcBuf[ADC_CAPTURE_SAMPLES];
volatile uint32_t triggerIdx = 0;
volatile bool triggerArmed = false;

volatile uint32_t viewAnchorIdx = 0;
volatile int32_t scrollOffsetSamples = 0;
#define SCROLL_STEP_SAMPLES 200

enum CaptureMode { CAPTURE_TRIGGERED, CAPTURE_ROLL };
volatile CaptureMode captureMode = CAPTURE_TRIGGERED;
volatile int16_t rollSamples[PLOT_W];
volatile int rollWriteIdx = 0;

enum TriggerMode { TRIG_NORMAL, TRIG_AUTO, TRIG_SINGLE };
volatile TriggerMode triggerMode = TRIG_AUTO;
volatile uint32_t lastTriggerMs = 0;
#define AUTO_TRIGGER_TIMEOUT_MS 60

extern "C" void DMA2_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdmaAdc1); }

extern "C" void ADC_IRQHandler(void) { HAL_ADC_IRQHandler(&hadc1); }

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *h) {
    if (h != &hadc1) return;
    if (captureMode != CAPTURE_ROLL) return;

    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    if (raw <= ADC_RAIL_LOW_COUNTS || raw >= ADC_RAIL_HIGH_COUNTS) waveformClipped = true;
    rollSamples[rollWriteIdx] = clampOffset(raw);
    rollWriteIdx = (rollWriteIdx + 1) % PLOT_W;
    HAL_ADC_Start_IT(&hadc1);
    requestRedraw();
}

void adcInit() {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;

    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef channelConfig = {0};
    channelConfig.Channel = ADC_CHANNEL_0;
    channelConfig.Rank = 1;
    channelConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &channelConfig);
}

void adcDmaInit() {
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdmaAdc1.Instance = DMA2_Stream4;
    hdmaAdc1.Init.Channel = DMA_CHANNEL_0;
    hdmaAdc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdmaAdc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdmaAdc1.Init.MemInc = DMA_MINC_ENABLE;
    hdmaAdc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdmaAdc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdmaAdc1.Init.Mode = DMA_CIRCULAR;
    hdmaAdc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdmaAdc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdmaAdc1);
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdmaAdc1);

    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void adcItInit() {
    HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
}

void adcReconfigure(FunctionalState continuousMode, uint32_t extTrigEdge, uint32_t extTrigConv) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_IT(&hadc1);
    hadc1.Init.ContinuousConvMode = continuousMode;
    hadc1.Init.ExternalTrigConvEdge = extTrigEdge;
    hadc1.Init.ExternalTrigConv = extTrigConv;
    HAL_ADC_Init(&hadc1);
}

enum AdcSpeedMode { ADC_SPEED_NORMAL, ADC_SPEED_FAST };
volatile AdcSpeedMode adcSpeedMode = ADC_SPEED_NORMAL;

void adcSetSpeedMode(AdcSpeedMode mode) {
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_IT(&hadc1);

    ADC_ChannelConfTypeDef channelConfig = {0};
    channelConfig.Channel = ADC_CHANNEL_0;
    channelConfig.Rank = 1;
    channelConfig.SamplingTime = (mode == ADC_SPEED_FAST) ? ADC_SAMPLE_CYCLES_FAST : ADC_SAMPLE_CYCLES_NORMAL;
    HAL_ADC_ConfigChannel(&hadc1, &channelConfig);

    adcSpeedMode = mode;
}

uint32_t apb1TimerClockHz() {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    return (ppre1 & 0x4u) ? (pclk1 * 2u) : pclk1;
}

TIM_HandleTypeDef htim2;

void triggerTimerInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);

    TIM_SlaveConfigTypeDef slaveConfig = {0};
    slaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    slaveConfig.InputTrigger = TIM_TS_TI2FP2;
    slaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
    slaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    slaveConfig.TriggerFilter = 8;
    HAL_TIM_SlaveConfigSynchro(&htim2, &slaveConfig);

    TIM_IC_InitTypeDef icFalling = {0};
    icFalling.ICPolarity = TIM_ICPOLARITY_FALLING;
    icFalling.ICSelection = TIM_ICSELECTION_DIRECTTI;
    icFalling.ICPrescaler = TIM_ICPSC_DIV1;
    icFalling.ICFilter = 8;
    HAL_TIM_IC_ConfigChannel(&htim2, &icFalling, TIM_CHANNEL_2);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

    TIM_IC_InitTypeDef icRising = {0};
    icRising.ICPolarity = TIM_ICPOLARITY_RISING;
    icRising.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    icRising.ICPrescaler = TIM_ICPSC_DIV1;
    icRising.ICFilter = 8;
    HAL_TIM_IC_ConfigChannel(&htim2, &icRising, TIM_CHANNEL_1);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

    HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_TIM_Base_Start_IT(&htim2);
}

volatile uint32_t measuredFreqMilliHz = 0;
volatile uint32_t measuredDutyPermille = 0;
volatile bool measurementValid = false;
volatile bool measurementPending = false;
volatile uint32_t rawPeriodTicks = 0;
volatile uint32_t rawRiseTicks = 0;

extern "C" void TIM2_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
        uint32_t writeIdx = ADC_CAPTURE_SAMPLES - __HAL_DMA_GET_COUNTER(&hdmaAdc1);
        triggerIdx = writeIdx % ADC_CAPTURE_SAMPLES;
        triggerArmed = true;
        lastTriggerMs = millis();

        bool realEdge = __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET;
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);

        if (realEdge) {
            rawPeriodTicks = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
            rawRiseTicks = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
            measurementPending = true;
        } else { measurementValid = false; }
    }
}

void updateFreqMeasurement() {
    if (!measurementPending) return;
    measurementPending = false;

    uint32_t periodTicks = rawPeriodTicks;
    uint32_t riseTicks = rawRiseTicks;
    if (periodTicks > 0 && riseTicks < periodTicks) {
        uint32_t clk = apb1TimerClockHz();
        measuredFreqMilliHz = (uint32_t)(((uint64_t)clk * 1000ull) / periodTicks);
        uint32_t highTicks = periodTicks - riseTicks;
        measuredDutyPermille = (uint32_t)(((uint64_t)highTicks * 1000ull) / periodTicks);
        measurementValid = true;
    }
}

void forceTrigger() { htim2.Instance->EGR = TIM_EGR_UG; }

TIM_HandleTypeDef htim3;

void rollTimerInit() {
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (apb1TimerClockHz() / 1000000u) - 1u;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim3);

    TIM_MasterConfigTypeDef masterConfig = {0};
    masterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &masterConfig);

}

void rollTimerSetPeriod(uint32_t divTimeMs) {
    uint32_t periodUs = ((uint64_t)divTimeMs * 1000ull * GRID_COLS) / PLOT_W;
    if (periodUs < 1) periodUs = 1;

    HAL_TIM_Base_Stop(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, periodUs - 1);
    HAL_TIM_Base_Start(&htim3);
}

TIM_HandleTypeDef htim1;
volatile uint16_t triggerThresholdCounts = 2048;

void triggerThresholdPwmInit() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4095;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef ocConfig = {0};
    ocConfig.OCMode = TIM_OCMODE_PWM1;
    ocConfig.Pulse = triggerThresholdCounts;
    ocConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim1, &ocConfig, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void setTriggerThreshold(uint16_t counts) {
    if (counts > 4095) counts = 4095;
    triggerThresholdCounts = counts;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, counts);
    updateTriggerArrow();
    markSettingsDirty();
}

TFT_eSPI tft = TFT_eSPI();

uint8_t traceBuf[PLOT_ROW_BYTES * PLOT_H];
uint16_t lineBuf[2][PLOT_W];
int16_t waveformSamples[PLOT_W];
int16_t waveformMin[PLOT_W];
int16_t waveformMax[PLOT_W];

#define FFT_SIZE 2048
arm_rfft_instance_q15 fftInst;
q15_t fftSrc[FFT_SIZE];
q15_t fftDst[FFT_SIZE * 2];
q15_t fftMag[FFT_SIZE / 2];
volatile bool spectrumMode = false;

void fftInit() { arm_rfft_init_q15(&fftInst, FFT_SIZE, 0, 1); }

const uint16_t COLOR_BG = TFT_BLACK;
const uint16_t COLOR_GRID = 0x2104;
const uint16_t COLOR_TRACE = TFT_YELLOW;
const uint16_t COLOR_SPECTRUM = TFT_GOLD;
const uint16_t COLOR_UI_TEXT = TFT_WHITE;
const uint16_t COLOR_BORDER = 0x6B6D;
const uint16_t COLOR_BORDER_PAUSED = TFT_RED;

inline void setPixel(uint8_t *buf, int x, int y, bool on) {
    if (x < 0 || x >= PLOT_W || y < 0 || y >= PLOT_H) return;
    int byteIdx = y * PLOT_ROW_BYTES + (x >> 3);
    uint8_t mask = 0x80 >> (x & 7);
    if (on) buf[byteIdx] |= mask;
    else buf[byteIdx] &= ~mask;
}

inline bool getPixel(const uint8_t *buf, int x, int y) {
    if (x < 0 || x >= PLOT_W || y < 0 || y >= PLOT_H) return false;
    int byteIdx = y * PLOT_ROW_BYTES + (x >> 3);
    uint8_t mask = 0x80 >> (x & 7);
    return (buf[byteIdx] & mask) != 0;
}

void clearPlotBuffer() { memset(traceBuf, 0, sizeof(traceBuf)); }

void drawLine(float x0, float y0, float x1, float y1) {
    bool steep = fabsf(y1 - y0) > fabsf(x1 - x0);
    if (steep) { float t = x0; x0 = y0; y0 = t; t = x1; x1 = y1; y1 = t; }
    if (x0 > x1) { float t = x0; x0 = x1; x1 = t; t = y0; y0 = y1; y1 = t; }

    float dx = x1 - x0;
    float dy = y1 - y0;
    float gradient = (dx == 0.0f) ? 1.0f : dy / dx;

    float y = y0;
    int xStart = (int)roundf(x0);
    int xEnd = (int)roundf(x1);
    for (int x = xStart; x <= xEnd; x++) {
        int yi = (int)roundf(y);
        if (steep) setPixel(traceBuf, yi, x, true);
        else setPixel(traceBuf, x, yi, true);
        y += gradient;
    }
}

void drawWaveformIntoBuffer(const int16_t *samples, int count) {
    int prevX = 0, prevY = PLOT_H / 2 - samples[0];
    for (int x = 1; x < count; x++) {
        int y = PLOT_H / 2 - samples[x];
        drawLine((float)prevX, (float)prevY, (float)x, (float)y);
        prevX = x;
        prevY = y;
    }
}

#define TRACE_SMOOTH_RADIUS 3

int16_t smoothTmpS[PLOT_W];

void smoothWaveformSpatial(int16_t *samples, int count) {
    for (int i = 0; i < count; i++) {
        int32_t sumS = 0, n = 0;
        for (int k = -TRACE_SMOOTH_RADIUS; k <= TRACE_SMOOTH_RADIUS; k++) {
            int idx = i + k;
            if (idx < 0 || idx >= count) continue;
            sumS += samples[idx];
            n++;
        }
        smoothTmpS[i] = (int16_t)(sumS / n);
    }
    memcpy(samples, smoothTmpS, count * sizeof(int16_t));
}

#define ATTEN_R_TOP_K 470
#define ATTEN_R_BOTTOM_K 68
#define PGA_GAIN_ASSUMED_X10 50
#define WAVEFORM_SCALE_NUM ((ATTEN_R_TOP_K + ATTEN_R_BOTTOM_K) * 10)
#define WAVEFORM_SCALE_DEN (ATTEN_R_BOTTOM_K * PGA_GAIN_ASSUMED_X10)

volatile int16_t zeroCalCounts = 0;

volatile bool cursorEnabled = false;
volatile int16_t cursorPixelY = PLOT_H / 2;
#define CURSOR_STEP_PX 2
const uint16_t COLOR_CURSOR = TFT_MAGENTA;

inline int16_t clampOffset(int32_t raw) {
    int32_t offset = ((raw - ADC_MIDPOINT - zeroCalCounts) * (PLOT_H / 2)) / ADC_MIDPOINT;
    offset = (offset * WAVEFORM_SCALE_NUM) / WAVEFORM_SCALE_DEN;
    if (offset > PLOT_H / 2 - 1) offset = PLOT_H / 2 - 1;
    if (offset < -(PLOT_H / 2 - 1)) offset = -(PLOT_H / 2 - 1);
    return (int16_t)offset;
}

uint32_t currentAdcSampleRateHz() { return (adcSpeedMode == ADC_SPEED_FAST) ? 4800000u : 2666667u; }

const uint32_t FFT_RANGE_SPAN_HZ[4] = { 25000u, 100000u, 400000u, 800000u };
const char *FFT_RANGE_NAME[4] = { "ZOOM", "NORM", "WIDE", "FULL" };
volatile uint8_t fftRangeIndex = 1;

int fftDecimationFactor() {
    uint32_t rate = currentAdcSampleRateHz();
    uint32_t targetRate = FFT_RANGE_SPAN_HZ[fftRangeIndex] * 2u;
    uint32_t factor = (rate + targetRate / 2) / targetRate;
    if (factor < 1) factor = 1;
    uint32_t maxFactor = ADC_CAPTURE_SAMPLES / FFT_SIZE;
    if (factor > maxFactor) factor = maxFactor;
    return (int)factor;
}

uint32_t currentFftSampleRateHz() { return currentAdcSampleRateHz() / (uint32_t)fftDecimationFactor(); }

#define FFT_DC_SKIP_HZ 200u

void computeFFT(const uint16_t *src, uint32_t base) {
    int factor = fftDecimationFactor();
    for (int i = 0; i < FFT_SIZE; i++) {
        int32_t blockSum = 0;
        for (int k = 0; k < factor; k++) {
            uint32_t idx = (base + (uint32_t)(i * factor + k)) % ADC_CAPTURE_SAMPLES;
            blockSum += (int32_t)src[idx];
        }
        int32_t avgRaw = blockSum / factor;
        int32_t centered = avgRaw - ADC_MIDPOINT - zeroCalCounts;
        float win = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (FFT_SIZE - 1));
        int32_t scaled = (int32_t)((float)centered * 16.0f * win);
        if (scaled > 32767) scaled = 32767;
        if (scaled < -32768) scaled = -32768;
        fftSrc[i] = (q15_t)scaled;
    }
    arm_rfft_q15(&fftInst, fftSrc, fftDst);
    arm_cmplx_mag_q15(fftDst, fftMag, FFT_SIZE / 2);

    uint32_t sampleRate = currentFftSampleRateHz();
    int minBin = (int)(((uint64_t)FFT_DC_SKIP_HZ * FFT_SIZE) / sampleRate);
    if (minBin < 1) minBin = 1;

    for (int i = 0; i < minBin && i < FFT_SIZE / 2; i++) fftMag[i] = 0;
}

#define FFT_PLOT_MIN_HZ 0

uint32_t fftPlotStepHz = 350u;
uint32_t fftPlotMaxHz = FFT_PLOT_MIN_HZ + 350u * (uint32_t)(PLOT_W - 1);

void applyFftRange() {
    uint32_t span = FFT_RANGE_SPAN_HZ[fftRangeIndex];
    fftPlotStepHz = span / (uint32_t)(PLOT_W - 1);
    if (fftPlotStepHz < 1) fftPlotStepHz = 1;
    fftPlotMaxHz = FFT_PLOT_MIN_HZ + fftPlotStepHz * (uint32_t)(PLOT_W - 1);
}

#define FFT_POINT_SIZE 2
#define FFT_STEM_COUNT 6
#define FFT_PEAK_MARKER_SIZE 4
#define FFT_PEAK_MARKER_RADIUS (FFT_PEAK_MARKER_SIZE / 2)

#define FFT_PEAK_MIN_SEP_PX 10

static uint16_t fftColMag[PLOT_W];
static int16_t fftColY[PLOT_W];

static int stemIdx[FFT_STEM_COUNT];
static uint16_t stemMag[FFT_STEM_COUNT];
static int stemCount = 0;

#define SPECTRUM_SMOOTH_ALPHA 0.3f
static float smoothedColMag[PLOT_W];
static bool spectrumSmoothInit = false;

void computeSpectrumColumns() {
    int bins = FFT_SIZE / 2;
    uint32_t sampleRate = currentFftSampleRateHz();

    for (int x = 0; x < PLOT_W; x++) {
        uint32_t freqHz = FFT_PLOT_MIN_HZ + (uint32_t)x * fftPlotStepHz;

        float exactBin = (float)freqHz * (float)FFT_SIZE / (float)sampleRate;
        int bin0 = (int)exactBin;
        float frac = exactBin - (float)bin0;

        float mag;
        if (bin0 < 0) {
            mag = (float)fftMag[0];
        } else if (bin0 >= bins - 1) {
            mag = (float)fftMag[bins - 1];
        } else {
            float m0 = (float)fftMag[bin0];
            float m1 = (float)fftMag[bin0 + 1];
            mag = m0 + (m1 - m0) * frac;
        }

        if (!spectrumSmoothInit) {
            smoothedColMag[x] = mag;
        } else { smoothedColMag[x] += SPECTRUM_SMOOTH_ALPHA * (mag - smoothedColMag[x]); }
        uint16_t smoothedMag = (uint16_t)(smoothedColMag[x] + 0.5f);

        float db = smoothedMag > 0 ? 20.0f * log10f((float)smoothedMag) : 0.0f;
        int y = PLOT_H - 1 - (int)((db / 90.0f) * (PLOT_H - 1));
        if (y < 0) y = 0;
        if (y > PLOT_H - 1 - FFT_PEAK_MARKER_RADIUS) y = PLOT_H - 1 - FFT_PEAK_MARKER_RADIUS;

        fftColMag[x] = smoothedMag;
        fftColY[x] = (int16_t)y;
    }
    spectrumSmoothInit = true;
}

void pickDrawnPeaks() {
    int candIdx[PLOT_W];
    uint16_t candMag[PLOT_W];
    int candCount = 0;
    for (int x = 0; x < PLOT_W; x++) {
        uint16_t mag = fftColMag[x];
        uint16_t left = (x > 0) ? fftColMag[x - 1] : 0;
        uint16_t right = (x < PLOT_W - 1) ? fftColMag[x + 1] : 0;

        if (mag >= left && mag > right) {
            candIdx[candCount] = x;
            candMag[candCount] = mag;
            candCount++;
        }
    }

    stemCount = 0;
    bool candUsed[PLOT_W] = { false };
    while (stemCount < FFT_STEM_COUNT) {
        int bestC = -1;
        for (int c = 0; c < candCount; c++) {
            if (candUsed[c]) continue;
            if (bestC == -1 || candMag[c] > candMag[bestC]) bestC = c;
        }
        if (bestC == -1) break;
        candUsed[bestC] = true;

        int x = candIdx[bestC];
        bool tooClose = false;
        for (int i = 0; i < stemCount; i++) {
            if (abs(stemIdx[i] - x) < FFT_PEAK_MIN_SEP_PX) {
                tooClose = true;
                break;
            }
        }
        if (tooClose) continue;

        stemIdx[stemCount] = x;
        stemMag[stemCount] = candMag[bestC];
        stemCount++;
    }
}

void drawSpectrumPoints() {
    tft.startWrite();

    for (int x = 0; x < PLOT_W; x++) { tft.fillRect(PLOT_X + x, PLOT_Y + fftColY[x], FFT_POINT_SIZE, FFT_POINT_SIZE, COLOR_SPECTRUM); }

    for (int i = 0; i < stemCount; i++) {
        int x = stemIdx[i];
        int cx = PLOT_X + x;
        int cy = PLOT_Y + fftColY[x];
        if (cx < PLOT_X + FFT_PEAK_MARKER_RADIUS) cx = PLOT_X + FFT_PEAK_MARKER_RADIUS;
        if (cx > PLOT_X + PLOT_W - 1 - FFT_PEAK_MARKER_RADIUS) cx = PLOT_X + PLOT_W - 1 - FFT_PEAK_MARKER_RADIUS;
        if (cy < PLOT_Y + FFT_PEAK_MARKER_RADIUS) cy = PLOT_Y + FFT_PEAK_MARKER_RADIUS;
        if (cy > PLOT_Y + PLOT_H - 1 - FFT_PEAK_MARKER_RADIUS) cy = PLOT_Y + PLOT_H - 1 - FFT_PEAK_MARKER_RADIUS;
        tft.fillCircle(cx, cy, FFT_PEAK_MARKER_RADIUS, COLOR_SPECTRUM);
    }

    tft.endWrite();
}

#define ACCURATE_SCAN_POINTS 100
#define ACCURATE_SCAN_STEP_HZ 5.0f

float goertzelMag(const q15_t *samples, int n, float freqHz, float sampleRateHz) {
    float k = (float)n * freqHz / sampleRateHz;
    float w = 2.0f * (float)M_PI * k / (float)n;
    float cosine = cosf(w);
    float coeff = 2.0f * cosine;
    float sine = sinf(w);
    float q1 = 0.0f, q2 = 0.0f;
    for (int i = 0; i < n; i++) {
        float q0 = coeff * q1 - q2 + (float)samples[i];
        q2 = q1;
        q1 = q0;
    }
    float real = q1 - q2 * cosine;
    float imag = q2 * sine;
    return sqrtf(real * real + imag * imag);
}

float refinePeakFreqHz(float coarseHz) {
    float sampleRateHz = (float)currentFftSampleRateHz();
    float startHz = coarseHz - (ACCURATE_SCAN_POINTS / 2) * ACCURATE_SCAN_STEP_HZ;
    if (startHz < 0.0f) startHz = 0.0f;

    float bestHz = coarseHz;
    float bestMag = -1.0f;
    for (int i = 0; i < ACCURATE_SCAN_POINTS; i++) {
        float f = startHz + (float)i * ACCURATE_SCAN_STEP_HZ;
        if (f >= sampleRateHz / 2.0f) break;
        float mag = goertzelMag(fftSrc, FFT_SIZE, f, sampleRateHz);
        if (mag > bestMag) {
            bestMag = mag;
            bestHz = f;
        }
    }
    return bestHz;
}

#define TIMEBASE_INTERP_LEVELS 3
#define TIMEBASE_TRIG_STEPS 51
#define TIMEBASE_TRIG_LEVELS (TIMEBASE_INTERP_LEVELS + TIMEBASE_TRIG_STEPS)
#define TIMEBASE_ROLL_LEVELS 35
#define TIMEBASE_LEVELS (TIMEBASE_TRIG_LEVELS + TIMEBASE_ROLL_LEVELS)
#define TIMEBASE_DEFAULT_LEVEL (TIMEBASE_INTERP_LEVELS + 31)

const float interpStep[TIMEBASE_INTERP_LEVELS] = { 0.125f, 0.25f, 0.5f };

const int32_t timebaseStride[TIMEBASE_TRIG_STEPS] = {
    1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 11,
    12, 13, 15, 16, 18, 20, 23, 26, 29, 32, 36, 40, 45, 50, 56, 62, 69, 78, 87, 97,
    108, 121, 135, 151, 169, 188, 210, 235, 263, 294, 328
};

const uint32_t rollDivMs[TIMEBASE_ROLL_LEVELS] = {
    20, 22, 25, 28, 32, 36, 40, 45, 50, 56, 63, 71, 80, 89, 100, 112, 126, 141,
    159, 178, 200, 224, 251, 282, 316, 355, 398, 447, 501, 563, 631, 708, 794, 891, 1000
};

int timebaseLevel = TIMEBASE_DEFAULT_LEVEL;

inline bool isRollLevel(int level) { return level >= TIMEBASE_TRIG_LEVELS; }

inline bool isInterpLevel(int level) { return level < TIMEBASE_INTERP_LEVELS; }

#define INTERP_NUM_TAPS 32

const float32_t interpCoeffs8[INTERP_NUM_TAPS] = { -0.00047714f, -0.00360496f, -0.01092732f, -0.02284785f, -0.03726816f, -0.04863857f, -0.04795195f, -0.02407124f, 0.03363191f, 0.13186376f, 0.27018207f, 0.43906897f, 0.62018524f, 0.78910727f, 0.92008435f, 0.99166361f, 0.99166361f, 0.92008435f, 0.78910727f, 0.62018524f, 0.43906897f, 0.27018207f, 0.13186376f, 0.03363191f, -0.02407124f, -0.04795195f, -0.04863857f, -0.03726816f, -0.02284785f, -0.01092732f, -0.00360496f, -0.00047714f };
const float32_t interpCoeffs4[INTERP_NUM_TAPS] = { -0.00046741f, -0.00299379f, -0.00606356f, -0.00445202f, 0.00726188f, 0.02698953f, 0.03982247f, 0.02358022f, -0.03294588f, -0.10950838f, -0.14992395f, -0.08555473f, 0.12084611f, 0.43787540f, 0.76409887f, 0.97143523f, 0.97143523f, 0.76409887f, 0.43787540f, 0.12084611f, -0.08555473f, -0.14992395f, -0.10950838f, -0.03294588f, 0.02358022f, 0.03982247f, 0.02698953f, 0.00726188f, -0.00445202f, -0.00606356f, -0.00299379f, -0.00046741f };
const float32_t interpCoeffs2[INTERP_NUM_TAPS] = { -0.00043200f, -0.00114614f, 0.00232136f, 0.00411479f, -0.00671182f, -0.01033262f, 0.01524556f, 0.02179409f, -0.03045032f, -0.04192398f, 0.05739660f, 0.07907421f, -0.11169238f, -0.16763539f, 0.29252617f, 0.89785187f, 0.89785187f, 0.29252617f, -0.16763539f, -0.11169238f, 0.07907421f, 0.05739660f, -0.04192398f, -0.03045032f, 0.02179409f, 0.01524556f, -0.01033262f, -0.00671182f, 0.00411479f, 0.00232136f, -0.00114614f, -0.00043200f };

float32_t interpState8[INTERP_NUM_TAPS / 8 + (PLOT_W / 8) - 1];
float32_t interpState4[INTERP_NUM_TAPS / 4 + (PLOT_W / 4) - 1];
float32_t interpState2[INTERP_NUM_TAPS / 2 + (PLOT_W / 2) - 1];

arm_fir_interpolate_instance_f32 interpInst8;
arm_fir_interpolate_instance_f32 interpInst4;
arm_fir_interpolate_instance_f32 interpInst2;

float32_t interpInBlock[PLOT_W / 2];
float32_t interpOutBlock[PLOT_W];

void interpFiltersInit() {
    arm_fir_interpolate_init_f32(&interpInst8, 8, INTERP_NUM_TAPS, (float32_t *)interpCoeffs8, interpState8, PLOT_W / 8);
    arm_fir_interpolate_init_f32(&interpInst4, 4, INTERP_NUM_TAPS, (float32_t *)interpCoeffs4, interpState4, PLOT_W / 4);
    arm_fir_interpolate_init_f32(&interpInst2, 2, INTERP_NUM_TAPS, (float32_t *)interpCoeffs2, interpState2, PLOT_W / 2);
}

void enterTriggeredMode() {
    adcReconfigure(ENABLE, ADC_EXTERNALTRIGCONVEDGE_NONE, ADC_SOFTWARE_START);
    captureMode = CAPTURE_TRIGGERED;
    triggerArmed = false;
    if (!paused) { HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuf, ADC_CAPTURE_SAMPLES); }
}

void enterRollMode(int level) {
    rollTimerSetPeriod(rollDivMs[level - TIMEBASE_TRIG_LEVELS]);
    adcReconfigure(DISABLE, ADC_EXTERNALTRIGCONVEDGE_RISING, ADC_EXTERNALTRIGCONV_T3_TRGO);
    if (captureMode != CAPTURE_ROLL) {
        uint32_t base = (viewAnchorIdx + (uint32_t)scrollOffsetSamples) % ADC_CAPTURE_SAMPLES;
        int step = (int)ADC_CAPTURE_SAMPLES / PLOT_W;
        if (step < 1) step = 1;
        for (int i = 0; i < PLOT_W; i++) {
            uint32_t idx = (base + (uint32_t)(i * step)) % ADC_CAPTURE_SAMPLES;
            rollSamples[i] = clampOffset((int32_t)adcBuf[idx]);
        }
        rollWriteIdx = 0;
    }
    captureMode = CAPTURE_ROLL;
    if (!paused) { HAL_ADC_Start_IT(&hadc1); }
}

void applyTimebaseLevel(int level) {
    if (isRollLevel(level)) enterRollMode(level);
    else enterTriggeredMode();
}

#define TIMEBASE_BASE_STRIDE 41

int computeStride(int count) {
    (void)count;
    return timebaseStride[timebaseLevel - TIMEBASE_INTERP_LEVELS];
}

int interpBlockSize(int count) {
    int L = (int)(1.0f / interpStep[timebaseLevel] + 0.5f);
    return count / L;
}

int neededSpan(int count) {

    if (isRollLevel(timebaseLevel)) return (int)ADC_CAPTURE_SAMPLES - 1;
    int span = isInterpLevel(timebaseLevel) ? interpBlockSize(count) : computeStride(count) * count;
    if (span > (int)ADC_CAPTURE_SAMPLES - 1) span = (int)ADC_CAPTURE_SAMPLES - 1;
    return span;
}

void generateWaveform(int16_t *samples, int count) {
    if (captureMode == CAPTURE_ROLL) {
        int start = rollWriteIdx;
        for (int i = 0; i < count; i++) {
            int idx = start + i;
            if (idx >= PLOT_W) idx -= PLOT_W;
            int16_t s = rollSamples[idx];
            samples[i] = s;
            waveformMin[i] = s;
            waveformMax[i] = s;
        }
        return;
    }

    uint32_t base = (viewAnchorIdx + (uint32_t)scrollOffsetSamples) % ADC_CAPTURE_SAMPLES;

    if (isInterpLevel(timebaseLevel)) {
#if RAW_ADC_MODE
        int L = (int)(1.0f / interpStep[timebaseLevel] + 0.5f);
        int blockSize = count / L;
        for (int i = 0; i < count; i++) {
            int srcI = (i * blockSize) / count;
            uint32_t idx = (base + (uint32_t)srcI) % ADC_CAPTURE_SAMPLES;
            uint16_t v = adcBuf[idx];
            if (v <= ADC_RAIL_LOW_COUNTS || v >= ADC_RAIL_HIGH_COUNTS) waveformClipped = true;
            int16_t s = clampOffset((int32_t)v);
            samples[i] = s;
            waveformMin[i] = s;
            waveformMax[i] = s;
        }
        return;
#else
        int L = (int)(1.0f / interpStep[timebaseLevel] + 0.5f);
        int blockSize = count / L;
        arm_fir_interpolate_instance_f32 *inst = &interpInst2;
        if (L == 8) inst = &interpInst8;
        else if (L == 4) inst = &interpInst4;
        else inst = &interpInst2;
        int phaseLen = INTERP_NUM_TAPS / L;
        int stateLen = phaseLen + blockSize - 1;
        memset(inst->pState, 0, stateLen * sizeof(float32_t));
        for (int j = 0; j < phaseLen - 1; j++) {
            uint32_t idx = (base + ADC_CAPTURE_SAMPLES - (uint32_t)(phaseLen - 1 - j)) % ADC_CAPTURE_SAMPLES;
            inst->pState[j] = (float32_t)adcBuf[idx];
        }
        for (int i = 0; i < blockSize; i++) {
            uint32_t idx = (base + (uint32_t)i) % ADC_CAPTURE_SAMPLES;
            uint16_t v = adcBuf[idx];
            if (v <= ADC_RAIL_LOW_COUNTS || v >= ADC_RAIL_HIGH_COUNTS) waveformClipped = true;
            interpInBlock[i] = (float32_t)v;
        }
        arm_fir_interpolate_f32(inst, interpInBlock, interpOutBlock, blockSize);
        for (int i = 0; i < count; i++) {
            int32_t raw = (int32_t)(interpOutBlock[i] + 0.5f);
            int16_t s = clampOffset(raw);
            samples[i] = s;
            waveformMin[i] = s;
            waveformMax[i] = s;
        }
        return;
#endif
    }

    int stride = computeStride(count);

    for (int i = 0; i < count; i++) {
        uint32_t colBase = (base + (uint32_t)(i * stride)) % ADC_CAPTURE_SAMPLES;
#if RAW_ADC_MODE
        int32_t v = adcBuf[colBase];
        if (v <= ADC_RAIL_LOW_COUNTS || v >= ADC_RAIL_HIGH_COUNTS) waveformClipped = true;
        int16_t s = clampOffset(v);
        samples[i] = s;
        waveformMax[i] = s;
        waveformMin[i] = s;
#else
        int32_t sum = 0;
        int32_t rawMin = 4096, rawMax = 0;
        for (int k = 0; k < stride; k++) {
            uint32_t idx = (colBase + (uint32_t)k) % ADC_CAPTURE_SAMPLES;
            int32_t v = adcBuf[idx];
            sum += v;
            if (v < rawMin) rawMin = v;
            if (v > rawMax) rawMax = v;
        }
        if (rawMin <= ADC_RAIL_LOW_COUNTS || rawMax >= ADC_RAIL_HIGH_COUNTS) waveformClipped = true;
        int32_t raw = sum / stride;
        samples[i] = clampOffset(raw);
        waveformMax[i] = clampOffset(rawMax);
        waveformMin[i] = clampOffset(rawMin);
#endif
    }
}

int32_t scrollMidOffset() {
    int32_t maxOffset = (int32_t)ADC_CAPTURE_SAMPLES - neededSpan(PLOT_W) - 1;
    if (maxOffset < 0) maxOffset = 0;
    return maxOffset / 2;
}

void captureTriggeredPoll() {
    if (captureMode != CAPTURE_TRIGGERED || !triggerArmed) return;

    uint32_t writeIdx = ADC_CAPTURE_SAMPLES - __HAL_DMA_GET_COUNTER(&hdmaAdc1);
    writeIdx %= ADC_CAPTURE_SAMPLES;
    uint32_t distance = (writeIdx + ADC_CAPTURE_SAMPLES - triggerIdx) % ADC_CAPTURE_SAMPLES;

    if ((int)distance < neededSpan(PLOT_W)) return;

    triggerArmed = false;
    viewAnchorIdx = triggerIdx;
    scrollOffsetSamples = scrollMidOffset();
    generateWaveform(waveformSamples, PLOT_W);
    requestRedraw();

    if (triggerMode == TRIG_SINGLE) { pauseRequested = true; }
}

bool isGridCol[PLOT_W];
bool isGridRow[PLOT_H];

void initGridLookup() {
    for (int x = 0; x < PLOT_W; x++) isGridCol[x] = false;
    for (int y = 0; y < PLOT_H; y++) isGridRow[y] = false;
    for (int c = 0; c <= GRID_COLS; c++) isGridCol[(c * (PLOT_W - 1)) / GRID_COLS] = true;
    for (int r = 0; r <= GRID_ROWS; r++) isGridRow[(r * (PLOT_H - 1)) / GRID_ROWS] = true;
}

void pushPlotBufferDMA() {
    tft.startWrite();
    tft.setAddrWindow(PLOT_X, PLOT_Y, PLOT_W, PLOT_H);

    int bufIdx = 0;
    for (int y = 0; y < PLOT_H; y++) {
        bool gridRow = isGridRow[y];
        bool evenY = (y & 1) == 0;
        bool cursorRow = cursorEnabled && (y == cursorPixelY);
        for (int x = 0; x < PLOT_W; x++) {
            bool gridHere = (gridRow && (x & 1) == 0) || (isGridCol[x] && evenY);
            uint16_t base = gridHere ? COLOR_GRID : COLOR_BG;
            if (cursorRow) base = COLOR_CURSOR;
            uint16_t c = getPixel(traceBuf, x, y) ? COLOR_TRACE : base;
            lineBuf[bufIdx][x] = SWAP16(c);
        }
        spiDmaWait();
        HAL_SPI_Transmit_DMA(hspi1Handle, (uint8_t *)lineBuf[bufIdx], PLOT_W * 2);
        bufIdx = 1 - bufIdx;
    }
    spiDmaWait();
    tft.endWrite();
}

#define UI_MARGIN 4

#define BOTTOM_BAR_Y (PLOT_Y + PLOT_H + 2 + 9 + UI_MARGIN)
#define BOTTOM_BAR_GAP 2

#define VDIV_LABEL_W 40
#define VDIV_LABEL_H 10

TFT_eSprite vdivSprite = TFT_eSprite(&tft);

void vdivSpriteInit() {
    vdivSprite.setColorDepth(1);
    vdivSprite.createSprite(VDIV_LABEL_W, VDIV_LABEL_H);
    vdivSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    vdivSprite.setTextColor(1, 0);
    vdivSprite.setTextSize(1);
    vdivSprite.fillSprite(0);
}

#define COUPLING_LABEL_W 16
#define COUPLING_LABEL_H 10

TFT_eSprite couplingSprite = TFT_eSprite(&tft);

void couplingSpriteInit() {
    couplingSprite.setColorDepth(1);
    couplingSprite.createSprite(COUPLING_LABEL_W, COUPLING_LABEL_H);
    couplingSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    couplingSprite.setTextColor(1, 0);
    couplingSprite.setTextSize(1);
    couplingSprite.fillSprite(0);
}

void updateCouplingLabel() {
    couplingSprite.fillSprite(0);
    int tw = couplingSprite.textWidth("DC");
    couplingSprite.setCursor((COUPLING_LABEL_W - tw) / 2, 0);
    couplingSprite.print("DC");
    couplingSprite.pushSprite(UI_MARGIN + VDIV_LABEL_W + BOTTOM_BAR_GAP, BOTTOM_BAR_Y);
}

#define TB_LABEL_W 34
#define TB_LABEL_H 10

TFT_eSprite tbSprite = TFT_eSprite(&tft);

void tbSpriteInit() {
    tbSprite.setColorDepth(1);
    tbSprite.createSprite(TB_LABEL_W, TB_LABEL_H);
    tbSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    tbSprite.setTextColor(1, 0);
    tbSprite.setTextSize(1);
    tbSprite.fillSprite(0);
}

const float TIME_PER_STRIDE_UNIT_US = 500.0f / 41.0f;

// NOTE: no %f anywhere here. snprintf's float support isn't linked on this
// target (newlib-nano without -u _printf_float), so %f silently prints
// garbage. All formatting below rounds to an integer scaled by 10 or 100
// and prints with %ld / %02ld instead.
void formatTimeUs(float us, char *buf, size_t n) {
    if (us < 1000.0f) {
        if (us < 10.0f) {
            long hundredths = (long)(us * 100.0f + 0.5f);
            snprintf(buf, n, "%ld.%02ldus", hundredths / 100, hundredths % 100);
        } else if (us < 100.0f) {
            long tenths = (long)(us * 10.0f + 0.5f);
            snprintf(buf, n, "%ld.%ldus", tenths / 10, tenths % 10);
        } else {
            snprintf(buf, n, "%ldus", (long)(us + 0.5f));
        }
    } else if (us < 1000000.0f) {
        float ms = us / 1000.0f;
        if (ms < 10.0f) {
            long hundredths = (long)(ms * 100.0f + 0.5f);
            snprintf(buf, n, "%ld.%02ldms", hundredths / 100, hundredths % 100);
        } else if (ms < 100.0f) {
            long tenths = (long)(ms * 10.0f + 0.5f);
            snprintf(buf, n, "%ld.%ldms", tenths / 10, tenths % 10);
        } else {
            snprintf(buf, n, "%ldms", (long)(ms + 0.5f));
        }
    } else {
        long hundredths = (long)((us / 1000000.0f) * 100.0f + 0.5f);
        snprintf(buf, n, "%ld.%02lds", hundredths / 100, hundredths % 100);
    }
}

void updateTimebaseLabel() {
    tbSprite.fillSprite(0);
    char label[16];
    if (isInterpLevel(timebaseLevel)) {
        static const char *interpLabels[TIMEBASE_INTERP_LEVELS] = { "1.5us", "3us", "6us" };
        snprintf(label, sizeof(label), "%s", interpLabels[timebaseLevel]);
    } else if (isRollLevel(timebaseLevel)) {
        formatTimeUs((float)rollDivMs[timebaseLevel - TIMEBASE_TRIG_LEVELS] * 1000.0f, label, sizeof(label));
    } else {
        float us = (float)timebaseStride[timebaseLevel - TIMEBASE_INTERP_LEVELS] * TIME_PER_STRIDE_UNIT_US;
        formatTimeUs(us, label, sizeof(label));
    }
    int tw = tbSprite.textWidth(label);
    tbSprite.setCursor((TB_LABEL_W - tw) / 2, 0);
    tbSprite.print(label);
    tbSprite.pushSprite(UI_MARGIN + VDIV_LABEL_W + BOTTOM_BAR_GAP + COUPLING_LABEL_W + BOTTOM_BAR_GAP, BOTTOM_BAR_Y);
}

#define TRIG_LABEL_W 34
#define TRIG_LABEL_H 10

TFT_eSprite trigSprite = TFT_eSprite(&tft);

void trigSpriteInit() {
    trigSprite.setColorDepth(1);
    trigSprite.createSprite(TRIG_LABEL_W, TRIG_LABEL_H);
    trigSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    trigSprite.setTextColor(1, 0);
    trigSprite.setTextSize(1);
    trigSprite.fillSprite(0);
}

void updateTriggerLabel() {
    trigSprite.fillSprite(0);
    char trigText[8];
    snprintf(trigText, sizeof(trigText), "t%s", triggerMode == TRIG_AUTO ? "AUTO" : (triggerMode == TRIG_SINGLE ? "SING" : "NORM"));
    int tw = trigSprite.textWidth(trigText);
    trigSprite.setCursor((TRIG_LABEL_W - tw) / 2, 0);
    trigSprite.print(trigText);
    trigSprite.pushSprite(UI_MARGIN + VDIV_LABEL_W + BOTTOM_BAR_GAP + COUPLING_LABEL_W + BOTTOM_BAR_GAP + TB_LABEL_W + BOTTOM_BAR_GAP, BOTTOM_BAR_Y);
}

#define RANGE_LABEL_W 34
#define RANGE_LABEL_H 10

TFT_eSprite rangeSprite = TFT_eSprite(&tft);

void rangeSpriteInit() {
    rangeSprite.setColorDepth(1);
    rangeSprite.createSprite(RANGE_LABEL_W, RANGE_LABEL_H);
    rangeSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    rangeSprite.setTextColor(1, 0);
    rangeSprite.setTextSize(1);
    rangeSprite.fillSprite(0);
}

void updateRangeLabel() {
    rangeSprite.fillSprite(0);
    char rangeText[8];
    snprintf(rangeText, sizeof(rangeText), "r%s", FFT_RANGE_NAME[fftRangeIndex]);
    int tw = rangeSprite.textWidth(rangeText);
    rangeSprite.setCursor((RANGE_LABEL_W - tw) / 2, 0);
    rangeSprite.print(rangeText);
    rangeSprite.pushSprite(UI_MARGIN + VDIV_LABEL_W + BOTTOM_BAR_GAP + COUPLING_LABEL_W + BOTTOM_BAR_GAP + TB_LABEL_W + BOTTOM_BAR_GAP + TRIG_LABEL_W + BOTTOM_BAR_GAP, BOTTOM_BAR_Y);
}

#define SCROLLBAR_X (UI_MARGIN + VDIV_LABEL_W + BOTTOM_BAR_GAP + COUPLING_LABEL_W + BOTTOM_BAR_GAP + TB_LABEL_W + BOTTOM_BAR_GAP + TRIG_LABEL_W + BOTTOM_BAR_GAP + RANGE_LABEL_W + BOTTOM_BAR_GAP)
#define SCROLLBAR_W 80
#define SCROLLBAR_H 10
#define SCROLLBAR_THUMB_THICK 6

#define SCROLLBAR_BRACKET_W 6
#define SCROLLBAR_TRACK_W (SCROLLBAR_W - 2 * SCROLLBAR_BRACKET_W)

TFT_eSprite scrollbarSprite = TFT_eSprite(&tft);

void scrollbarSpriteInit() {
    scrollbarSprite.setColorDepth(16);
    scrollbarSprite.createSprite(SCROLLBAR_W, SCROLLBAR_H);
    scrollbarSprite.fillSprite(COLOR_BG);
}

void drawScrollbar() {
    scrollbarSprite.fillSprite(COLOR_BG);

    int trackY = SCROLLBAR_H / 2;
    scrollbarSprite.drawFastHLine(SCROLLBAR_BRACKET_W, trackY, SCROLLBAR_TRACK_W, COLOR_UI_TEXT);

    uint32_t span = (uint32_t)neededSpan(PLOT_W);
    float frac = (float)span / (float)ADC_CAPTURE_SAMPLES;
    if (frac > 1.0f) frac = 1.0f;
    int thumbW = (int)(SCROLLBAR_TRACK_W * frac + 0.5f);
    if (thumbW < 3) thumbW = 3;
    if (thumbW > SCROLLBAR_TRACK_W) thumbW = SCROLLBAR_TRACK_W;

    int32_t maxOffset = (int32_t)ADC_CAPTURE_SAMPLES - (int32_t)span - 1;
    if (maxOffset < 0) maxOffset = 0;
    float posFrac = isRollLevel(timebaseLevel) || maxOffset == 0
        ? 0.5f
        : (float)scrollOffsetSamples / (float)maxOffset;
    int thumbX = SCROLLBAR_BRACKET_W + (int)(posFrac * (SCROLLBAR_TRACK_W - thumbW) + 0.5f);
    if (thumbX < SCROLLBAR_BRACKET_W) thumbX = SCROLLBAR_BRACKET_W;
    if (thumbX > SCROLLBAR_BRACKET_W + SCROLLBAR_TRACK_W - thumbW) thumbX = SCROLLBAR_BRACKET_W + SCROLLBAR_TRACK_W - thumbW;

    int thumbY = (SCROLLBAR_H - SCROLLBAR_THUMB_THICK) / 2;
    scrollbarSprite.fillRect(thumbX, thumbY, thumbW, SCROLLBAR_THUMB_THICK, COLOR_UI_TEXT);

    int bracketTop = (SCROLLBAR_H - SCROLLBAR_THUMB_THICK) / 2 - 1;
    int bracketBottom = bracketTop + SCROLLBAR_THUMB_THICK + 1;
    int bracketTick = 3;
    int bracketLeftX = 5;
    int bracketRightX = SCROLLBAR_W - 6;

    scrollbarSprite.drawFastVLine(bracketLeftX, bracketTop, bracketBottom - bracketTop + 1, COLOR_UI_TEXT);
    scrollbarSprite.drawFastHLine(bracketLeftX, bracketTop, bracketTick, COLOR_UI_TEXT);
    scrollbarSprite.drawFastHLine(bracketLeftX, bracketBottom, bracketTick, COLOR_UI_TEXT);

    scrollbarSprite.drawFastVLine(bracketRightX, bracketTop, bracketBottom - bracketTop + 1, COLOR_UI_TEXT);
    scrollbarSprite.drawFastHLine(bracketRightX - bracketTick + 1, bracketTop, bracketTick, COLOR_UI_TEXT);
    scrollbarSprite.drawFastHLine(bracketRightX - bracketTick + 1, bracketBottom, bracketTick, COLOR_UI_TEXT);

    scrollbarSprite.pushSprite(SCROLLBAR_X, BOTTOM_BAR_Y - 1);
}

#define FFT_INFO_LABEL_W SCREEN_W
#define FFT_INFO_LABEL_H 9
#define FFT_INFO_LABEL_Y (PLOT_Y + PLOT_H + 2)

TFT_eSprite fftInfoSprite = TFT_eSprite(&tft);

void fftInfoSpriteInit() {
    fftInfoSprite.setColorDepth(1);
    fftInfoSprite.createSprite(FFT_INFO_LABEL_W, FFT_INFO_LABEL_H);
    fftInfoSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    fftInfoSprite.setTextColor(1, 0);
    fftInfoSprite.setTextSize(1);
    fftInfoSprite.fillSprite(0);
}

void formatFreqHz(char *buf, size_t bufSize, uint32_t hz) {
    if (hz >= 1000) {
        snprintf(buf, bufSize, "%lukHz", (unsigned long)(hz / 1000));
    } else { snprintf(buf, bufSize, "%luHz", (unsigned long)hz); }
}

#define FREQ_SMOOTH_ALPHA 0.25f
#define FREQ_SMOOTH_SNAP_HZ 50.0f
static float displayedPkHz[5];
static bool displayedPkValid[5] = { false, false, false, false, false };

void updateFftInfoLabel() {
    int labelCount = stemCount < 5 ? stemCount : 5;

    uint32_t pkHz[5];
    char pkBuf[5][12];
    for (int i = 0; i < labelCount; i++) {
        float coarseHz = (float)FFT_PLOT_MIN_HZ + (float)stemIdx[i] * (float)fftPlotStepHz;
        float rawHz = refinePeakFreqHz(coarseHz);

        if (displayedPkValid[i] && fabsf(rawHz - displayedPkHz[i]) < FREQ_SMOOTH_SNAP_HZ) {
            displayedPkHz[i] += FREQ_SMOOTH_ALPHA * (rawHz - displayedPkHz[i]);
        } else { displayedPkHz[i] = rawHz; }
        displayedPkValid[i] = true;

        pkHz[i] = (uint32_t)(displayedPkHz[i] + 0.5f);
        formatFreqHz(pkBuf[i], sizeof(pkBuf[i]), pkHz[i]);
    }
    for (int i = labelCount; i < 5; i++) displayedPkValid[i] = false;

    fftInfoSprite.fillSprite(0);

    int placedLeft[5], placedRight[5];
    int placedCount = 0;
    const int LABEL_GAP_PX = 2;

    for (int i = 0; i < labelCount; i++) {
        int col = (int)(pkHz[i] / fftPlotStepHz);
        if (col < 0) col = 0;
        if (col > PLOT_W - 1) col = PLOT_W - 1;
        int px = PLOT_X + col;

        int w = fftInfoSprite.textWidth(pkBuf[i]);
        int cursorX = px - w / 2;
        if (cursorX < 5) cursorX = 5;
        if (cursorX > PLOT_X + PLOT_W - w) cursorX = PLOT_X + PLOT_W - w;

        int left = cursorX - LABEL_GAP_PX;
        int right = cursorX + w + LABEL_GAP_PX;
        bool overlaps = false;
        for (int j = 0; j < placedCount; j++) {
            if (left < placedRight[j] && right > placedLeft[j]) {
                overlaps = true;
                break;
            }
        }
        if (overlaps) continue;

        fftInfoSprite.setCursor(cursorX, 0);
        fftInfoSprite.print(pkBuf[i]);
        placedLeft[placedCount] = left;
        placedRight[placedCount] = right;
        placedCount++;
    }

    fftInfoSprite.pushSprite(0, FFT_INFO_LABEL_Y);
}

#define ADC_VREF_MV 3300
#define ADC_MIDPOINT_MV (ADC_VREF_MV / 2)
#define VOLT_CAL_OFFSET_MV 0

inline int32_t pixelsToMilliVolts(int16_t px) { return ((int32_t)px * ADC_MIDPOINT_MV) / (PLOT_H / 2) + VOLT_CAL_OFFSET_MV; }

char voltsPerDivLabel[16] = "?V";

void updateVoltsPerDivLabel() {
    int32_t divPx = PLOT_H / GRID_ROWS;
    int32_t mvPerDiv = pixelsToMilliVolts((int16_t)divPx) - pixelsToMilliVolts(0);
    snprintf(voltsPerDivLabel, sizeof(voltsPerDivLabel), "%ld.%02ldV",
             (long)(mvPerDiv / 1000), (long)((mvPerDiv % 1000) / 10));
    vdivSprite.fillSprite(0);
    int tw = vdivSprite.textWidth(voltsPerDivLabel);
    vdivSprite.setCursor((VDIV_LABEL_W - tw) / 2, 0);
    vdivSprite.print(voltsPerDivLabel);
    vdivSprite.pushSprite(UI_MARGIN, BOTTOM_BAR_Y);
}

#define FREQ_LABEL_W 130
#define FREQ_LABEL_H 10

TFT_eSprite freqSprite = TFT_eSprite(&tft);

void freqSpriteInit() {
    freqSprite.setColorDepth(1);
    freqSprite.createSprite(FREQ_LABEL_W, FREQ_LABEL_H);
    freqSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    freqSprite.setTextColor(1, 0);
    freqSprite.setTextSize(1);
    freqSprite.fillSprite(0);
}

void updateFreqLabel() {
    freqSprite.fillSprite(0);
    char buf[24];
    if (!measurementValid) {
        snprintf(buf, sizeof(buf), "-- --");
    } else {
        uint32_t f = measuredFreqMilliHz;
        uint32_t dutyPct = (measuredDutyPermille + 5) / 10;
        if (f < 1000000u) {
            snprintf(buf, sizeof(buf), "%lu.%02luHz %lu%%", (unsigned long)(f / 1000u), (unsigned long)((f % 1000u) / 10u), (unsigned long)dutyPct);
        } else { snprintf(buf, sizeof(buf), "%lu.%03lukHz %lu%%", (unsigned long)(f / 1000000u), (unsigned long)((f / 1000u) % 1000u), (unsigned long)dutyPct); }
    }
    freqSprite.setCursor(0, 0);
    freqSprite.print(buf);
    freqSprite.pushSprite(UI_MARGIN, UI_MARGIN);
}

void computeVoltStats(int32_t *vminMv, int32_t *vmaxMv, int32_t *vppMv, int32_t *avgMv) {
    int16_t rawMin = waveformMin[0];
    int16_t rawMax = waveformMax[0];
    int64_t sum = 0;
    for (int i = 0; i < PLOT_W; i++) {
        if (waveformMin[i] < rawMin) rawMin = waveformMin[i];
        if (waveformMax[i] > rawMax) rawMax = waveformMax[i];
        sum += waveformMin[i];
        sum += waveformMax[i];
    }
    *vmaxMv = pixelsToMilliVolts(rawMax);
    *vminMv = pixelsToMilliVolts(rawMin);
    *vppMv = *vmaxMv - *vminMv;
    int32_t avgRaw = (int32_t)(sum / (2 * PLOT_W));
    *avgMv = pixelsToMilliVolts(avgRaw);
}

void formatVolts(char *buf, size_t bufSize, int32_t mv) {
    bool neg = mv < 0;
    uint32_t absMv = neg ? (uint32_t)(-mv) : (uint32_t)mv;
    snprintf(buf, bufSize, "%s%lu.%02lu", neg ? "-" : "", (unsigned long)(absMv / 1000u), (unsigned long)((absMv % 1000u) / 10u));
}

#define MEAS_LABEL_W 200
#define MEAS_LABEL_H 10

TFT_eSprite measSprite = TFT_eSprite(&tft);

void measSpriteInit() {
    measSprite.setColorDepth(1);
    measSprite.createSprite(MEAS_LABEL_W, MEAS_LABEL_H);
    measSprite.setBitmapColor(COLOR_UI_TEXT, COLOR_BG);
    measSprite.setTextColor(1, 0);
    measSprite.setTextSize(1);
    measSprite.fillSprite(0);
}

void updateMeasLabel() {
    char buf[50];

    int32_t vminMv, vmaxMv, vppMv, avgMv;
    computeVoltStats(&vminMv, &vmaxMv, &vppMv, &avgMv);

    bool clipped = waveformClipped;
    waveformClipped = false;

    if (clipped) {
        snprintf(buf, sizeof(buf), "CLIPPED - reduce input");
    } else {
        char vminBuf[10], vmaxBuf[10], vppBuf[10], avgBuf[10];
        formatVolts(vminBuf, sizeof(vminBuf), vminMv);
        formatVolts(vmaxBuf, sizeof(vmaxBuf), vmaxMv);
        formatVolts(vppBuf, sizeof(vppBuf), vppMv);
        formatVolts(avgBuf, sizeof(avgBuf), avgMv);
        snprintf(buf, sizeof(buf), "MIN%s MAX%s PP%s AVG%s", vminBuf, vmaxBuf, vppBuf, avgBuf);
    }

    measSprite.fillSprite(0);
    int w = measSprite.textWidth(buf);
    measSprite.setCursor(MEAS_LABEL_W - w, 0);
    measSprite.print(buf);
    measSprite.pushSprite(SCREEN_W - UI_MARGIN - MEAS_LABEL_W, UI_MARGIN);
}

volatile bool paused = false;
volatile bool pauseRequested = false;

void updatePauseBorder() { tft.drawRect(PLOT_X - 1, PLOT_Y - 1, PLOT_W + 2, PLOT_H + 2, paused ? COLOR_BORDER_PAUSED : COLOR_BORDER); }

#define CURSOR_LABEL_W 64
#define CURSOR_LABEL_H 10

TFT_eSprite cursorSprite = TFT_eSprite(&tft);

void cursorSpriteInit() {
    cursorSprite.setColorDepth(1);
    cursorSprite.createSprite(CURSOR_LABEL_W, CURSOR_LABEL_H);
    cursorSprite.setBitmapColor(COLOR_CURSOR, COLOR_BG);
    cursorSprite.setTextColor(1, 0);
    cursorSprite.setTextSize(1);
    cursorSprite.fillSprite(0);
}

void updateCursorLabel() {
    cursorSprite.fillSprite(0);
    if (cursorEnabled) {
        int32_t offset = PLOT_H / 2 - cursorPixelY;
        int32_t mv = pixelsToMilliVolts(offset);
        char vBuf[10], buf[24];
        formatVolts(vBuf, sizeof(vBuf), mv);
        snprintf(buf, sizeof(buf), "CUR%sV", vBuf);
        int w = cursorSprite.textWidth(buf);
        cursorSprite.setCursor(CURSOR_LABEL_W - w, 0);
        cursorSprite.print(buf);
    }
    cursorSprite.pushSprite(SCREEN_W - UI_MARGIN - CURSOR_LABEL_W, BOTTOM_BAR_Y);
}

void toggleCursor() {
    cursorEnabled = !cursorEnabled;
    updateCursorLabel();
    requestRedraw();
}

void moveCursor(int16_t deltaPx) {
    if (!cursorEnabled) return;
    int32_t y = (int32_t)cursorPixelY + deltaPx;
    if (y < 0) y = 0;
    if (y > PLOT_H - 1) y = PLOT_H - 1;
    cursorPixelY = (int16_t)y;
    updateCursorLabel();
    requestRedraw();
}

void drawUIOverlay() {
    tft.fillRect(0, 0, SCREEN_W, PLOT_Y, COLOR_BG);
    tft.fillRect(0, PLOT_Y + PLOT_H, SCREEN_W, SCREEN_H - (PLOT_Y + PLOT_H), COLOR_BG);
    tft.fillRect(0, PLOT_Y, ARROW_MARGIN, PLOT_H, COLOR_BG);
    tft.fillRect(PLOT_X + PLOT_W, PLOT_Y, ARROW_MARGIN, PLOT_H, COLOR_BG);

    updatePauseBorder();

    tft.setTextColor(COLOR_UI_TEXT, COLOR_BG);
    tft.setTextSize(1);
}

#define ARROW_LENGTH 4
#define ARROW_TAIL_SIZE 5
#define ARROW_BASE_HEIGHT 4

enum ParamIdx {
    PARAM_TRIG_LEVEL,
    PARAM_ZERO,
    PARAM_TIMEBASE,
    PARAM_TRIG_MODE,
    PARAM_RANGE,
    PARAM_SCROLL,
    PARAM_CURSOR,
    PARAM_COUNT
};

int selectedParam = PARAM_TRIG_LEVEL;

const uint16_t COLOR_ZERO_ARROW = TFT_YELLOW;
const uint16_t COLOR_TRIG_ARROW = 0xF9EF;
const uint16_t COLOR_ZERO_ARROW_SEL = TFT_WHITE;
const uint16_t COLOR_TRIG_ARROW_SEL = TFT_WHITE;

#define ZERO_CAL_MIN -1500
#define ZERO_CAL_MAX 1500
#define ZERO_CAL_STEP 4

void setZeroCal(int16_t counts) {
    if (counts < ZERO_CAL_MIN) counts = ZERO_CAL_MIN;
    if (counts > ZERO_CAL_MAX) counts = ZERO_CAL_MAX;
    zeroCalCounts = counts;
    updateTriggerArrow();
    markSettingsDirty();
}

#define AUTO_ZERO_SAMPLE_COUNT 2048

void autoZeroCal() {
    uint32_t base = triggerIdx;
    int64_t sum = 0;
    for (uint32_t i = 0; i < AUTO_ZERO_SAMPLE_COUNT; i++) {
        uint32_t idx = (base + i) % ADC_CAPTURE_SAMPLES;
        sum += adcBuf[idx];
    }
    int32_t avg = (int32_t)(sum / AUTO_ZERO_SAMPLE_COUNT);
    int32_t deviation = avg - ADC_MIDPOINT;
    setZeroCal((int16_t)deviation);
}

void drawZeroLineArrow() {
    int y = PLOT_Y + PLOT_H / 2;
    int tipX = PLOT_X - 2;
    int baseX = tipX - ARROW_LENGTH;
    int tailX = baseX - ARROW_TAIL_SIZE;
    uint16_t color = (selectedParam == PARAM_ZERO) ? COLOR_ZERO_ARROW_SEL : COLOR_ZERO_ARROW;
    tft.fillTriangle(tipX, y, baseX, y - ARROW_BASE_HEIGHT / 2, baseX, y + ARROW_BASE_HEIGHT / 2, color);
    tft.fillRect(tailX, y - ARROW_TAIL_SIZE / 2, ARROW_TAIL_SIZE, ARROW_TAIL_SIZE, color);
}

void updateTriggerArrow() {
    tft.fillRect(PLOT_X + PLOT_W + 1, PLOT_Y, ARROW_MARGIN - 1, PLOT_H, COLOR_BG);
    int y = PLOT_Y + PLOT_H / 2 - clampOffset(triggerThresholdCounts);
    int tipX = PLOT_X + PLOT_W + 2;
    int baseX = tipX + ARROW_LENGTH;
    int tailX = baseX;
    uint16_t color = (selectedParam == PARAM_TRIG_LEVEL) ? COLOR_TRIG_ARROW_SEL : COLOR_TRIG_ARROW;
    tft.fillTriangle(tipX, y, baseX, y - ARROW_BASE_HEIGHT / 2, baseX, y + ARROW_BASE_HEIGHT / 2, color);
    tft.fillRect(tailX, y - ARROW_TAIL_SIZE / 2, ARROW_TAIL_SIZE, ARROW_TAIL_SIZE, color);
}

void squareWaveSetup() {
    HardwareTimer *sqTimer = new HardwareTimer(TIM4);
    sqTimer->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PB6);
    sqTimer->setOverflow(100000, HERTZ_FORMAT);
    sqTimer->setCaptureCompare(1, 50, PERCENT_COMPARE_FORMAT);
    sqTimer->resume();
}

#define SAMPLE_RATE_HZ I2S_AUDIOFREQ_96K
#define DMA_HALF_FRAMES 256u
#define DMA_FULL_FRAMES (DMA_HALF_FRAMES * 2u)
#define TABLE_BITS 12
#define TABLE_SIZE (1u << TABLE_BITS)

int16_t sineTable[TABLE_SIZE + 1];

volatile uint32_t phaseAcc = 0;
volatile uint32_t phaseInc = 0;
volatile uint32_t currentFreqHz = 1000;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;
int16_t i2sBuf[DMA_FULL_FRAMES * 2];

inline uint32_t freqToPhaseInc(uint32_t f) { return (uint32_t)(((uint64_t)f << 32) / 96000u); }

void buildSineTable() {
    for (uint32_t i = 0; i < TABLE_SIZE; i++) sineTable[i] = (int16_t)(sinf(2.0f * (float)M_PI * (float)i / TABLE_SIZE) * 32000.0f);
    sineTable[TABLE_SIZE] = sineTable[0];
}

inline void ddsNextSample(int16_t &sine, int16_t &square) {
    uint32_t acc = phaseAcc;
    uint32_t index = acc >> (32 - TABLE_BITS);
    uint32_t frac = (acc >> (32 - TABLE_BITS - 16)) & 0xFFFFu;

    int32_t a = sineTable[index];
    int32_t b = sineTable[index + 1];
    int32_t v = a + (((b - a) * (int32_t)frac) >> 16);

    sine = (int16_t)v;
    square = (acc & 0x80000000u) ? (int16_t)-32000 : (int16_t)32000;

    phaseAcc = acc + phaseInc;
}

void fillDDSBlock(int16_t *dst, uint32_t frames) {
    for (uint32_t i = 0; i < frames; i++) {
        int16_t s, sq;
        ddsNextSample(s, sq);
        dst[i * 2] = s;
        dst[i * 2 + 1] = sq;
    }
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *h) { if (h == &hi2s2) fillDDSBlock(i2sBuf, DMA_HALF_FRAMES); }
extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *h) { if (h == &hi2s2) fillDDSBlock(&i2sBuf[DMA_HALF_FRAMES * 2], DMA_HALF_FRAMES); }
extern "C" void DMA1_Stream4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi2_tx); }

void i2sClockConfig() {
    RCC_PeriphCLKInitTypeDef cfg = {0};
    cfg.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    cfg.PLLI2S.PLLI2SM = 25;
    cfg.PLLI2S.PLLI2SN = 344;
    cfg.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&cfg);
}

void i2sGpioInit() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void i2sDmaInit() {
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi2_tx.Init.Mode = DMA_CIRCULAR;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_spi2_tx);
    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_spi2_tx);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

void i2sInit() {
    __HAL_RCC_SPI2_CLK_ENABLE();
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = SAMPLE_RATE_HZ;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    HAL_I2S_Init(&hi2s2);
}

void ddsStart() {
    phaseAcc = 0;
    phaseInc = freqToPhaseInc(currentFreqHz);
    fillDDSBlock(i2sBuf, DMA_FULL_FRAMES);
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)i2sBuf, DMA_FULL_FRAMES * 2);
}

#define SETTINGS_MAGIC 0x53434F50u
#define SETTINGS_VERSION 1
#define SETTINGS_SAVE_DEBOUNCE_MS 800

struct __attribute__((packed)) PersistedSettings {
    uint32_t magic;
    uint16_t version;
    int16_t  timebaseLevel;
    uint16_t triggerThresholdCounts;
    uint8_t  triggerMode;
    uint8_t  adcSpeedMode;
    uint32_t currentFreqHz;
    int16_t  zeroCalCounts;
    uint16_t crc;
};

static uint16_t settingsCrc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) { crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1); }
    }
    return crc;
}

volatile bool settingsDirty = false;
volatile uint32_t settingsDirtyMs = 0;

void markSettingsDirty() {
    settingsDirty = true;
    settingsDirtyMs = millis();
}

static void buildSettings(PersistedSettings *s) {
    s->magic = SETTINGS_MAGIC;
    s->version = SETTINGS_VERSION;
    s->timebaseLevel = (int16_t)timebaseLevel;
    s->triggerThresholdCounts = triggerThresholdCounts;
    s->triggerMode = (uint8_t)triggerMode;
    s->adcSpeedMode = (uint8_t)adcSpeedMode;
    s->currentFreqHz = currentFreqHz;
    s->zeroCalCounts = zeroCalCounts;
    s->crc = settingsCrc16((const uint8_t *)s, offsetof(PersistedSettings, crc));
}

#define SETTINGS_FLASH_ADDR 0

void saveSettingsToEEPROM() {
    PersistedSettings s;
    buildSettings(&s);

    PersistedSettings onFlash;
    eeprom_buffer_fill();
    uint8_t *onFlashRaw = (uint8_t *)&onFlash;
    for (size_t i = 0; i < sizeof(onFlash); i++) { onFlashRaw[i] = eeprom_buffered_read_byte(SETTINGS_FLASH_ADDR + i); }
    if (memcmp(&onFlash, &s, sizeof(s)) == 0) return;

    const uint8_t *raw = (const uint8_t *)&s;
    for (size_t i = 0; i < sizeof(s); i++) { eeprom_buffered_write_byte(SETTINGS_FLASH_ADDR + i, raw[i]); }
    eeprom_buffer_flush();
}

bool loadSettingsFromEEPROM() {
    PersistedSettings s;
    eeprom_buffer_fill();
    uint8_t *raw = (uint8_t *)&s;
    for (size_t i = 0; i < sizeof(s); i++) { raw[i] = eeprom_buffered_read_byte(SETTINGS_FLASH_ADDR + i); }

    if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION) return false;
    if (settingsCrc16((const uint8_t *)&s, offsetof(PersistedSettings, crc)) != s.crc) return false;

    timebaseLevel = s.timebaseLevel;
    if (timebaseLevel < 0 || timebaseLevel >= TIMEBASE_LEVELS) timebaseLevel = TIMEBASE_DEFAULT_LEVEL;

    triggerThresholdCounts = s.triggerThresholdCounts;
    if (triggerThresholdCounts > 4095) triggerThresholdCounts = 2048;

    triggerMode = (s.triggerMode <= TRIG_SINGLE) ? (TriggerMode)s.triggerMode : TRIG_AUTO;
    adcSpeedMode = (s.adcSpeedMode == ADC_SPEED_FAST) ? ADC_SPEED_FAST : ADC_SPEED_NORMAL;

    currentFreqHz = s.currentFreqHz;
    if (currentFreqHz == 0 || currentFreqHz > 48000) currentFreqHz = 1000;

    zeroCalCounts = s.zeroCalCounts;
    if (zeroCalCounts < ZERO_CAL_MIN || zeroCalCounts > ZERO_CAL_MAX) zeroCalCounts = 0;

    return true;
}

void trySaveSettings() {
    if (!settingsDirty) return;
    if (millis() - settingsDirtyMs < SETTINGS_SAVE_DEBOUNCE_MS) return;
    settingsDirty = false;
    saveSettingsToEEPROM();
}

static char cmdBuf[16];
static uint8_t cmdLen = 0;
static bool waitingArg = false;

void adjustTimebase(int dir) {
    int newLevel = timebaseLevel + dir;
    if (newLevel < 0) newLevel = 0;
    if (newLevel > TIMEBASE_LEVELS - 1) newLevel = TIMEBASE_LEVELS - 1;
    if (newLevel == timebaseLevel) return;
    timebaseLevel = newLevel;
    updateTimebaseLabel();
    applyTimebaseLevel(timebaseLevel);
    drawScrollbar();
    requestRedraw();
    markSettingsDirty();
}

void adjustScroll(int32_t deltaSamples) {
    int32_t maxOffset = (int32_t)ADC_CAPTURE_SAMPLES - neededSpan(PLOT_W) - 1;
    if (maxOffset < 0) maxOffset = 0;
    scrollOffsetSamples += deltaSamples;
    if (scrollOffsetSamples < 0) scrollOffsetSamples = 0;
    if (scrollOffsetSamples > maxOffset) scrollOffsetSamples = maxOffset;
    requestRedraw();
}

void cycleTriggerMode(int dir) {
    int m = ((int)triggerMode + dir) % 3;
    if (m < 0) m += 3;
    triggerMode = (TriggerMode)m;
    updateTriggerLabel();
    markSettingsDirty();
}

void cycleRange(int dir) {
    int r = (fftRangeIndex + dir) % 4;
    if (r < 0) r += 4;
    fftRangeIndex = r;
    applyFftRange();
    updateRangeLabel();
    requestRedraw();
}

#define ENC_HIGHLIGHT_H 2
#define ENC_BOTTOM_ROW_Y (BOTTOM_BAR_Y + 10)

void updateParamHighlight() {
    tft.fillRect(0, ENC_BOTTOM_ROW_Y, SCREEN_W, ENC_HIGHLIGHT_H, COLOR_BG);

    int tbX = UI_MARGIN + VDIV_LABEL_W + BOTTOM_BAR_GAP + COUPLING_LABEL_W + BOTTOM_BAR_GAP;
    int trigX = tbX + TB_LABEL_W + BOTTOM_BAR_GAP;
    int rangeX = trigX + TRIG_LABEL_W + BOTTOM_BAR_GAP;

    // Zero and trigger level are indicated by recoloring their arrows directly
    // rather than a separate highlight strip, since a static top-row bar
    // didn't correspond to where the actual control (the arrow) lives.
    drawZeroLineArrow();
    updateTriggerArrow();

    switch (selectedParam) {
        case PARAM_TRIG_LEVEL:
        case PARAM_ZERO:
            break;
        case PARAM_TIMEBASE:
            tft.fillRect(tbX, ENC_BOTTOM_ROW_Y, TB_LABEL_W, ENC_HIGHLIGHT_H, TFT_ORANGE);
            break;
        case PARAM_TRIG_MODE:
            tft.fillRect(trigX, ENC_BOTTOM_ROW_Y, TRIG_LABEL_W, ENC_HIGHLIGHT_H, TFT_ORANGE);
            break;
        case PARAM_RANGE:
            tft.fillRect(rangeX, ENC_BOTTOM_ROW_Y, RANGE_LABEL_W, ENC_HIGHLIGHT_H, TFT_ORANGE);
            break;
        case PARAM_SCROLL:
            tft.fillRect(SCROLLBAR_X, ENC_BOTTOM_ROW_Y, SCROLLBAR_W, ENC_HIGHLIGHT_H, TFT_ORANGE);
            break;
        case PARAM_CURSOR:
            tft.fillRect(SCREEN_W - UI_MARGIN - CURSOR_LABEL_W, ENC_BOTTOM_ROW_Y, CURSOR_LABEL_W, ENC_HIGHLIGHT_H, TFT_ORANGE);
            break;
    }
}

void togglePause() {
    paused = !paused;
    if (paused) {
        if (captureMode == CAPTURE_TRIGGERED) {
            HAL_ADC_Stop_DMA(&hadc1);
        } else { HAL_ADC_Stop_IT(&hadc1); }
    } else {
        scrollOffsetSamples = scrollMidOffset();
        if (captureMode == CAPTURE_TRIGGERED) {
            triggerArmed = false;
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuf, ADC_CAPTURE_SAMPLES);
        } else { HAL_ADC_Start_IT(&hadc1); }
        requestRedraw();
    }
    updatePauseBorder();
}

void applyEncoderStep(int dir) {
    switch (selectedParam) {
        case PARAM_TRIG_LEVEL: {
            int32_t counts = (int32_t)triggerThresholdCounts + dir * 16;
            if (counts < 0) counts = 0;
            if (counts > 4095) counts = 4095;
            setTriggerThreshold((uint16_t)counts);
            break;
        }
        case PARAM_ZERO:
            setZeroCal((int16_t)(zeroCalCounts + dir * ZERO_CAL_STEP));
            break;
        case PARAM_TIMEBASE:
            adjustTimebase(dir);
            break;
        case PARAM_TRIG_MODE:
            cycleTriggerMode(dir);
            break;
        case PARAM_RANGE:
            cycleRange(dir);
            break;
        case PARAM_SCROLL:
            adjustScroll((int32_t)dir * SCROLL_STEP_SAMPLES);
            break;
        case PARAM_CURSOR:
            if (cursorEnabled) moveCursor((int16_t)(dir * CURSOR_STEP_PX));
            break;
    }
}

#define ENC_PIN_A PB7
#define ENC_PIN_B PB8

#define ENC_PIN_BTN PB9

#define ENC2_PIN_A PB4
#define ENC2_PIN_B PB5
#define ENC2_PIN_BTN PB6

#define ENC_BTN_DEBOUNCE_MS 250

volatile uint8_t encState = 0;
volatile int8_t encAccum = 0;
volatile int32_t encDelta = 0;
volatile bool encBtnFlag = false;
volatile uint32_t encBtnLastMs = 0;
volatile uint8_t encRawLast = 0xFF;
volatile uint8_t encRawRepeat = 0;

volatile uint8_t encState2 = 0;
volatile int8_t encAccum2 = 0;
volatile int32_t encDelta2 = 0;
volatile bool encBtn2Flag = false;
volatile uint32_t encBtn2LastMs = 0;
volatile uint8_t enc2RawLast = 0xFF;
volatile uint8_t enc2RawRepeat = 0;

// Real switch bounce on cheap rotary encoders commonly rings for 1-5ms, well
// past any sane per-edge debounce window. Reacting to every raw edge and
// trying to time-filter it out is fighting the hardware. Instead this samples
// A/B on a fixed low-rate timer (see encoderTimerInit, TIM4) and only accepts
// a raw 2-bit state once it has read identically for the channel's stable-
// sample count in a row - bounce never gets that many consecutive equal
// readings, so it's rejected structurally rather than by tuning a timeout.
#define ENC_POLL_HZ 1000
// ENC1 (PB7/PB8) measured noisier than ENC2 in testing, so it gets a deeper
// confirmation window; ENC2 (PB4/PB5) stays fast since it's already clean.
#define ENC1_STABLE_SAMPLES 4
#define ENC2_STABLE_SAMPLES 2

const int8_t encTable[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// Diagnostics only - cheap counters to tell apart "pin isn't producing raw
// transitions at all" (wiring/pin problem) from "transitions happen but get
// rejected as illegal" (aliasing - poll rate too slow for this channel) from
// "legal steps happen but rarely reach the +-4 threshold" (noisy/reversing
// contacts). Dumped via the 'y' serial command.
volatile uint32_t encRawChanges = 0, encIllegalCount = 0, encAcceptedSteps = 0;
volatile uint32_t enc2RawChanges = 0, enc2IllegalCount = 0, enc2AcceptedSteps = 0;

static inline void encoderDecodeStable(uint8_t raw, volatile uint8_t &state, volatile int8_t &accum, volatile int32_t &delta,
                                        volatile uint32_t &illegalCount, volatile uint32_t &acceptedSteps) {
    state = ((state << 2) | raw) & 0x0F;
    int8_t step = encTable[state];

    if (step == 0) {
        // Illegal/skipped quadrature transition - genuinely untrustworthy,
        // discard any partial progress.
        illegalCount++;
        accum = 0;
        return;
    }
    // A reversed edge near a detent is normal contact settling, not
    // necessarily bounce - let it partially cancel the accumulator instead
    // of wiping out a turn's worth of progress.
    accum += step;
    if (accum >= 4) {
        delta++;
        acceptedSteps++;
        accum = 0;
    } else if (accum <= -4) {
        delta--;
        acceptedSteps++;
        accum = 0;
    }
}

// Called from TIM4's ISR at ENC_POLL_HZ. One GPIOB->IDR read serves both
// encoders (all four A/B lines are on port B), so this stays cheap even
// running at 1kHz.
void encoderPollTick() {
    uint32_t port = GPIOB->IDR;
    uint8_t raw1 = (uint8_t)(((port >> 7) & 1) << 1 | ((port >> 8) & 1)); // A=PB7,B=PB8
    uint8_t raw2 = (uint8_t)(((port >> 4) & 1) << 1 | ((port >> 5) & 1)); // A=PB4,B=PB5

    if (raw1 == encRawLast) {
        if (encRawRepeat < 0xFF) encRawRepeat++;
    } else {
        encRawLast = raw1;
        encRawRepeat = 1;
        encRawChanges++;
    }
    if (encRawRepeat == ENC1_STABLE_SAMPLES) {
        encRawRepeat = ENC1_STABLE_SAMPLES + 1; // act once per newly-confirmed value
        encoderDecodeStable(raw1, encState, encAccum, encDelta, encIllegalCount, encAcceptedSteps);
    }

    if (raw2 == enc2RawLast) {
        if (enc2RawRepeat < 0xFF) enc2RawRepeat++;
    } else {
        enc2RawLast = raw2;
        enc2RawRepeat = 1;
        enc2RawChanges++;
    }
    if (enc2RawRepeat == ENC2_STABLE_SAMPLES) {
        enc2RawRepeat = ENC2_STABLE_SAMPLES + 1;
        encoderDecodeStable(raw2, encState2, encAccum2, encDelta2, enc2IllegalCount, enc2AcceptedSteps);
    }
}

void encoderBtnISR() {
    uint32_t now = millis();
    if (now - encBtnLastMs > ENC_BTN_DEBOUNCE_MS) {
        encBtnFlag = true;
        encBtnLastMs = now;
    }
}

void encoder2BtnISR() {
    uint32_t now = millis();
    if (now - encBtn2LastMs > ENC_BTN_DEBOUNCE_MS) {
        encBtn2Flag = true;
        encBtn2LastMs = now;
    }
}

void encoderTimerInit();

void encoderInit() {
    pinMode(ENC_PIN_A, INPUT_PULLUP);
    pinMode(ENC_PIN_B, INPUT_PULLUP);
    pinMode(ENC_PIN_BTN, INPUT_PULLUP);

    pinMode(ENC2_PIN_A, INPUT_PULLUP);
    pinMode(ENC2_PIN_B, INPUT_PULLUP);
    pinMode(ENC2_PIN_BTN, INPUT_PULLUP);

    // A/B lines are sampled by encoderPollTick() on TIM4, not by pin-change
    // interrupts - see the comment above ENC_POLL_HZ. Buttons are still
    // low-frequency, genuinely edge-driven events, so those stay on EXTI.
    attachInterrupt(digitalPinToInterrupt(ENC_PIN_BTN), encoderBtnISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENC2_PIN_BTN), encoder2BtnISR, FALLING);

    encoderTimerInit();
}

TIM_HandleTypeDef htim4;

void encoderTimerInit() {
    __HAL_RCC_TIM4_CLK_ENABLE();

    uint32_t timClkHz = apb1TimerClockHz();
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (timClkHz / 1000000u) - 1u; // 1 MHz tick
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = (1000000u / ENC_POLL_HZ) - 1u;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim4);

    HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_TIM_Base_Start_IT(&htim4);
}

extern "C" void TIM4_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
        encoderPollTick();
    }
}

// Guards TIM4 (the encoder poll timer, sole writer of encDelta/encDelta2)
// instead of __disable_irq()/__enable_irq(), which stalls every interrupt in
// the system - ADC DMA completion, the SPI display DMA, and the TIM2
// frequency-capture IRQ all ride on separate NVIC lines and don't need to be
// touched just to read two encoder counters. This also no longer needs to
// touch the button EXTI lines at all, since those are a separate mechanism now.
static inline void encoderIrqGuardBegin() {
    NVIC_DisableIRQ(TIM4_IRQn);
}
static inline void encoderIrqGuardEnd() {
    NVIC_EnableIRQ(TIM4_IRQn);
}

void pollEncoder() {
    if (encBtnFlag) {
        encBtnFlag = false;
        togglePause();
    }

    if (encBtn2Flag) {
        encBtn2Flag = false;
        spectrumMode = !spectrumMode;
        requestRedraw();
    }

    encoderIrqGuardBegin();
    int32_t delta2 = encDelta2;
    encDelta2 = 0;
    int32_t delta = encDelta;
    encDelta = 0;
    encoderIrqGuardEnd();

    if (delta2 != 0) {
        int steps2 = delta2 > 0 ? delta2 : -delta2;
        int dir2 = delta2 > 0 ? 1 : -1;
        for (int i = 0; i < steps2; i++) {
            selectedParam = (selectedParam + dir2 + PARAM_COUNT) % PARAM_COUNT;
        }
        updateParamHighlight();
    }

    if (delta != 0) {
        int steps = delta > 0 ? delta : -delta;
        int dir = delta > 0 ? 1 : -1;
        for (int i = 0; i < steps; i++) { applyEncoderStep(dir); }
    }
}

// Live diagnostic: prints PB4..PB9 whenever any of them changes, so you can
// wiggle one physical control at a time and see exactly which pin reacts -
// no guessing about which header/wire maps to which firmware #define.
// Toggle with the 'i' serial command.
volatile bool pinWatchMode = false;
uint32_t pinWatchLastBits = 0xFFFFFFFF;

void checkPinWatch() {
    if (!pinWatchMode) return;
    uint32_t bits = (GPIOB->IDR >> 4) & 0x3F; // PB4..PB9 packed into bits 0..5
    if (bits == pinWatchLastBits) return;
    pinWatchLastBits = bits;
    Serial.print("PB4="); Serial.print((bits >> 0) & 1);
    Serial.print(" PB5="); Serial.print((bits >> 1) & 1);
    Serial.print(" PB6="); Serial.print((bits >> 2) & 1);
    Serial.print(" PB7="); Serial.print((bits >> 3) & 1);
    Serial.print(" PB8="); Serial.print((bits >> 4) & 1);
    Serial.print(" PB9="); Serial.println((bits >> 5) & 1);
}

void handleSerialCommand() {
    while (Serial.available()) {
        char c = Serial.read();

        if (!waitingArg && cmdLen == 0) {
            switch (c) {
                case 'z':
                    adjustTimebase(1);
                    break;
                case 'x':
                    adjustTimebase(-1);
                    break;
                case 's':
                    adcSetSpeedMode(adcSpeedMode == ADC_SPEED_FAST ? ADC_SPEED_NORMAL : ADC_SPEED_FAST);
                    applyTimebaseLevel(timebaseLevel);
                    drawScrollbar();
                    requestRedraw();
                    markSettingsDirty();
                    break;
                case 'a':
                    cycleTriggerMode(1);
                    break;
                case 'p':
                    togglePause();
                    break;
                case 'w':
                    spectrumMode = !spectrumMode;
                    requestRedraw();
                    break;
                case 'r':
                    cycleRange(1);
                    break;
                case 'n':
                    adjustScroll(-SCROLL_STEP_SAMPLES);
                    break;
                case 'm':
                    adjustScroll(SCROLL_STEP_SAMPLES);
                    break;
                case 'c':
                    setZeroCal(zeroCalCounts - ZERO_CAL_STEP);
                    break;
                case 'v':
                    setZeroCal(zeroCalCounts + ZERO_CAL_STEP);
                    break;
                case 'f':
                    cmdBuf[0] = 'f';
                    cmdLen = 1;
                    waitingArg = true;
                    break;
                case 't':
                    cmdBuf[0] = 't';
                    cmdLen = 1;
                    waitingArg = true;
                    break;
                case 'o':
                    autoZeroCal();
                    break;
                case 'u':
                    toggleCursor();
                    break;
                case 'k':
                    moveCursor(-CURSOR_STEP_PX);
                    break;
                case 'j':
                    moveCursor(CURSOR_STEP_PX);
                    break;
                case 'i':
                    pinWatchMode = !pinWatchMode;
                    Serial.println(pinWatchMode ? "pin watch ON" : "pin watch OFF");
                    break;
                case 'y': {
                    uint32_t r1, i1, a1, r2, i2, a2;
                    NVIC_DisableIRQ(TIM4_IRQn);
                    r1 = encRawChanges; i1 = encIllegalCount; a1 = encAcceptedSteps;
                    r2 = enc2RawChanges; i2 = enc2IllegalCount; a2 = enc2AcceptedSteps;
                    NVIC_EnableIRQ(TIM4_IRQn);
                    Serial.print("ENC1 raw="); Serial.print(r1);
                    Serial.print(" illegal="); Serial.print(i1);
                    Serial.print(" accepted="); Serial.println(a1);
                    Serial.print("ENC2 raw="); Serial.print(r2);
                    Serial.print(" illegal="); Serial.print(i2);
                    Serial.print(" accepted="); Serial.println(a2);
                    break;
                }
            }
            continue;
        }
        if (waitingArg) {
            if (c == '\n' || c == '\r') {
                cmdBuf[cmdLen] = 0;
                if (cmdLen > 1) {
                    if (cmdBuf[0] == 'f') {
                        uint32_t freq = strtoul(&cmdBuf[1], nullptr, 10);
                        if (freq > 0 && freq <= 48000) {
                            currentFreqHz = freq;
                            phaseInc = freqToPhaseInc(currentFreqHz);
                            markSettingsDirty();
                        }
                    } else if (cmdBuf[0] == 't') {
                        uint32_t counts = strtoul(&cmdBuf[1], nullptr, 10);
                        setTriggerThreshold((uint16_t)counts);
                    }
                }
                cmdLen = 0;
                waitingArg = false;
                continue;
            }

            if (c >= '0' && c <= '9') if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
            else cmdLen = 0, waitingArg = false;
        }
    }
}

void SystemClock_Config() {
    RCC_OscInitTypeDef oscConfig = {0};
    RCC_ClkInitTypeDef clockConfig = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    oscConfig.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscConfig.HSEState = RCC_HSE_ON;
    oscConfig.PLL.PLLState = RCC_PLL_ON;
    oscConfig.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscConfig.PLL.PLLM = 25;
    oscConfig.PLL.PLLN = 288;
    oscConfig.PLL.PLLP = RCC_PLLP_DIV2;
    oscConfig.PLL.PLLQ = 6;

    HAL_RCC_OscConfig(&oscConfig);

    clockConfig.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clockConfig.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clockConfig.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clockConfig.APB1CLKDivider = RCC_HCLK_DIV2;
    clockConfig.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&clockConfig, FLASH_LATENCY_6);
    SystemCoreClockUpdate();
}

void setup() {
    buildSineTable();

    loadSettingsFromEEPROM();

    i2sClockConfig();
    i2sGpioInit();
    i2sInit();
    i2sDmaInit();
    ddsStart();

    Serial.begin(115200);
    tft.init();
    tft.setRotation(3);
    spiDmaSetup();
    tft.fillScreen(COLOR_BG);
    drawUIOverlay();
    drawZeroLineArrow();
    vdivSpriteInit();
    updateVoltsPerDivLabel();
    couplingSpriteInit();
    updateCouplingLabel();
    tbSpriteInit();
    updateTimebaseLabel();
    trigSpriteInit();
    rangeSpriteInit();
    freqSpriteInit();
    measSpriteInit();
    cursorSpriteInit();
    updateCursorLabel();
    fftInfoSpriteInit();
    scrollbarSpriteInit();
    initGridLookup();

    triggerTimerInit();
    triggerThresholdPwmInit();
    updateTriggerArrow();
    rollTimerInit();
    adcInit();
    adcDmaInit();
    adcItInit();
    adcSetSpeedMode(adcSpeedMode);
    interpFiltersInit();
    fftInit();
    applyFftRange();
    updateRangeLabel();
    drawScrollbar();
    lastTriggerMs = millis();
    applyTimebaseLevel(timebaseLevel);
    updateTriggerLabel();
    // squareWaveSetup(); // disabled for now
    encoderInit();
    updateParamHighlight();
}

#define FRAME_INTERVAL_MS 33

uint32_t lastFrameMs = 0;

void checkAutoTrigger() {
    if (captureMode != CAPTURE_TRIGGERED) return;
    if (triggerMode != TRIG_AUTO) return;
    uint32_t now = millis();
    if (now - lastTriggerMs > AUTO_TRIGGER_TIMEOUT_MS) {
        forceTrigger();
        lastTriggerMs = now;
    }
}

void loop() {
    handleSerialCommand();
    checkPinWatch();
    trySaveSettings();
    pollEncoder();

    if (!paused) {
        checkAutoTrigger();
        captureTriggeredPoll();
        updateFreqMeasurement();
    }

    if (!needsRedraw) return;

    uint32_t now = millis();
    if (now - lastFrameMs < FRAME_INTERVAL_MS) return;

    needsRedraw = false;
    lastFrameMs = now;

    generateWaveform(waveformSamples, PLOT_W);
#if !RAW_ADC_MODE
    smoothWaveformSpatial(waveformSamples, PLOT_W);
#endif

    uint32_t fftBase = (viewAnchorIdx + (uint32_t)scrollOffsetSamples) % ADC_CAPTURE_SAMPLES;
    computeFFT(adcBuf, fftBase);
    computeSpectrumColumns();
    pickDrawnPeaks();

    updateFreqLabel();
    updateMeasLabel();
    clearPlotBuffer();
    drawWaveformIntoBuffer(waveformSamples, PLOT_W);
    pushPlotBufferDMA();
    if (spectrumMode) { drawSpectrumPoints(); }

    updateFftInfoLabel();
    drawScrollbar();

    if (pauseRequested) {
        pauseRequested = false;
        paused = true;
        if (captureMode == CAPTURE_TRIGGERED) { HAL_ADC_Stop_DMA(&hadc1); }
    }
    updatePauseBorder();
}