#include <driver/i2s_std.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include <HOPE_Sound_Classification_inferencing.h>
#include "direction_estimate.h"

// I2S0가 BCLK/WS를 만들고 I2S1은 같은 클럭을 입력받는다.
// 반드시 GPIO15-BCLK를 GPIO4에, GPIO16-WS를 GPIO5에 연결해야 한다.
constexpr gpio_num_t I2S0_BCLK_OUT_PIN = GPIO_NUM_15;
constexpr gpio_num_t I2S0_WS_OUT_PIN = GPIO_NUM_16;
constexpr gpio_num_t I2S0_SD_IN_PIN = GPIO_NUM_17;

constexpr gpio_num_t I2S1_BCLK_IN_PIN = GPIO_NUM_4;
constexpr gpio_num_t I2S1_WS_IN_PIN = GPIO_NUM_5;
constexpr gpio_num_t I2S1_SD_IN_PIN = GPIO_NUM_6;

constexpr uint32_t SAMPLE_RATE = 16000;
constexpr size_t DMA_BUFFER_COUNT = 8;
constexpr size_t DMA_FRAME_COUNT = 256;
constexpr size_t CHANNEL_COUNT = 2;
constexpr size_t I2S_VALUES_PER_BLOCK = DMA_FRAME_COUNT * CHANNEL_COUNT;
constexpr size_t I2S_BYTES_PER_FRAME = CHANNEL_COUNT * sizeof(int32_t);
constexpr uint32_t I2S_READ_TIMEOUT_MS = 100;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 500;

static_assert(
    EI_CLASSIFIER_FREQUENCY == SAMPLE_RATE,
    "Edge Impulse model frequency and I2S sample rate do not match."
);

static_assert(
    GCC_PHAT_N_BUFFER % DMA_FRAME_COUNT == 0,
    "GCC-PHAT buffer must be a multiple of the DMA frame count."
);

// I2S DMA에서 읽는 원시 스테레오 버퍼
static int32_t i2s0RawBuffer[I2S_VALUES_PER_BLOCK];
static int32_t i2s1RawBuffer[I2S_VALUES_PER_BLOCK];
static i2s_chan_handle_t i2s0RxChannel = nullptr;
static i2s_chan_handle_t i2s1RxChannel = nullptr;

// 마이크별 signed 24-bit PCM 버퍼
static int32_t mic1Buffer[DMA_FRAME_COUNT];
static int32_t mic2Buffer[DMA_FRAME_COUNT];
static int32_t mic3Buffer[DMA_FRAME_COUNT];
static int32_t mic4Buffer[DMA_FRAME_COUNT];

// GCC-PHAT 입력 버퍼
static float directionMic1[GCC_PHAT_N_BUFFER];
static float directionMic2[GCC_PHAT_N_BUFFER];
static float directionMic3[GCC_PHAT_N_BUFFER];
static float directionMic4[GCC_PHAT_N_BUFFER];
static size_t directionSampleCount = 0;

// 16,000샘플 분류 버퍼는 내부 RAM을 아끼기 위해 PSRAM에 할당한다.
static float *classifierBuffer = nullptr;
static size_t classifierSampleCount = 0;

static uint32_t lastDebugPrintTime = 0;
static const char *TAG = "hope_audio";

void stopWithError(const char *message, esp_err_t errorCode)
{
    ESP_LOGE(TAG, "%s: %s", message, esp_err_to_name(errorCode));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void checkEspError(esp_err_t errorCode, const char *message)
{
    if (errorCode != ESP_OK) {
        stopWithError(message, errorCode);
    }
}

void installI2SRxChannel(
    int port,
    i2s_role_t role,
    gpio_num_t bclkPin,
    gpio_num_t wsPin,
    gpio_num_t dataInPin,
    i2s_chan_handle_t &rxChannel
)
{
    i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(port, role);
    channelConfig.dma_desc_num = DMA_BUFFER_COUNT;
    channelConfig.dma_frame_num = DMA_FRAME_COUNT;

    checkEspError(
        i2s_new_channel(&channelConfig, nullptr, &rxChannel),
        port == I2S_NUM_0
            ? "I2S0 RX channel creation failed"
            : "I2S1 RX channel creation failed"
    );

    i2s_std_config_t standardConfig = {};
    standardConfig.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
    standardConfig.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_STEREO
    );
    standardConfig.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    standardConfig.gpio_cfg.bclk = bclkPin;
    standardConfig.gpio_cfg.ws = wsPin;
    standardConfig.gpio_cfg.dout = I2S_GPIO_UNUSED;
    standardConfig.gpio_cfg.din = dataInPin;
    standardConfig.gpio_cfg.invert_flags.mclk_inv = false;
    standardConfig.gpio_cfg.invert_flags.bclk_inv = false;
    standardConfig.gpio_cfg.invert_flags.ws_inv = false;

    checkEspError(
        i2s_channel_init_std_mode(rxChannel, &standardConfig),
        port == I2S_NUM_0
            ? "I2S0 standard mode initialization failed"
            : "I2S1 standard mode initialization failed"
    );
}

bool startSynchronizedI2S()
{
    // 클럭을 기다리는 Slave를 먼저 시작한 후 Master 클럭을 출력한다.
    esp_err_t error = i2s_channel_enable(i2s1RxChannel);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "I2S1 start failed: %s", esp_err_to_name(error));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(2));

    error = i2s_channel_enable(i2s0RxChannel);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "I2S0 start failed: %s", esp_err_to_name(error));
        i2s_channel_disable(i2s1RxChannel);
        return false;
    }

    return true;
}

bool restartSynchronizedI2S()
{
    ESP_LOGW(TAG, "Restarting synchronized I2S...");

    i2s_channel_disable(i2s0RxChannel);
    i2s_channel_disable(i2s1RxChannel);

    directionSampleCount = 0;
    classifierSampleCount = 0;

    bool started = startSynchronizedI2S();
    if (started) {
        ESP_LOGI(TAG, "I2S restarted");
    }
    else {
        ESP_LOGE(TAG, "I2S restart failed");
    }
    return started;
}

void setupSynchronizedI2S()
{
    installI2SRxChannel(
        I2S_NUM_1,
        I2S_ROLE_SLAVE,
        I2S1_BCLK_IN_PIN,
        I2S1_WS_IN_PIN,
        I2S1_SD_IN_PIN,
        i2s1RxChannel
    );

    installI2SRxChannel(
        I2S_NUM_0,
        I2S_ROLE_MASTER,
        I2S0_BCLK_OUT_PIN,
        I2S0_WS_OUT_PIN,
        I2S0_SD_IN_PIN,
        i2s0RxChannel
    );

    if (!startSynchronizedI2S()) {
        stopWithError("Synchronized I2S start failed", ESP_FAIL);
    }

    ESP_LOGI(TAG, "Common-clock I2S started");
}

bool readSynchronizedBlock(size_t &framesRead)
{
    framesRead = 0;
    size_t bytesRead0 = 0;
    size_t bytesRead1 = 0;

    esp_err_t error0 = i2s_channel_read(
        i2s0RxChannel,
        i2s0RawBuffer,
        sizeof(i2s0RawBuffer),
        &bytesRead0,
        I2S_READ_TIMEOUT_MS
    );

    esp_err_t error1 = i2s_channel_read(
        i2s1RxChannel,
        i2s1RawBuffer,
        sizeof(i2s1RawBuffer),
        &bytesRead1,
        I2S_READ_TIMEOUT_MS
    );

    if (error0 != ESP_OK || error1 != ESP_OK) {
        ESP_LOGE(
            TAG,
            "I2S read failed: port0=%s, port1=%s",
            esp_err_to_name(error0),
            esp_err_to_name(error1)
        );
        restartSynchronizedI2S();
        return false;
    }

    if (
        bytesRead0 == 0 ||
        bytesRead0 != bytesRead1 ||
        bytesRead0 % I2S_BYTES_PER_FRAME != 0
    ) {
        ESP_LOGW(
            TAG,
            "I2S block mismatch: port0=%u, port1=%u",
            static_cast<unsigned>(bytesRead0),
            static_cast<unsigned>(bytesRead1)
        );
        restartSynchronizedI2S();
        return false;
    }

    framesRead = bytesRead0 / I2S_BYTES_PER_FRAME;
    if (framesRead > DMA_FRAME_COUNT) {
        framesRead = DMA_FRAME_COUNT;
    }

    return framesRead > 0;
}

void deinterleaveMicrophones(size_t frameCount)
{
    for (size_t i = 0; i < frameCount; ++i) {
        // INMP441의 signed 24-bit 데이터는 32-bit 슬롯의 상위 비트에 있다.
        mic1Buffer[i] = i2s0RawBuffer[i * 2] >> 8;
        mic2Buffer[i] = i2s0RawBuffer[i * 2 + 1] >> 8;
        mic3Buffer[i] = i2s1RawBuffer[i * 2] >> 8;
        mic4Buffer[i] = i2s1RawBuffer[i * 2 + 1] >> 8;
    }
}

float pcm24ToFloat(int32_t sample)
{
    constexpr float PCM24_SCALE = 8388608.0f;
    return static_cast<float>(sample) / PCM24_SCALE;
}

const char *directionName(Direction direction)
{
    switch (direction) {
        case RIGHT: return "RIGHT";
        case FRONT: return "FRONT";
        case LEFT: return "LEFT";
        case BACK: return "BACK";
        default: return "UNKNOWN";
    }
}

void runDirectionEstimate()
{
    // 배치 가정: mic1=LEFT, mic2=RIGHT, mic3=BACK, mic4=FRONT.
    // 실제 배치가 다르면 두 함수 호출의 마이크 순서를 바꿔야 한다.
    float tauX = gcc_phat_compute(directionMic1, directionMic2);
    float tauY = gcc_phat_compute(directionMic3, directionMic4);
    float angle = compute_TDOA(tauX, tauY);
    Direction direction = get_direction(angle);

    ESP_LOGI(
        TAG,
        "DOA tauX=%.8f, tauY=%.8f, angle=%.1f deg, direction=%s",
        tauX,
        tauY,
        angle * 180.0f / static_cast<float>(M_PI),
        directionName(direction)
    );
}

void processDirectionBlock(size_t frameCount)
{
    size_t sourceIndex = 0;

    while (sourceIndex < frameCount) {
        size_t freeCount = GCC_PHAT_N_BUFFER - directionSampleCount;
        size_t remaining = frameCount - sourceIndex;
        size_t copyCount = remaining < freeCount ? remaining : freeCount;

        for (size_t i = 0; i < copyCount; ++i) {
            size_t source = sourceIndex + i;
            size_t destination = directionSampleCount + i;

            directionMic1[destination] = pcm24ToFloat(mic1Buffer[source]);
            directionMic2[destination] = pcm24ToFloat(mic2Buffer[source]);
            directionMic3[destination] = pcm24ToFloat(mic3Buffer[source]);
            directionMic4[destination] = pcm24ToFloat(mic4Buffer[source]);
        }

        sourceIndex += copyCount;
        directionSampleCount += copyCount;

        if (directionSampleCount == GCC_PHAT_N_BUFFER) {
            runDirectionEstimate();
            directionSampleCount = 0;
        }
    }
}

int classifierGetData(size_t offset, size_t length, float *output)
{
    if (
        classifierBuffer == nullptr ||
        output == nullptr ||
        offset + length > EI_CLASSIFIER_RAW_SAMPLE_COUNT
    ) {
        return -1;
    }

    memcpy(output, classifierBuffer + offset, length * sizeof(float));
    return 0;
}

void runSoundClassifier()
{
    signal_t signal = {};
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = classifierGetData;

    ei_impulse_result_t result = {};
    EI_IMPULSE_ERROR error = run_classifier(&signal, &result, false);

    if (error != EI_IMPULSE_OK) {
        ESP_LOGE(TAG, "Edge Impulse inference failed: %d", static_cast<int>(error));
        return;
    }

    ESP_LOGI(TAG, "Classifier result:");
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; ++i) {
        ESP_LOGI(
            TAG,
            "%s: %.5f",
            result.classification[i].label,
            result.classification[i].value
        );
    }
}

void processClassifierBlock(size_t frameCount)
{
    if (classifierBuffer == nullptr) {
        return;
    }

    size_t sourceIndex = 0;

    while (sourceIndex < frameCount) {
        size_t freeCount = EI_CLASSIFIER_RAW_SAMPLE_COUNT - classifierSampleCount;
        size_t remaining = frameCount - sourceIndex;
        size_t copyCount = remaining < freeCount ? remaining : freeCount;

        for (size_t i = 0; i < copyCount; ++i) {
            // Edge Impulse 오디오 입력에는 signed 16-bit 크기의 값을 float로 전달한다.
            classifierBuffer[classifierSampleCount + i] =
                static_cast<float>(mic1Buffer[sourceIndex + i] >> 8);
        }

        sourceIndex += copyCount;
        classifierSampleCount += copyCount;

        if (classifierSampleCount == EI_CLASSIFIER_RAW_SAMPLE_COUNT) {
            runSoundClassifier();
            classifierSampleCount = 0;
        }
    }
}

double calculateRms(const int32_t *samples, size_t sampleCount)
{
    if (samples == nullptr || sampleCount == 0) {
        return 0.0;
    }

    double sumSquares = 0.0;
    for (size_t i = 0; i < sampleCount; ++i) {
        double value = static_cast<double>(samples[i]);
        sumSquares += value * value;
    }

    return sqrt(sumSquares / static_cast<double>(sampleCount));
}

void printAudioStatus(size_t frameCount)
{
    uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    if (now - lastDebugPrintTime < DEBUG_PRINT_INTERVAL_MS) {
        return;
    }

    lastDebugPrintTime = now;

    ESP_LOGI(
        TAG,
        "RMS frames=%u | mic1=%.0f, mic2=%.0f, mic3=%.0f, mic4=%.0f",
        static_cast<unsigned>(frameCount),
        calculateRms(mic1Buffer, frameCount),
        calculateRms(mic2Buffer, frameCount),
        calculateRms(mic3Buffer, frameCount),
        calculateRms(mic4Buffer, frameCount)
    );
}

void discardInitialBlocks(size_t blockCount)
{
    for (size_t block = 0; block < blockCount; ++block) {
        size_t framesRead = 0;
        if (!readSynchronizedBlock(framesRead)) {
            ESP_LOGW(
                TAG,
                "Initial block %u failed",
                static_cast<unsigned>(block)
            );
        }
    }
}

void setupClassifierBuffer()
{
    size_t psramSize = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psramSize == 0) {
        stopWithError("PSRAM was not detected", ESP_ERR_NOT_FOUND);
    }

    classifierBuffer = static_cast<float *>(
        heap_caps_malloc(
            EI_CLASSIFIER_RAW_SAMPLE_COUNT * sizeof(float),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        )
    );

    if (classifierBuffer == nullptr) {
        stopWithError("Failed to allocate classifier buffer in PSRAM", ESP_ERR_NO_MEM);
    }

    ESP_LOGI(
        TAG,
        "PSRAM=%u bytes, classifier buffer=%u bytes",
        static_cast<unsigned>(psramSize),
        static_cast<unsigned>(EI_CLASSIFIER_RAW_SAMPLE_COUNT * sizeof(float))
    );
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 synchronized 4-microphone system");
    ESP_LOGI(TAG, "Sample rate: %u Hz", static_cast<unsigned>(SAMPLE_RATE));

    setupClassifierBuffer();

    esp_err_t dspError = gcc_phat_init();
    if (dspError != ESP_OK) {
        stopWithError("GCC-PHAT initialization failed", dspError);
    }
    ESP_LOGI(TAG, "GCC-PHAT initialized");

    setupSynchronizedI2S();
    discardInitialBlocks(4);
    ESP_LOGI(TAG, "Audio capture started");

    while (true) {
        size_t framesRead = 0;

        if (!readSynchronizedBlock(framesRead)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        deinterleaveMicrophones(framesRead);
        processDirectionBlock(framesRead);
        processClassifierBlock(framesRead);
        printAudioStatus(framesRead);
    }
}
