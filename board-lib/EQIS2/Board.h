#pragma once

#include <Arduino.h>

#include "SSD1306Display.h"

// グループ化するサンプル数 (* 10ms = 計測震度の計算間隔)
#define INTENSITY_PROCESS_SAMPLE_GROUP_SIZE 20

#include "IntensityProcessor.h"

// main.cpp のシリアルコマンドで使用する ADC ステップあたりの gal
#define SEISMOMETER_ADC_STEP (sensor.getSensitivity())

void printNmea(const char *format, ...);
void printErrorNmea(const char *id);
void serialCommandTask(void *pvParameters);

IntensityProcessor *processor;
QueueHandle_t displayIntensityQueue;

// XIAO ESP32C3: SPI は SCK=D8/GPIO8, MISO=D9/GPIO9, MOSI=D10/GPIO10, CS=D3/GPIO5
// ※variant の SS(GPIO20) は UART RX と重複しているため使わない
// センサーは platformio.ini の SEISMOMETER_SENSOR_* マクロで選択する
#if defined(SEISMOMETER_SENSOR_ADXL345)
#include "ADXL345.h"
auto sensor = ADXL345(&SPI, D3);
#elif defined(SEISMOMETER_SENSOR_ADXL355)
#include "ADXL355.h"
auto sensor = ADXL355(&SPI, D3);
#else
#error "platformio.ini で SEISMOMETER_SENSOR_ADXL355 または SEISMOMETER_SENSOR_ADXL345 を定義してください"
#endif
void measureTask(void *pvParameters) {
    auto xLastWakeTime = xTaskGetTickCount();
    int16_t offsetRawData[3];
    int16_t rawData[3];
    float sample[3];
    sensor.begin();

    // 収束の高速化のために初回の値をオフセット値として保存する
    sensor.read(offsetRawData);
    while (1) {
        sensor.read(rawData);
        printNmea("XSRAW,%d,%d,%d", rawData[0], rawData[1], rawData[2]);

        for (auto i = 0; i < 3; i++)
            sample[i] = ((float)offsetRawData[i] - rawData[i]) * sensor.getSensitivity();

        processor->process(sample);

        // 100Hz で動かす
        if (!xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 100)) {
            printErrorNmea("MEASURE_DROPPED");
        }
    }
}

// I2C デフォルトピンを使用 (XIAO ESP32C3: SDA=D4/GPIO6, SCL=D5/GPIO7)
auto display = SSD1306Display();
void oledDisplayTask(void *pvParameters) {
    auto xLastWakeTime = xTaskGetTickCount();
    float rawInt = NAN;

    JmaIntensity maxIntensity = JMA_INT_0;
    ulong maxIntensityAt = millis();

    display.begin();
    display.wakeup();
    vTaskDelay(configTICK_RATE_HZ * 3);

    while (1) {
        // 100Hz で動かす
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 100);

        // キューから値を取り出す
        if (xQueueReceive(displayIntensityQueue, &rawInt, 0) == pdFALSE && rawInt == NAN)
            continue;

        if (!processor->getIsStable()) {
            display.stabilityAnimate(rawInt, processor->calcStdDev());
            continue;
        }

        auto latestIntensity = getJmaIntensity(rawInt);
        if (millis() - maxIntensityAt > (ulong)displayConfig.resetMinutes * 60UL * 1000UL || maxIntensity <= latestIntensity) {
            maxIntensity = latestIntensity;
            maxIntensityAt = millis();
        }

        bool showCurrent = latestIntensity >= (JmaIntensity)displayConfig.currentThreshold;
        bool showMax = maxIntensity > JMA_INT_0 && maxIntensity >= (JmaIntensity)displayConfig.maxThreshold;
        // 常時表示をすると OLED の寿命が溶けるので動きがない場合は消灯させる
        bool hideDisplay = processor->calcStdDev() <= 0.05;
        bool effectiveHide = hideDisplay || !showCurrent;

        display.displayIntensity(
            latestIntensity, rawInt,
            effectiveHide,
            maxIntensity,
            showMax && (effectiveHide || maxIntensity != latestIntensity)
        );
    }
}

void setup() {
    Serial.begin(115200);
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
    // HWCDC はホストがモニタを閉じても切断を検出できず、TXバッファ満杯時に
    // write が最大 100ms ブロックして 100Hz 計測を乱す (docs/esp32-design-issues.md 問題2)。
    // タイムアウト 0 で書けない分は捨てるノンブロッキング動作にする
    Serial.setTxTimeoutMs(0);
#endif
    loadDisplayConfig();

    displayIntensityQueue = xQueueCreate(1, sizeof(float));
    processor = new IntensityProcessor([](float sample[3]) {
        printNmea("XSACC,%.3f,%.3f,%.3f", sample[0], sample[1], sample[2]);
    }, [](float rawInt) {
        printNmea("XSINT,%.3f,%.2f", -1.0, processor->getIsStable() ? rawInt : NAN);
        xQueueOverwrite(displayIntensityQueue, &rawInt);
    });

    // ESP-IDF のスタック指定はバイト単位 (docs/esp32-design-issues.md 問題1)。
    // コア番号はシングルコア (C3) では 0 しか存在しない (同 問題3)
#if CONFIG_FREERTOS_UNICORE
#define SEISMOMETER_TASK_CORE 0
#else
#define SEISMOMETER_TASK_CORE 1
#endif
    xTaskCreatePinnedToCore(measureTask, "Measure", 8192, NULL, 10, NULL, SEISMOMETER_TASK_CORE);
    xTaskCreatePinnedToCore(oledDisplayTask, "OLEDDisplay", 8192, NULL, 5, NULL, SEISMOMETER_TASK_CORE);
    xTaskCreatePinnedToCore(serialCommandTask, "Serial", 8192, NULL, 5, NULL, SEISMOMETER_TASK_CORE);

    vTaskDelete(NULL);  /* delete loopTask. */
}

void loop() {
}
