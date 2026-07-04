#pragma once

#include <Arduino.h>
#include <SPI.h>

// ADXL355 レジスタ直接制御ドライバ (SPI モード0, 最大10MHz)
class ADXL355 {
    // レジスタアドレス (データシート Rev.D Table 15)
    static constexpr uint8_t REG_PARTID = 0x02;      // = 0xED
    static constexpr uint8_t REG_XDATA3 = 0x08;      // XDATA3..ZDATA1 の9バイト連続読み出し起点
    static constexpr uint8_t REG_FILTER = 0x28;
    static constexpr uint8_t REG_RANGE = 0x2C;
    static constexpr uint8_t REG_POWER_CTL = 0x2D;
    static constexpr uint8_t REG_RESET = 0x2F;

    static constexpr uint8_t PARTID_VALUE = 0xED;
    static constexpr uint8_t RESET_CODE = 0x52;

    // ODR_LPF = 0b0101: ODR 125Hz / LPF 31.25Hz
    // 100Hz で読み出すため LPF がナイキスト周波数 (50Hz) 未満になる最速の設定を選ぶ
    static constexpr uint8_t FILTER_ODR_125HZ = 0x05;

    // ±2g レンジ: 256,000 LSB/g (20bit)。16bit へ切り詰めるため 1/16 の 16,000 LSB/g
    static constexpr float SENSITIVITY_GAL_PER_LSB = 980.665f / 16000.0f;

    SPIClass *spi;
    int csPin;
    SPISettings spiSettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);

    // コマンドバイトは 7bit アドレス << 1 | R/W (1 = read)
    uint8_t readRegister(uint8_t reg) {
        this->spi->beginTransaction(this->spiSettings);
        digitalWrite(this->csPin, LOW);
        this->spi->transfer((reg << 1) | 0x01);
        auto value = this->spi->transfer(0x00);
        digitalWrite(this->csPin, HIGH);
        this->spi->endTransaction();
        return value;
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        this->spi->beginTransaction(this->spiSettings);
        digitalWrite(this->csPin, LOW);
        this->spi->transfer(reg << 1);
        this->spi->transfer(value);
        digitalWrite(this->csPin, HIGH);
        this->spi->endTransaction();
    }

public:
    ADXL355(SPIClass *spi = &SPI, int csPin = SS) : spi(spi), csPin(csPin) {}

    void begin() {
        pinMode(this->csPin, OUTPUT);
        digitalWrite(this->csPin, HIGH);
        this->spi->begin();

        // ソフトリセット (POR 相当) 後、デバイス応答を待つ
        this->writeRegister(REG_RESET, RESET_CODE);
        delay(10);
        while (this->readRegister(REG_PARTID) != PARTID_VALUE)
            delay(1);

        // 計測モードへ入る前に全設定レジスタを書き込む必要がある (データシート Register Map 冒頭注記)
        this->writeRegister(REG_RANGE, (this->readRegister(REG_RANGE) & ~0x03) | 0x01);  // ±2g
        this->writeRegister(REG_FILTER, FILTER_ODR_125HZ);
        this->writeRegister(REG_POWER_CTL, this->readRegister(REG_POWER_CTL) & ~0x01);  // Standby 解除
    }

    float getSensitivity() {
        return SENSITIVITY_GAL_PER_LSB;
    }

    void read(int16_t (&data)[3])
    {
        uint8_t buffer[9];

        this->spi->beginTransaction(this->spiSettings);
        digitalWrite(this->csPin, LOW);
        this->spi->transfer((REG_XDATA3 << 1) | 0x01);
        for (auto i = 0; i < 9; i++)
            buffer[i] = this->spi->transfer(0x00);
        digitalWrite(this->csPin, HIGH);
        this->spi->endTransaction();

        // 各軸 20bit 左詰め (DATA3=bits[19:12], DATA2=bits[11:4], DATA1[7:4]=bits[3:0]) の
        // 上位 16bit を取り出す (= 20bit 値の算術右シフト 4bit 相当)
        for (auto i = 0; i < 3; i++)
            data[i] = (int16_t)(((uint16_t)buffer[i * 3] << 8) | buffer[i * 3 + 1]);
    }
};
