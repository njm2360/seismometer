#pragma once

#include <Arduino.h>
#include <SPI.h>

// ADXL345 レジスタ直接制御ドライバ (SPI モード3, 最大5MHz)
class ADXL345 {
    // レジスタアドレス (データシート Rev.G Table 19)
    static constexpr uint8_t REG_DEVID = 0x00;       // = 0xE5
    static constexpr uint8_t REG_BW_RATE = 0x2C;
    static constexpr uint8_t REG_POWER_CTL = 0x2D;
    static constexpr uint8_t REG_DATA_FORMAT = 0x31;
    static constexpr uint8_t REG_DATAX0 = 0x32;      // DATAX0..DATAZ1 の6バイト連続読み出し起点

    static constexpr uint8_t DEVID_VALUE = 0xE5;

    // コマンドバイトは bit7=R/W (1=read), bit6=MB (連続読み出し), bit5:0=アドレス
    static constexpr uint8_t CMD_READ = 0x80;
    static constexpr uint8_t CMD_MULTI_BYTE = 0x40;

    // ODR 100Hz / 帯域 50Hz
    static constexpr uint8_t BW_RATE_100HZ = 0x0A;

    // FULL_RES (bit3) + レンジ ±2g (bit1:0 = 00)
    static constexpr uint8_t DATA_FORMAT_FULL_RES_2G = 0x08;

    // Measure (bit3) で計測モード
    static constexpr uint8_t POWER_CTL_MEASURE = 0x08;

    // FULL_RES モード: 全レンジで 256 LSB/g (typ)
    static constexpr float SENSITIVITY_GAL_PER_LSB = 980.665f / 256.0f;

    SPIClass *spi;
    int csPin;
    SPISettings spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE3);

    uint8_t readRegister(uint8_t reg) {
        this->spi->beginTransaction(this->spiSettings);
        digitalWrite(this->csPin, LOW);
        this->spi->transfer(CMD_READ | reg);
        auto value = this->spi->transfer(0x00);
        digitalWrite(this->csPin, HIGH);
        this->spi->endTransaction();
        return value;
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        this->spi->beginTransaction(this->spiSettings);
        digitalWrite(this->csPin, LOW);
        this->spi->transfer(reg);
        this->spi->transfer(value);
        digitalWrite(this->csPin, HIGH);
        this->spi->endTransaction();
    }

public:
    ADXL345(SPIClass *spi = &SPI, int csPin = SS) : spi(spi), csPin(csPin) {}

    void begin() {
        pinMode(this->csPin, OUTPUT);
        digitalWrite(this->csPin, HIGH);
        this->spi->begin();

        // ソフトリセットは存在しないため、デバイス応答の確認のみ行う
        while (this->readRegister(REG_DEVID) != DEVID_VALUE)
            delay(1);

        // スタンバイに戻してから設定し、最後に計測モードへ入れる (データシート Power Sequencing 推奨手順)
        this->writeRegister(REG_POWER_CTL, 0x00);
        this->writeRegister(REG_BW_RATE, BW_RATE_100HZ);
        this->writeRegister(REG_DATA_FORMAT, DATA_FORMAT_FULL_RES_2G);
        this->writeRegister(REG_POWER_CTL, POWER_CTL_MEASURE);

        // 計測モード移行後の安定待ち (1/ODR + 1.1ms ≈ 11.1ms @100Hz)
        delay(12);
    }

    float getSensitivity() {
        return SENSITIVITY_GAL_PER_LSB;
    }

    void read(int16_t (&data)[3])
    {
        uint8_t buffer[6];

        this->spi->beginTransaction(this->spiSettings);
        digitalWrite(this->csPin, LOW);
        this->spi->transfer(CMD_READ | CMD_MULTI_BYTE | REG_DATAX0);
        for (auto i = 0; i < 6; i++)
            buffer[i] = this->spi->transfer(0x00);
        digitalWrite(this->csPin, HIGH);
        this->spi->endTransaction();

        // リトルエンディアン (DATAx0=下位, DATAx1=上位)・2の補数・右詰め符号拡張済み
        for (auto i = 0; i < 3; i++)
            data[i] = (int16_t)(((uint16_t)buffer[i * 2 + 1] << 8) | buffer[i * 2]);
    }
};
