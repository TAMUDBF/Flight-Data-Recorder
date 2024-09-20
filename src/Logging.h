#ifndef LOGGING_H
#define LOGGING_H

#include <SdFat.h>

class loggingHandler{
private:
    //String filename;
    SdFs sd;            // Use SdFs for SD_FAT_TYPE == 3
    FsFile csvFile;        // Use FsFile for SD_FAT_TYPE == 3 (always declared here)
    
    // Pin configuration
    #ifndef SDCARD_SS_PIN
    const uint8_t SD_CS_PIN = SS;
    #else
    const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
    #endif

    // SD config setup
    #define SPI_CLOCK SD_SCK_MHZ(50)
    #if HAS_SDIO_CLASS
    #define SD_CONFIG SdioConfig(FIFO_SDIO)
    #elif ENABLE_DEDICATED_SPI
    #define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
    #else
    #define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
    #endif
public:
    void setup();
    void csvLog();
    void serialLog();
};

#endif