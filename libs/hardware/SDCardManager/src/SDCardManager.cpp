#include "SDCardManager.h"

#include <BoardConfig.h>
#include <SPI.h>
#if defined(SDCARDMANAGER_USE_SD_MMC)
#include <SD.h>
#include <SD_MMC.h>
#endif

namespace {
constexpr uint32_t SPI_FQ = 40000000;
constexpr uint32_t MURPHY_M3_BRINGUP_SPI_FQ = 4000000;
constexpr uint32_t MURPHY_M3_SDMMC_BRINGUP_FQ_KHZ = 4000;
constexpr int MURPHY_SDMMC_CLK = 16;
constexpr int MURPHY_SDMMC_CMD = 17;
constexpr int MURPHY_SDMMC_D0 = 15;
constexpr int MURPHY_SDMMC_D1 = 14;
constexpr int MURPHY_SDMMC_D2 = 21;
constexpr int MURPHY_SDMMC_D3 = 18;
// OEM SD init drives GPIO10 low immediately before SD_MMC.begin(), then
// drives it high during the corresponding close/unmount path.
constexpr int MURPHY_SDMMC_ENABLE = 10;
constexpr int MURPHY_SD_SPI_SCK = 39;
constexpr int MURPHY_SD_SPI_MISO = 13;
constexpr int MURPHY_SD_SPI_MOSI = 40;
constexpr int MURPHY_SD_SPI_CS = 10;

const char* spiOptionName(uint8_t option) {
  return (option & DEDICATED_SPI) ? "dedicated" : "shared";
}

#if defined(SDCARDMANAGER_USE_SD_MMC)
bool murphyUseSdMmc = true;

const char* fileModeFromOpenFlags(const oflag_t oflag) {
  if ((oflag & O_APPEND) != 0) return FILE_APPEND;
  if ((oflag & (O_WRONLY | O_RDWR | O_CREAT | O_TRUNC)) != 0) return FILE_WRITE;
  return FILE_READ;
}

fs::FS& murphyFs() {
  return murphyUseSdMmc ? static_cast<fs::FS&>(SD_MMC) : static_cast<fs::FS&>(SD);
}
#endif
}

SDCardManager SDCardManager::instance;

SDCardManager::SDCardManager() : sd() {}

bool SDCardManager::begin() {
  const auto& board = BoardConfig::ACTIVE;
#if defined(SDCARDMANAGER_USE_SD_MMC)
  if (BoardConfig::isMurphyM3()) {
    constexpr uint32_t murphyFrequenciesKhz[] = {MURPHY_M3_SDMMC_BRINGUP_FQ_KHZ, 1000, 400};
    if (Serial) {
      Serial.printf("[%lu] [SD] Murphy SD_MMC init clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d\n",
                    millis(), MURPHY_SDMMC_CLK, MURPHY_SDMMC_CMD, MURPHY_SDMMC_D0, MURPHY_SDMMC_D1,
                    MURPHY_SDMMC_D2, MURPHY_SDMMC_D3);
    }

    pinMode(MURPHY_SDMMC_ENABLE, OUTPUT);
    digitalWrite(MURPHY_SDMMC_ENABLE, LOW);
    if (Serial) {
      Serial.printf("[%lu] [SD] Murphy OEM SD enable GPIO%d LOW\n", millis(), MURPHY_SDMMC_ENABLE);
    }
    delay(5);

    if (board.sd.powerEnable >= 0) {
      pinMode(board.sd.powerEnable, OUTPUT);
      digitalWrite(board.sd.powerEnable, HIGH);
      delay(100);
    }

    SD_MMC.setPins(MURPHY_SDMMC_CLK, MURPHY_SDMMC_CMD, MURPHY_SDMMC_D0, MURPHY_SDMMC_D1, MURPHY_SDMMC_D2,
                   MURPHY_SDMMC_D3);
    for (uint32_t freq : murphyFrequenciesKhz) {
      if (Serial) {
        Serial.printf("[%lu] [SD] try SD_MMC 4-bit freq=%lu kHz\n", millis(),
                      static_cast<unsigned long>(freq));
      }
      if (SD_MMC.begin("/sd", false, false, static_cast<int>(freq), 5)) {
        murphyUseSdMmc = true;
        initialized = true;
        if (Serial) {
          Serial.printf("[%lu] [SD] SD_MMC mounted type=%d size=%llu MB\n", millis(), SD_MMC.cardType(),
                        static_cast<unsigned long long>(SD_MMC.cardSize() / (1024ULL * 1024ULL)));
        }
        return true;
      }
      SD_MMC.end();
      delay(250);
    }

    if (Serial) Serial.printf("[%lu] [SD] SD_MMC 4-bit failed; trying 1-bit fallback\n", millis());
    SD_MMC.setPins(MURPHY_SDMMC_CLK, MURPHY_SDMMC_CMD, MURPHY_SDMMC_D0);
    for (uint32_t freq : murphyFrequenciesKhz) {
      if (Serial) {
        Serial.printf("[%lu] [SD] try SD_MMC 1-bit freq=%lu kHz\n", millis(),
                      static_cast<unsigned long>(freq));
      }
      if (SD_MMC.begin("/sd", true, false, static_cast<int>(freq), 5)) {
        murphyUseSdMmc = true;
        initialized = true;
        if (Serial) {
          Serial.printf("[%lu] [SD] SD_MMC 1-bit mounted type=%d size=%llu MB\n", millis(), SD_MMC.cardType(),
                        static_cast<unsigned long long>(SD_MMC.cardSize() / (1024ULL * 1024ULL)));
        }
        return true;
      }
      SD_MMC.end();
      delay(250);
    }

    static SPIClass murphySdSpi(FSPI);
    murphySdSpi.begin(MURPHY_SD_SPI_SCK, MURPHY_SD_SPI_MISO, MURPHY_SD_SPI_MOSI, MURPHY_SD_SPI_CS);
    pinMode(MURPHY_SD_SPI_CS, OUTPUT);
    digitalWrite(MURPHY_SD_SPI_CS, HIGH);
    delay(20);

    constexpr uint32_t murphySpiFrequenciesHz[] = {4000000, 1000000, 400000};
    if (Serial) {
      Serial.printf("[%lu] [SD] SD_MMC failed; trying Arduino SD SPI sck=%d miso=%d mosi=%d cs=%d\n",
                    millis(), MURPHY_SD_SPI_SCK, MURPHY_SD_SPI_MISO, MURPHY_SD_SPI_MOSI, MURPHY_SD_SPI_CS);
    }
    for (uint32_t freq : murphySpiFrequenciesHz) {
      if (Serial) {
        Serial.printf("[%lu] [SD] try Arduino SD SPI freq=%lu Hz\n", millis(),
                      static_cast<unsigned long>(freq));
      }
      if (SD.begin(MURPHY_SD_SPI_CS, murphySdSpi, freq, "/sd", 5)) {
        murphyUseSdMmc = false;
        initialized = true;
        if (Serial) {
          Serial.printf("[%lu] [SD] Arduino SD mounted type=%d size=%llu MB\n", millis(), SD.cardType(),
                        static_cast<unsigned long long>(SD.cardSize() / (1024ULL * 1024ULL)));
        }
        return true;
      }
      SD.end();
      delay(250);
    }

    initialized = false;
    if (Serial) Serial.printf("[%lu] [SD] SD card not detected on SD_MMC or Arduino SD SPI\n", millis());
    return false;
  }
#endif

  if (board.sd.powerEnable >= 0) {
    pinMode(board.sd.powerEnable, OUTPUT);
    digitalWrite(board.sd.powerEnable, HIGH);
    delay(BoardConfig::isMurphyM3() ? 100 : 10);
  }

  const uint32_t spiFrequency = BoardConfig::isMurphyM3() ? MURPHY_M3_BRINGUP_SPI_FQ : SPI_FQ;
  bool mounted = false;
  if (board.sd.separateSpi) {
    static SPIClass sdSpi(FSPI);
    if (Serial) {
      Serial.printf("[%lu] [SD] Murphy/X4 SDK SD bringup v2 separate SPI sclk=%d miso=%d mosi=%d cs=%d pwr=%d freq=%lu\n", millis(),
                    board.sd.sclk, board.sd.miso, board.sd.mosi, board.sd.cs, board.sd.powerEnable,
                    static_cast<unsigned long>(spiFrequency));
    }
    sdSpi.begin(board.sd.sclk, board.sd.miso, board.sd.mosi, board.sd.cs);
    pinMode(board.sd.cs, OUTPUT);
    digitalWrite(board.sd.cs, HIGH);
    delay(BoardConfig::isMurphyM3() ? 20 : 1);

    if (BoardConfig::isMurphyM3()) {
      constexpr uint32_t murphyFrequencies[] = {MURPHY_M3_BRINGUP_SPI_FQ, 1000000, 400000};
      constexpr uint8_t murphyOptions[] = {static_cast<uint8_t>(SHARED_SPI | USER_SPI_BEGIN)};
      {
        pinMode(board.sd.miso, INPUT_PULLUP);
        pinMode(board.sd.mosi, INPUT_PULLUP);
        pinMode(board.sd.cs, OUTPUT);
        digitalWrite(board.sd.cs, HIGH);
        delay(10);

        for (uint8_t option : murphyOptions) {
          for (uint32_t freq : murphyFrequencies) {
            if (Serial) {
              Serial.printf("[%lu] [SD] try %s SPI freq=%lu\n", millis(), spiOptionName(option),
                            static_cast<unsigned long>(freq));
            }
            sd.end();
            delay(5);
            mounted = sd.begin(SdSpiConfig(board.sd.cs, option, freq, &sdSpi));
            if (mounted) {
              if (Serial) {
                Serial.printf("[%lu] [SD] mounted with %s SPI freq=%lu\n", millis(), spiOptionName(option),
                              static_cast<unsigned long>(freq));
              }
              break;
            }
            if (Serial) {
              Serial.printf("[%lu] [SD] fail %s SPI freq=%lu err=0x%02x data=0x%02x\n", millis(),
                            spiOptionName(option), static_cast<unsigned long>(freq), sd.sdErrorCode(),
                            sd.sdErrorData());
            }
          }
          if (mounted) break;
        }
      }
    } else {
      mounted = sd.begin(SdSpiConfig(board.sd.cs, DEDICATED_SPI, spiFrequency, &sdSpi));
    }
  } else {
    if (Serial) {
      Serial.printf("[%lu] [SD] init shared SPI cs=%d freq=%lu\n", millis(), board.sd.cs,
                    static_cast<unsigned long>(spiFrequency));
    }
    mounted = sd.begin(board.sd.cs, spiFrequency);
  }

  if (!mounted) {
    if (Serial) Serial.printf("[%lu] [SD] SD card not detected\n", millis());
    initialized = false;
  } else {
    if (Serial) Serial.printf("[%lu] [SD] SD card detected\n", millis());
    initialized = true;
  }

  return initialized;
}

bool SDCardManager::ready() const {
  return initialized;
}

std::vector<String> SDCardManager::listFiles(const char* path, const int maxFiles) {
  std::vector<String> ret;
  if (!initialized) {
    if (Serial) Serial.printf("[%lu] [SD] not initialized, returning empty list\n", millis());
    return ret;
  }

  auto root = open(path);
  if (!root) {
    if (Serial) Serial.printf("[%lu] [SD] Failed to open directory\n", millis());
    return ret;
  }
  if (!root.isDirectory()) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    root.close();
    return ret;
  }

  int count = 0;
  char name[128];
  for (auto f = root.openNextFile(); f && count < maxFiles; f = root.openNextFile()) {
    if (f.isDirectory()) {
      f.close();
      continue;
    }
#if defined(SDCARDMANAGER_USE_SD_MMC)
    strlcpy(name, f.name(), sizeof(name));
#else
    f.getName(name, sizeof(name));
#endif
    ret.emplace_back(name);
    f.close();
    count++;
  }
  root.close();
  return ret;
}

String SDCardManager::readFile(const char* path) {
  if (!initialized) {
    if (Serial) Serial.printf("[%lu] [SD] not initialized; cannot read file\n", millis());
    return {""};
  }

  SDManagedFile f;
  if (!openFileForRead("SD", path, f)) {
    return {""};
  }

  String content = "";
  constexpr size_t maxSize = 50000;  // Limit to 50KB
  size_t readSize = 0;
  while (f.available() && readSize < maxSize) {
    const char c = static_cast<char>(f.read());
    content += c;
    readSize++;
  }
  f.close();
  return content;
}

bool SDCardManager::readFileToStream(const char* path, Print& out, const size_t chunkSize) {
  if (!initialized) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.println("SDCardManager: not initialized; cannot read file");
    return false;
  }

  SDManagedFile f;
  if (!openFileForRead("SD", path, f)) {
    return false;
  }

  constexpr size_t localBufSize = 256;
  uint8_t buf[localBufSize];
  const size_t toRead = (chunkSize == 0) ? localBufSize : (chunkSize < localBufSize ? chunkSize : localBufSize);

  while (f.available()) {
    const int r = f.read(buf, toRead);
    if (r > 0) {
      out.write(buf, static_cast<size_t>(r));
    } else {
      break;
    }
  }

  f.close();
  return true;
}

size_t SDCardManager::readFileToBuffer(const char* path, char* buffer, const size_t bufferSize, const size_t maxBytes) {
  if (!buffer || bufferSize == 0)
    return 0;
  if (!initialized) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.println("SDCardManager: not initialized; cannot read file");
    buffer[0] = '\0';
    return 0;
  }

  SDManagedFile f;
  if (!openFileForRead("SD", path, f)) {
    buffer[0] = '\0';
    return 0;
  }

  const size_t maxToRead = (maxBytes == 0) ? (bufferSize - 1) : min(maxBytes, bufferSize - 1);
  size_t total = 0;

  while (f.available() && total < maxToRead) {
    constexpr size_t chunk = 64;
    const size_t want = maxToRead - total;
    const size_t readLen = (want < chunk) ? want : chunk;
    const int r = f.read(reinterpret_cast<uint8_t*>(buffer + total), readLen);
    if (r > 0) {
      total += static_cast<size_t>(r);
    } else {
      break;
    }
  }

  buffer[total] = '\0';
  f.close();
  return total;
}

bool SDCardManager::writeFile(const char* path, const String& content) {
  if (!initialized) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.println("SDCardManager: not initialized; cannot write file");
    return false;
  }

  // Remove existing file so we perform an overwrite rather than append
  if (exists(path)) {
    remove(path);
  }

  SDManagedFile f;
  if (!openFileForWrite("SD", path, f)) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.printf("Failed to open file for write: %s\n", path);
    return false;
  }

  const size_t written = f.print(content);
  f.close();
  return written == content.length();
}

bool SDCardManager::ensureDirectoryExists(const char* path) {
  if (!initialized) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.println("SDCardManager: not initialized; cannot create directory");
    return false;
  }

  // Check if directory already exists
  if (exists(path)) {
    SDManagedFile dir = open(path);
    if (dir && dir.isDirectory()) {
      dir.close();
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
      if (Serial) Serial.printf("Directory already exists: %s\n", path);
      return true;
    }
    dir.close();
  }

  // Create the directory
  if (mkdir(path)) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.printf("Created directory: %s\n", path);
    return true;
  } else {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.printf("Failed to create directory: %s\n", path);
    return false;
  }
}

SDManagedFile SDCardManager::open(const char* path, const oflag_t oflag) {
#if defined(SDCARDMANAGER_USE_SD_MMC)
  return murphyFs().open(path, fileModeFromOpenFlags(oflag));
#else
  return sd.open(path, oflag);
#endif
}

bool SDCardManager::mkdir(const char* path, const bool pFlag) {
#if defined(SDCARDMANAGER_USE_SD_MMC)
  (void)pFlag;
  return murphyFs().mkdir(path);
#else
  return sd.mkdir(path, pFlag);
#endif
}

bool SDCardManager::exists(const char* path) {
#if defined(SDCARDMANAGER_USE_SD_MMC)
  return murphyFs().exists(path);
#else
  return sd.exists(path);
#endif
}

bool SDCardManager::remove(const char* path) {
#if defined(SDCARDMANAGER_USE_SD_MMC)
  return murphyFs().remove(path);
#else
  return sd.remove(path);
#endif
}

bool SDCardManager::rmdir(const char* path) {
#if defined(SDCARDMANAGER_USE_SD_MMC)
  return murphyFs().rmdir(path);
#else
  return sd.rmdir(path);
#endif
}

bool SDCardManager::rename(const char* path, const char* newPath) {
#if defined(SDCARDMANAGER_USE_SD_MMC)
  return murphyFs().rename(path, newPath);
#else
  return sd.rename(path, newPath);
#endif
}

bool SDCardManager::openFileForRead(const char* moduleName, const char* path, SDManagedFile& file) {
  if (!exists(path)) {
    if (Serial) Serial.printf("[%lu] [%s] File does not exist: %s\n", millis(), moduleName, path);
    return false;
  }

  file = open(path, O_RDONLY);
  if (!file) {
    if (Serial) Serial.printf("[%lu] [%s] Failed to open file for reading: %s\n", millis(), moduleName, path);
    return false;
  }
  return true;
}

bool SDCardManager::openFileForRead(const char* moduleName, const std::string& path, SDManagedFile& file) {
  return openFileForRead(moduleName, path.c_str(), file);
}

bool SDCardManager::openFileForRead(const char* moduleName, const String& path, SDManagedFile& file) {
  return openFileForRead(moduleName, path.c_str(), file);
}

bool SDCardManager::openFileForWrite(const char* moduleName, const char* path, SDManagedFile& file) {
  file = open(path, O_RDWR | O_CREAT | O_TRUNC);
  if (!file) {
    if (Serial) Serial.printf("[%lu] [%s] Failed to open file for writing: %s\n", millis(), moduleName, path);
    return false;
  }
  return true;
}

bool SDCardManager::openFileForWrite(const char* moduleName, const std::string& path, SDManagedFile& file) {
  return openFileForWrite(moduleName, path.c_str(), file);
}

bool SDCardManager::openFileForWrite(const char* moduleName, const String& path, SDManagedFile& file) {
  return openFileForWrite(moduleName, path.c_str(), file);
}

bool SDCardManager::removeDir(const char* path) {
  // 1. Open the directory
  auto dir = open(path);
  if (!dir) {
    return false;
  }
  if (!dir.isDirectory()) {
    return false;
  }

  auto file = dir.openNextFile();
  char name[128];
  while (file) {
    String filePath = path;
    if (!filePath.endsWith("/")) {
      filePath += "/";
    }
#if defined(SDCARDMANAGER_USE_SD_MMC)
    strlcpy(name, file.name(), sizeof(name));
#else
    file.getName(name, sizeof(name));
#endif
    filePath += name;

    if (file.isDirectory()) {
      if (!removeDir(filePath.c_str())) {
        return false;
      }
    } else {
      if (!remove(filePath.c_str())) {
        return false;
      }
    }
    file = dir.openNextFile();
  }

  return rmdir(path);
}
