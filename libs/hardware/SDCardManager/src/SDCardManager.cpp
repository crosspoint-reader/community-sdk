#include "SDCardManager.h"
#include <memory>
#include <utility>

namespace {
constexpr uint8_t SD_CS = 12;
constexpr uint32_t SPI_FQ = 40000000;
}

SDCardManager SDCardManager::instance;

SDCardManager::SDCardManager() : sd() {}

bool SDCardManager::begin() {
  if (!sd.begin(SD_CS, SPI_FQ)) {
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

  auto root = sd.open(path);
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
    f.getName(name, sizeof(name));
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

  FsFile f;
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

  FsFile f;
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

  FsFile f;
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
    const int r = f.read(buffer + total, readLen);
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
  if (sd.exists(path)) {
    sd.remove(path);
  }

  FsFile f;
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
  if (sd.exists(path)) {
    FsFile dir = sd.open(path);
    if (dir && dir.isDirectory()) {
      dir.close();
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
      if (Serial) Serial.printf("Directory already exists: %s\n", path);
      return true;
    }
    dir.close();
  }

  // Create the directory
  if (sd.mkdir(path)) {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.printf("Created directory: %s\n", path);
    return true;
  } else {
    if (Serial) Serial.printf("[%lu] [SD] Path is not a directory\n", millis());
    if (Serial) Serial.printf("Failed to create directory: %s\n", path);
    return false;
  }
}

bool SDCardManager::openFileForRead(const char* moduleName, const char* path, FsFile& file) {
  if (!sd.exists(path)) {
    if (Serial) Serial.printf("[%lu] [%s] File does not exist: %s\n", millis(), moduleName, path);
    return false;
  }

  file = sd.open(path, O_RDONLY);
  if (!file) {
    if (Serial) Serial.printf("[%lu] [%s] Failed to open file for reading: %s\n", millis(), moduleName, path);
    return false;
  }
  return true;
}

bool SDCardManager::openFileForRead(const char* moduleName, const std::string& path, FsFile& file) {
  return openFileForRead(moduleName, path.c_str(), file);
}

bool SDCardManager::openFileForRead(const char* moduleName, const String& path, FsFile& file) {
  return openFileForRead(moduleName, path.c_str(), file);
}

bool SDCardManager::openFileForWrite(const char* moduleName, const char* path, FsFile& file) {
  file = sd.open(path, O_RDWR | O_CREAT | O_TRUNC);
  if (!file) {
    if (Serial) Serial.printf("[%lu] [%s] Failed to open file for writing: %s\n", millis(), moduleName, path);
    return false;
  }
  return true;
}

bool SDCardManager::openFileForWrite(const char* moduleName, const std::string& path, FsFile& file) {
  return openFileForWrite(moduleName, path.c_str(), file);
}

bool SDCardManager::openFileForWrite(const char* moduleName, const String& path, FsFile& file) {
  return openFileForWrite(moduleName, path.c_str(), file);
}

bool SDCardManager::removeDir(const char* path) {
  // Heap-allocated name buffer: 255 UTF-8 characters + null terminator
  constexpr size_t NAME_BUFFER_SIZE = 1021;
  auto nameBuf = std::make_unique<char[]>(NAME_BUFFER_SIZE);

  // Stack of (dirPath, postOrder): postOrder=true means rmdir this path
  std::vector<std::pair<std::string, bool>> stack;
  stack.reserve(16);
  stack.push_back({path, false});

  while (!stack.empty()) {
    auto [dirPath, postOrder] = std::move(stack.back());
    stack.pop_back();

    if (postOrder) {
      if (!sd.rmdir(dirPath.c_str())) {
        return false;
      }
      continue;
    }

    auto dir = sd.open(dirPath.c_str());
    if (!dir || !dir.isDirectory()) {
      return false;
    }

    // Push this dir for post-order rmdir (after all children are processed)
    stack.push_back({dirPath, true});

    for (auto file = dir.openNextFile(); file; file = dir.openNextFile()) {
      file.getName(nameBuf.get(), NAME_BUFFER_SIZE);
      std::string entryPath = dirPath;
      if (entryPath.back() != '/') {
        entryPath += '/';
      }
      entryPath += nameBuf.get();

      const bool isDir = file.isDirectory();
      file.close();

      if (isDir) {
        stack.push_back({std::move(entryPath), false});
      } else {
        if (!sd.remove(entryPath.c_str())) {
          return false;
        }
      }
    }
    dir.close();
  }

  return true;
}
