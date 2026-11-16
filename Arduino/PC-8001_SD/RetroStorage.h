#ifndef RETRO_STORAGE_H
#define RETRO_STORAGE_H
#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"

// One raw SD instance is shared by the original program and host-to-SD copy.
static SdFat bridgeNativeSD;
static bool bridgeSdReady = false;
#if RETRO_PROFILE == 3 || RETRO_PROFILE == 4
// These original sketches already have a 260-byte transfer array.
extern uint8_t s_data[260];
#define bridgeBuffer s_data
#else
static uint8_t bridgeBuffer[256];
#endif
static uint16_t bridgeEpoch = 1;
static uint8_t bridgeWriters = 0;
static uint32_t bridgeSplitLeft = 0;
static bool bridgePcMode();
static void bridgeFatal();
static bool bridgeFlush();
static void bridgeReset();
static uint8_t bridgeHandshake();
static uint8_t bridgeCall(
    uint8_t op, const uint8_t *data, uint16_t count, uint16_t limit, uint16_t &received);

#if defined(__AVR__)
#define BRIDGE_NOINLINE __attribute__((noinline))
#else
#define BRIDGE_NOINLINE
#endif

// AVR/FAT builds do not need Arduino Stream/Print state in each SD handle.
// Keep the same file operations with the smaller native FatFile object.
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000 && SDFAT_FILE_TYPE == 1
class BridgeNativeFile : public FatFile
{
  public:
    uint32_t size() const
    {
        return fileSize();
    }

    void rewindDirectory()
    {
        rewind();
    }

    bool seek(uint32_t offset)
    {
        return seekSet(offset);
    }

    uint32_t position() const
    {
        return curPosition();
    }

    BridgeNativeFile openNextFile()
    {
        BridgeNativeFile next;
        next.openNext(this, FILE_READ);
        return next;
    }
};
#else
typedef File BridgeNativeFile;
#endif

static bool bridgeNativeOpen(BridgeNativeFile &file, const char *name, int flags);

struct BridgeSlot
{
    BridgeNativeFile local;
    uint8_t references;
    bool used;
    bool remote;
    bool writable;
    uint8_t id;
    uint32_t position;
    uint32_t length;
};

static BridgeSlot bridgeSlots[4];
static uint8_t bridgeCacheKind = 0;
static uint8_t bridgeCacheSlot = 0;
static uint16_t bridgeCacheLength = 0;
static uint32_t bridgeCacheStart = 0;

class BridgeFile
{
  public:
    BridgeFile();
    BridgeFile(const BridgeFile &other);
    ~BridgeFile();
    BridgeFile &operator=(const BridgeFile &other);
    operator bool() const;
    uint32_t size();
    int read();
    int read(void *destination, size_t length);
    size_t write(uint8_t value);
    size_t write(const void *data, size_t length);
    bool seek(uint32_t position);
    bool sync();
    void close();
    BridgeFile openNextFile();
    void getName(char *destination, size_t capacity);
    void rewindDirectory();

  private:
    friend class BridgeSd;
    int8_t slot;
    uint8_t kind;
    uint16_t epoch;
    uint16_t entryIndex;
    uint16_t entryCount;
    bool valid() const;
};

class BridgeSd
{
  public:
    bool begin(int chipSelect, int speed);
    bool exists(const char *name);
    bool remove(const char *name);
    BridgeFile open(const char *name, int flags = FILE_READ);
    BridgeFile openNativeEntry(BridgeNativeFile &directory, uint16_t index);
};
#endif
