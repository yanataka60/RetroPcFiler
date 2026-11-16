#ifndef RETRO_STORAGE_IMPL_H
#define RETRO_STORAGE_IMPL_H

static bool bridgeNativeOpen(BridgeNativeFile &file, const char *name, int flags)
{
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    return file.open(bridgeNativeSD.vol(), name, flags);
#else
    return file.open(name, flags);
#endif
}

static bool bridgePcMode()
{
    return rt_backend != 0;
}

static uint32_t bridgeRead32(const uint8_t *data)
{
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static void bridgeWrite32(uint8_t *data, uint32_t value)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        data[i] = (uint8_t)(value >> (8 * i));
    }
}

static void bridgeFatal()
{
    // Keep the MZ-era machine waiting rather than returning fabricated bytes.
    rt_busy = true;
    rt_emit(0x72, 0, 0);
    for (;;)
    {
        rt_poll(false);
    }
}

static uint8_t bridgeCall(uint8_t op, const uint8_t *data, uint16_t count, uint16_t limit, uint16_t &received)
{
    return pc_call(op, data, count, limit, received);
}

static uint8_t bridgeHandshake()
{
    uint8_t profile = RETRO_PROFILE;
    uint16_t received = 0;
    uint8_t result = bridgeCall(0x01, &profile, 1, 1, received);
    return result == 0 && received == 1 && bridgeBuffer[0] == 1 ? 0 : 0xf2;
}

static bool bridgeFlush()
{
    if (bridgeCacheKind != 2)
    {
        return true;
    }
    BridgeSlot &handle = bridgeSlots[bridgeCacheSlot];
    bridgeBuffer[0] = handle.remoteFile.id;
    bridgeWrite32(bridgeBuffer + 1, bridgeCacheStart);
    uint16_t received = 0;
    uint8_t status = bridgeCall(0x53, bridgeBuffer, bridgeCacheLength + 5, 0, received);
    if (status != 0)
    {
        return false;
    }
    bridgeCacheKind = 0;
    bridgeCacheLength = 0;
    return true;
}

static void bridgeReset()
{
    // A mode change is refused while any writer or split save is held.
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (bridgeSlots[i].used && !bridgeSlots[i].remote)
        {
            bridgeSlots[i].local.close();
        }
        bridgeSlots[i].used = false;
    }
    ++bridgeEpoch;
    bridgeCacheKind = 0;
    bridgeCacheLength = 0;
    bridgeWriters = 0;
    bridgeSplitLeft = 0;
}

BridgeFile::BridgeFile() : slot(-1), kind(0), epoch(bridgeEpoch), entryIndex(0), entryCount(0)
{
}

bool BridgeFile::valid() const
{
    return epoch == bridgeEpoch && kind != 0 &&
           ((slot < 0 && kind != 1) || (slot >= 0 && bridgeSlots[(uint8_t)slot].used));
}

BridgeFile::operator bool() const
{
    return valid();
}

BridgeFile::BridgeFile(const BridgeFile &other)
    : slot(other.slot), kind(other.kind), epoch(other.epoch), entryIndex(other.entryIndex),
      entryCount(other.entryCount)
{
    if (valid() && slot >= 0 && (kind == 1 || kind == 2 || kind == 3))
    {
        ++bridgeSlots[(uint8_t)slot].references;
    }
}

BridgeFile::~BridgeFile()
{
    close();
}

BridgeFile &BridgeFile::operator=(const BridgeFile &other)
{
    if (this != &other)
    {
        close();
        slot = other.slot;
        kind = other.kind;
        epoch = other.epoch;
        entryIndex = other.entryIndex;
        entryCount = other.entryCount;
        if (valid() && slot >= 0 && (kind == 1 || kind == 2 || kind == 3))
        {
            ++bridgeSlots[(uint8_t)slot].references;
        }
    }
    return *this;
}

uint32_t BridgeFile::size()
{
    if (!valid() || kind != 1)
    {
        return 0;
    }
    BridgeSlot &handle = bridgeSlots[(uint8_t)slot];
    return handle.remote ? handle.remoteFile.length : handle.local.size();
}

int BridgeFile::read()
{
    if (!valid() || kind != 1)
    {
        return -1;
    }
    BridgeSlot &handle = bridgeSlots[(uint8_t)slot];
    if (!handle.remote)
    {
        return handle.local.read();
    }
    if (handle.remoteFile.position >= handle.remoteFile.length)
    {
        return -1;
    }
    if (bridgeCacheKind != 1 || bridgeCacheSlot != (uint8_t)slot ||
        handle.remoteFile.position < bridgeCacheStart ||
        handle.remoteFile.position >= bridgeCacheStart + bridgeCacheLength)
    {
        if (!bridgeFlush())
        {
            bridgeFatal();
        }
        uint16_t count = handle.remoteFile.length - handle.remoteFile.position > 256
                             ? 256
                             : (uint16_t)(handle.remoteFile.length - handle.remoteFile.position);
        uint8_t request[7] = {handle.remoteFile.id};
        bridgeWrite32(request + 1, handle.remoteFile.position);
        request[5] = (uint8_t)count;
        request[6] = (uint8_t)(count >> 8);
        uint16_t received = 0;
        if (bridgeCall(0x51, request, sizeof(request), 256, received) != 0 || received != count)
        {
            bridgeFatal();
        }
        bridgeCacheKind = 1;
        bridgeCacheSlot = (uint8_t)slot;
        bridgeCacheStart = handle.remoteFile.position;
        bridgeCacheLength = count;
    }
    return bridgeBuffer[handle.remoteFile.position++ - bridgeCacheStart];
}

int BridgeFile::read(void *destination, size_t length)
{
    uint8_t *bytes = (uint8_t *)destination;
    size_t actual = 0;
    while (actual < length)
    {
        int value = read();
        if (value < 0)
        {
            break;
        }
        bytes[actual++] = (uint8_t)value;
    }
    return (int)actual;
}

size_t BridgeFile::write(uint8_t value)
{
    return write(&value, 1);
}

size_t BridgeFile::write(const void *data, size_t length)
{
    if (!valid() || kind != 1)
    {
        return 0;
    }
    BridgeSlot &handle = bridgeSlots[(uint8_t)slot];
    if (!handle.writable)
    {
        return 0;
    }
    if (!handle.remote)
    {
        return handle.local.write((const uint8_t *)data, length);
    }
    if (data == (const void *)bridgeBuffer && length <= 256)
    {
        // X1 receives a legacy block directly into the shared array.
        // Preserve the final bytes before making room for the RPC header.
        uint8_t tail[5];
        size_t first = length > 251 ? 251 : length;
        size_t remaining = length - first;
        if (remaining)
        {
            memcpy(tail, bridgeBuffer + first, remaining);
        }
        bridgeCacheKind = 0;
        for (uint8_t part = 0; part < 2; ++part)
        {
            size_t count = part == 0 ? first : remaining;
            if (count == 0)
            {
                continue;
            }
            if (part == 0)
            {
                memmove(bridgeBuffer + 5, bridgeBuffer, count);
            }
            else
            {
                memcpy(bridgeBuffer + 5, tail, count);
            }
            bridgeBuffer[0] = handle.remoteFile.id;
            bridgeWrite32(bridgeBuffer + 1, handle.remoteFile.position);
            uint16_t received = 0;
            if (bridgeCall(0x53, bridgeBuffer, (uint16_t)(count + 5), 0, received) != 0)
            {
                bridgeFatal();
            }
            handle.remoteFile.position += (uint32_t)count;
        }
        if (handle.remoteFile.position > handle.remoteFile.length)
        {
            handle.remoteFile.length = handle.remoteFile.position;
        }
        return length;
    }
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < length; ++i)
    {
        if (bridgeCacheKind != 2 || bridgeCacheSlot != (uint8_t)slot || bridgeCacheLength == 251)
        {
            if (!bridgeFlush())
            {
                bridgeFatal();
            }
            bridgeCacheKind = 2;
            bridgeCacheSlot = (uint8_t)slot;
            bridgeCacheStart = handle.remoteFile.position;
            bridgeCacheLength = 0;
        }
        bridgeBuffer[5 + bridgeCacheLength++] = bytes[i];
        ++handle.remoteFile.position;
        if (handle.remoteFile.position > handle.remoteFile.length)
        {
            handle.remoteFile.length = handle.remoteFile.position;
        }
    }
    return length;
}

bool BridgeFile::seek(uint32_t position)
{
    if (!valid() || kind != 1)
    {
        return false;
    }
    BridgeSlot &handle = bridgeSlots[(uint8_t)slot];
    if (!handle.remote)
    {
        return handle.local.seek(position);
    }
    if (!bridgeFlush())
    {
        bridgeFatal();
    }
    handle.remoteFile.position = position;
    bridgeCacheKind = 0;
    return true;
}

bool BridgeFile::sync()
{
    if (!valid() || kind != 1)
    {
        return false;
    }
    BridgeSlot &handle = bridgeSlots[(uint8_t)slot];
    return handle.remote ? bridgeFlush() : handle.local.sync();
}

void BridgeFile::close()
{
    if (valid() && slot >= 0 && (kind == 1 || kind == 2 || kind == 3))
    {
        BridgeSlot &handle = bridgeSlots[(uint8_t)slot];
        if (handle.references > 1)
        {
            --handle.references;
            kind = 0;
            slot = -1;
            return;
        }
        if (handle.remote)
        {
            if (!bridgeFlush())
            {
                bridgeFatal();
            }
            uint16_t received = 0;
            if (bridgeCall(0x54, &handle.remoteFile.id, 1, 0, received) != 0)
            {
                bridgeFatal();
            }
        }
        else
        {
            handle.local.close();
        }
        if (handle.writable && bridgeWriters)
        {
            --bridgeWriters;
        }
        handle.used = false;
        if (bridgeCacheSlot == (uint8_t)slot)
        {
            bridgeCacheKind = 0;
        }
    }
    kind = 0;
    slot = -1;
}

BRIDGE_NOINLINE BridgeFile BridgeFile::openNextFile()
{
    BridgeFile entry;
    if (!valid() || kind != 2)
    {
        return entry;
    }
    if (!bridgePcMode())
    {
        BridgeNativeFile native = bridgeSlots[(uint8_t)slot].local.openNextFile();
        if (native)
        {
            // Retain an index and the parent handle, not 37 bytes per object.
            entry.kind = 3;
            entry.slot = slot;
            entry.entryIndex = native.dirIndex();
            ++bridgeSlots[(uint8_t)slot].references;
            native.close();
        }
        return entry;
    }
    if (entryIndex < entryCount)
    {
        entry.kind = 3;
        entry.entryIndex = entryIndex++;
    }
    return entry;
}

void BridgeFile::getName(char *destination, size_t capacity)
{
    if (capacity == 0)
    {
        return;
    }
    destination[0] = 0;
    if (!valid() || kind != 3)
    {
        return;
    }
    // Preserve the existing 36-character name limit including extension.
    size_t limit = capacity < 37 ? capacity : 37;
    if (slot >= 0)
    {
        BridgeNativeFile &parent = bridgeSlots[(uint8_t)slot].local;
        uint32_t savedPosition = parent.position();
        BridgeNativeFile native;
        if (native.open(&parent, entryIndex, FILE_READ))
        {
            native.getName(destination, limit);
            native.close();
        }
        // Re-reading an earlier entry must not rewind directory enumeration.
        parent.seek(savedPosition);
        destination[limit - 1] = 0;
        return;
    }
    if (!bridgeFlush())
    {
        bridgeFatal();
    }
    uint8_t request[2] = {(uint8_t)entryIndex, (uint8_t)(entryIndex >> 8)};
    uint16_t received = 0;
    if (bridgeCall(0x58, request, sizeof(request), 36, received) != 0)
    {
        bridgeFatal();
    }
    bridgeCacheKind = 0;
    size_t length = received < limit - 1 ? received : limit - 1;
    memcpy(destination, bridgeBuffer, length);
    destination[length] = 0;
}

uint16_t BridgeFile::directoryEntryIndex() const
{
    return entryIndex;
}

void BridgeFile::getEntryName(uint16_t index, char *destination, size_t capacity)
{
    if (capacity == 0)
        return;
    destination[0] = 0;
    if (!valid() || kind != 2)
        return;
    BridgeFile entry;
    entry.kind = 3;
    entry.entryIndex = index;
    entry.slot = slot;
    if (slot >= 0)
        ++bridgeSlots[(uint8_t)slot].references;
    entry.getName(destination, capacity);
}

void BridgeFile::rewindDirectory()
{
    if (!valid() || kind != 2)
    {
        return;
    }
    if (bridgePcMode())
    {
        entryIndex = 0;
    }
    else
    {
        bridgeSlots[(uint8_t)slot].local.rewindDirectory();
    }
}

bool BridgeSd::begin(int chipSelect, int speed)
{
    if (bridgePcMode())
    {
        return true;
    }
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    uint32_t divider = speed > 0 ? (uint32_t)speed : 8UL;
    bridgeSdReady = bridgeNativeSD.begin(chipSelect, (uint32_t)F_CPU / divider);
#else
    bridgeSdReady = bridgeNativeSD.begin(chipSelect, speed);
#endif
    return bridgeSdReady;
}

bool BridgeSd::exists(const char *name)
{
    if (!bridgePcMode())
    {
        return bridgeNativeSD.exists(name);
    }
    if (!bridgeFlush())
    {
        bridgeFatal();
    }
    uint16_t received = 0;
    uint8_t result = bridgeCall(0x55, (const uint8_t *)name, strlen(name), 1, received);
    bridgeCacheKind = 0;
    if (result != 0)
    {
        bridgeFatal();
    }
    return received == 1 && bridgeBuffer[0] != 0;
}

bool BridgeSd::remove(const char *name)
{
    if (!bridgePcMode())
    {
        return bridgeNativeSD.remove(name);
    }
    if (!bridgeFlush())
    {
        bridgeFatal();
    }
    uint16_t received = 0;
    uint8_t result = bridgeCall(0x56, (const uint8_t *)name, strlen(name), 0, received);
    bridgeCacheKind = 0;
    return result == 0;
}

// Keep the PC-only request buffer out of the native SD call stack.
static BRIDGE_NOINLINE bool bridgeOpenRemote(BridgeSlot &handle, const char *name, int flags)
{
    size_t length = strlen(name);
    if (length == 0 || length > 36)
    {
        return false;
    }
    uint8_t request[37];
    request[0] = handle.writable ? ((flags & O_TRUNC) ? 2 : 1) : 0;
    memcpy(request + 1, name, length);
    uint16_t received = 0;
    if (bridgeCall(0x50, request, (uint16_t)(length + 1), 9, received) != 0 || received != 9)
    {
        return false;
    }
    handle.remoteFile.id = bridgeBuffer[0];
    handle.remoteFile.length = bridgeRead32(bridgeBuffer + 1);
    handle.remoteFile.position = bridgeRead32(bridgeBuffer + 5);
    return true;
}

BRIDGE_NOINLINE BridgeFile BridgeSd::open(const char *name, int flags)
{
    BridgeFile result;
    bool directory = strcmp(name, "/") == 0;
    if (!bridgeFlush())
    {
        bridgeFatal();
    }
    bridgeCacheKind = 0;
    if (directory && bridgePcMode())
    {
        uint16_t received = 0;
        if (bridgeCall(0x57, 0, 0, 2, received) == 0 && received == 2)
        {
            result.kind = 2;
            result.entryCount = bridgeBuffer[0] | (uint16_t)bridgeBuffer[1] << 8;
        }
        return result;
    }
    uint8_t index = 0;
    while (index < 4 && bridgeSlots[index].used)
    {
        ++index;
    }
    if (index == 4)
    {
        return result;
    }
    BridgeSlot &handle = bridgeSlots[index];
    bool writing = flags != FILE_READ;
    handle.prepare(bridgePcMode());
    handle.writable = writing;
    if (!handle.remote)
    {
        // Open directly into the retained handle; no extra File return object.
        if (!bridgeNativeOpen(handle.local, name, flags))
        {
            return result;
        }
    }
    else if (!bridgeOpenRemote(handle, name, flags))
    {
        return result;
    }
    handle.used = true;
    handle.references = 1;
    if (writing)
    {
        ++bridgeWriters;
    }
    result.slot = (int8_t)index;
    result.kind = directory ? 2 : 1;
    return result;
}

// Used only by an explicit SD-list selection, never by a PC backend path.
BRIDGE_NOINLINE BridgeFile BridgeSd::openNativeEntry(BridgeNativeFile &directory, uint16_t index)
{
    BridgeFile result;
    if (bridgePcMode())
        return result;
    uint8_t slot = 0;
    while (slot < 4 && bridgeSlots[slot].used)
        ++slot;
    if (slot == 4)
        return result;
    BridgeSlot &handle = bridgeSlots[slot];
    handle.prepare(false);
    if (!handle.local.open(&directory, index, FILE_READ))
        return result;
    handle.remote = false;
    handle.writable = false;
    handle.used = true;
    handle.references = 1;
    result.slot = (int8_t)slot;
    result.kind = 1;
    return result;
}

#endif
