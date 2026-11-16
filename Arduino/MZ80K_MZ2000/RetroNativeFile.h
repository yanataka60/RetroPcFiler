#ifndef RETRO_NATIVE_FILE_H
#define RETRO_NATIVE_FILE_H

// FAT needs no Arduino Stream/Print state in each open file.
// Keep the original File operations, without retaining unused virtual methods.
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000 && SDFAT_FILE_TYPE == 1
#if defined(__AVR__)
#define MZ_FILE_SHARED __attribute__((noinline, noclone))
#else
#define MZ_FILE_SHARED
#endif
class MZNativeFile : public FatFile
{
  public:
    using FatFile::write;

    MZ_FILE_SHARED size_t write(uint8_t value)
    {
        return FatFile::write(value);
    }

    MZNativeFile()
    {
    }

    MZNativeFile(const MZNativeFile &other)
    {
        copy(&other);
    }

    MZNativeFile &operator=(const MZNativeFile &other)
    {
        if (this != &other)
        {
            close();
            copy(&other);
        }
        return *this;
    }

    uint32_t size() const
    {
        return fileSize();
    }

    bool seek(uint32_t at)
    {
        return seekSet(at);
    }

    uint32_t position() const
    {
        return curPosition();
    }

    void rewindDirectory()
    {
        rewind();
    }

    MZNativeFile openNextFile()
    {
        MZNativeFile next;
        next.openNext(this, FILE_READ);
        return next;
    }
};
#else
typedef File MZNativeFile;
#endif
#endif
