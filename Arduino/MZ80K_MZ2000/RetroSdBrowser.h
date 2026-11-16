#ifndef RETRO_SD_BROWSER_H
#define RETRO_SD_BROWSER_H

// Root-compatible file browser with optional directory context. Reuse the existing raw upload handle and 256-byte buffer.
// Each operation closes its files before returning to the machine command loop.
typedef decltype(rt_upload_file) RtBrowserFile;

// Keep browser deletion retries separate from SD-upload/read-open retry IDs.
static bool rt_sd_delete_cached = false;
static uint16_t rt_sd_delete_id = 0, rt_sd_delete_crc = 0;
static uint8_t rt_sd_delete_result = 0;

static void rt_browser_put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

// SdFat streams the UTF-8 name through Print. Keep only the requested
// 256-byte window while hashing the entire name, including names >256 bytes.
class RtBrowserName : public Print
{
  public:
    uint32_t hash;
    uint16_t offset, length, count;
    bool overflow;

    explicit RT_HANDLER RtBrowserName(uint16_t start)
        : hash(2166136261UL), offset(start), length(0), count(0), overflow(false)
    {
    }

    size_t write(uint8_t value)
    {
        if (length >= 1024)
        {
            overflow = true;
            return 0;
        }
        hash = (hash ^ value) * 16777619UL;
        if (length >= offset && count < 256)
            RT_BUFFER[count++] = value;
        ++length;
        return 1;
    }

    RT_HANDLER uint32_t signature(uint32_t size) const
    {
        uint32_t result = hash;
        for (uint8_t index = 0; index < 4; ++index)
        {
            result = (result ^ (uint8_t)size) * 16777619UL;
            size >>= 8;
        }
        return result;
    }
};

static size_t rt_browser_print_name(RtBrowserName &name)
{
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000 && (!defined(SDFAT_FILE_TYPE) || SDFAT_FILE_TYPE == 1)
    // Keep UTF-8 names independent of the machine-side ASCII name setting.
    return rt_upload_file.printName8(&name);
#else
    return rt_upload_file.printName(&name);
#endif
}

static RT_HANDLER bool rt_browser_read_name(RtBrowserName &name)
{
    size_t length = rt_browser_print_name(name);
    return length != 0 && length == name.length && !name.overflow;
}

static RT_HANDLER bool rt_browser_busy(bool allowed)
{
    return !allowed || rt_busy || rt_upload || rt_write_left || RT_BROWSER_WRITING || rt_line_active();
}

static RT_HANDLER bool rt_browser_media_ready()
{
    if (RT_BROWSER_READY)
        return true;
    bool ready = rt_sd_begin();
    RT_BROWSER_SET_READY(ready);
    return ready;
}

static bool rt_browser_root(RtBrowserFile &root)
{
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    return root.open(RT_BROWSER_SD.vol(), "/", FILE_READ);
#else
    return root.open("/", FILE_READ);
#endif
}

static RT_HANDLER void rt_browser_reply(uint8_t op, uint16_t id, uint8_t status, uint16_t length)
{
    uint16_t crc = rt_frame_begin((uint8_t)(op | 0x80), id, status, length);
    crc = rt_frame_bytes(crc, RT_BUFFER, length);
    rt_frame_end(crc);
}

static RT_HANDLER void rt_sd_request(uint8_t op,
                                     uint16_t id,
                                     uint16_t length,
                                     bool allowed,
                                     uint16_t requestCrc,
                                     RtBrowserFile *selectedDirectory = 0)
{
    const bool includeDirectories = selectedDirectory != 0;
    const uint8_t replyOp = includeDirectories ? 0x7d : op;
    if (rt_browser_busy(allowed))
    {
        rt_browser_reply(replyOp, id, 0xf7, 0);
        return;
    }
    if (op == 0x7e && RT_BROWSER_DELETE_BUSY)
    {
        rt_browser_reply(replyOp, id, 0xf7, 0);
        return;
    }
    if (op == 0x7f && rt_backend != 0)
    {
        rt_browser_reply(replyOp, id, 0xf4, 0);
        return;
    }
    if (op == 0x7d)
    {
        rt_browser_reply(replyOp, id, 0xf4, 0);
        return;
    }
    // A repeated DELETE must acknowledge the original result, not delete again.
    if (op == 0x7e && rt_sd_delete_cached && id == rt_sd_delete_id)
    {
        uint8_t result = rt_sd_delete_crc == requestCrc ? rt_sd_delete_result : 0xf4;
        rt_browser_reply(replyOp, id, result, 0);
        return;
    }
    if (op != 0x7e)
        rt_sd_delete_cached = false;
    uint8_t status = 0xf4;
    uint16_t responseLength = 0;
    if (op == 0x7a ? length != 4 : (op == 0x7b ? length != 12 : (op == 0x7c ? length != 16 : length != 10)))
    {
        rt_browser_reply(replyOp, id, status, 0);
        return;
    }
    uint32_t cursor = rt_u32(RT_BUFFER);
    uint16_t index = (uint16_t)(RT_BUFFER[0] | (uint16_t)RT_BUFFER[1] << 8);
    uint32_t expectedSize = op == 0x7a ? 0 : rt_u32(RT_BUFFER + 2);
    uint32_t expectedSignature = op == 0x7a ? 0 : rt_u32(RT_BUFFER + 6);
    uint16_t nameOffset = op == 0x7b ? (uint16_t)(RT_BUFFER[10] | (uint16_t)RT_BUFFER[11] << 8) : 0;
    uint32_t offset = op == 0x7c ? rt_u32(RT_BUFFER + 10) : 0;
    uint16_t count = op == 0x7c ? (uint16_t)(RT_BUFFER[14] | (uint16_t)RT_BUFFER[15] << 8) : 0;
    if (op == 0x7c && (count == 0 || count > 256 || offset > expectedSize))
    {
        rt_browser_reply(replyOp, id, status, 0);
        return;
    }
    if (!rt_browser_media_ready())
    {
        rt_browser_reply(replyOp, id, 0xf2, 0);
        return;
    }
    rt_upload_file.close();
    rt_card_error_clear();
    RtBrowserFile localRoot;
    RtBrowserFile &root = selectedDirectory ? *selectedDirectory : localRoot;
    if (!selectedDirectory && !rt_browser_root(root))
    {
        RT_BROWSER_SET_READY(false);
        rt_browser_reply(replyOp, id, 0xf2, 0);
        return;
    }
    if (op == 0x7a)
    {
        status = 0xf1;
        if (root.seek(cursor))
        {
            while (rt_upload_file.openNext(&root, FILE_READ))
            {
                if (includeDirectories || !rt_upload_file.isDir())
                {
                    index = rt_upload_file.dirIndex();
                    uint32_t size = rt_upload_file.isDir() ? 0 : rt_upload_file.size();
                    RtBrowserName name(65535);
                    if (!rt_browser_read_name(name))
                    {
                        status = 0xf2;
                        break;
                    }
                    uint32_t signature = name.signature(size);
                    rt_browser_put32(RT_BUFFER, root.position());
                    RT_BUFFER[4] = (uint8_t)index;
                    RT_BUFFER[5] = (uint8_t)(index >> 8);
                    rt_browser_put32(RT_BUFFER + 6, size);
                    rt_browser_put32(RT_BUFFER + 10, signature);
                    RT_BUFFER[14] = (uint8_t)name.length;
                    RT_BUFFER[15] = (uint8_t)(name.length >> 8);
                    RT_BUFFER[16] = rt_upload_file.isDir() ? 1 : 0;
                    responseLength = includeDirectories ? 17 : 16;
                    status = 0;
                    break;
                }
                rt_upload_file.close();
            }
        }
        if (status == 0xf1 && RT_BROWSER_SD.sdErrorCode() != 0)
            status = 0xf2;
    }
    else if (rt_upload_file.open(&root, index, op == 0x7e ? O_WRONLY : FILE_READ) &&
             (!rt_upload_file.isDir() || (includeDirectories && op == 0x7b)))
    {
        status = 0xf2;
        RtBrowserName name(op == 0x7b ? nameOffset : 65535);
        if (rt_browser_read_name(name))
        {
            if ((rt_upload_file.isDir() ? 0 : rt_upload_file.size()) != expectedSize ||
                name.signature(expectedSize) != expectedSignature)
                status = 0xf6; // Entry changed since listing: require a fresh list.
            else if (op == 0x7b)
            {
                responseLength = name.count;
                status = nameOffset < name.length ? 0 : 0xf4;
            }
            else if (op == 0x7c)
            {
                if (count > expectedSize - offset)
                    count = (uint16_t)(expectedSize - offset);
                if (rt_upload_file.seek(offset))
                {
                    int received = rt_upload_file.read(RT_BUFFER, count);
                    if (received == count)
                    {
                        responseLength = count;
                        status = 0;
                    }
                }
            }
            else if (op == 0x7e)
                status = rt_upload_file.remove() ? 0 : 0xf2;
            else if (op == 0x7f)
            {
                rt_upload_file.close();
#ifdef RT_BROWSER_PREPARE_READ
#ifdef RT_BROWSER_PREPARE_READ
#ifdef RT_BROWSER_PREPARE_READ
                status = RT_BROWSER_PREPARE_READ(root, index);
#else
                status = 0xf4;
#endif
#else
                status = 0xf4;
#endif
#else
                status = 0xf4;
#endif
            }
        }
    }
    else
        status = 0xf1;
    rt_upload_file.close();
    root.close();
    if (status != 0 && RT_BROWSER_SD.sdErrorCode() != 0)
        RT_BROWSER_SET_READY(false);
    if (op == 0x7e)
    {
        rt_sd_delete_cached = true;
        rt_sd_delete_id = id;
        rt_sd_delete_crc = requestCrc;
        rt_sd_delete_result = status;
    }
    rt_browser_reply(replyOp, id, status, status == 0 ? responseLength : 0);
}
#endif
