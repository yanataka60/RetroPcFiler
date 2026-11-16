#ifndef RETRO_SD_FOLDERS_H
#define RETRO_SD_FOLDERS_H

// All directory paths are supplied by Windows as index/signature pairs.
// The machine protocol and the native working directory are never changed.
// At most 16 nested directories; no persistent path or transfer buffer is added.
static bool rt_folder_cached = false;
static uint16_t rt_folder_id = 0, rt_folder_crc = 0;
static uint8_t rt_folder_result = 0;

static bool rt_folder_path(uint16_t &at, uint16_t length, uint8_t &depth)
{
    if (at >= length)
        return false;
    depth = RT_BUFFER[at++];
    if (depth > 16 || at + (uint16_t)depth * 6 > length)
        return false;
    at += (uint16_t)depth * 6;
    return true;
}

static uint8_t rt_folder_open(RtBrowserFile &directory, uint16_t pathAt, uint8_t depth)
{
    if (!rt_browser_root(directory))
        return 0xf2;
    for (uint8_t level = 0; level < depth; ++level)
    {
        uint16_t index = RT_BUFFER[pathAt] | (uint16_t)RT_BUFFER[pathAt + 1] << 8;
        uint32_t signature = rt_u32(RT_BUFFER + pathAt + 2);
        pathAt += 6;
        rt_upload_file.close();
        if (!rt_upload_file.open(&directory, index, FILE_READ) || !rt_upload_file.isDir())
            return 0xf6;
        RtBrowserName name(65535);
        if (!rt_browser_print_name(name) || name.overflow || name.signature(0) != signature)
            return 0xf6;
        // Alternating existing handles avoids a path string and recursion.
        directory.close();
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000 && SDFAT_FILE_TYPE == 1
        directory.copy(&rt_upload_file);
#else
        directory = rt_upload_file;
#endif
        rt_upload_file.close();
    }
    return 0;
}

static bool rt_folder_valid_name(const uint8_t *name, uint16_t length)
{
    if (!length || length > 255 || name[length - 1] == ' ' || name[length - 1] == '.')
        return false;
    for (uint16_t index = 0; index < length; ++index)
    {
        uint8_t value = name[index];
        if (value < 32 || value == 127 || strchr("\\/:*?\"<>|", value))
            return false;
#if !defined(USE_UTF8_LONG_NAMES) || !USE_UTF8_LONG_NAMES
        // SdFat's default name writer cannot create UTF-8 LFNs. Never corrupt
        // such names by silently interpreting their bytes as separate letters.
        if (value >= 128)
            return false;
#endif
    }
    return true;
}

static uint8_t rt_folder_transfer(uint8_t command, uint16_t length)
{
    uint16_t at = 1;
    uint8_t sourceDepth, targetDepth;
    if (!rt_folder_path(at, length, sourceDepth))
        return 0xf4;
    uint16_t sourcePath = 2;
    if (at + 10 >= length)
        return 0xf4;
    uint16_t index = RT_BUFFER[at] | (uint16_t)RT_BUFFER[at + 1] << 8;
    uint32_t size = rt_u32(RT_BUFFER + at + 2);
    uint32_t signature = rt_u32(RT_BUFFER + at + 6);
    at += 10;
    uint16_t targetPath = at + 1;
    if (!rt_folder_path(at, length, targetDepth))
        return 0xf4;
    uint32_t offset = 0;
    if (command == 1)
    {
        if (at + 4 != length)
            return 0xf4;
        offset = rt_u32(RT_BUFFER + at);
        if (offset > size || (offset % 256 != 0 && offset != size))
            return 0xf4;
    }
    else if (at != length)
        return 0xf4;
    if (sourceDepth == targetDepth &&
        memcmp(RT_BUFFER + sourcePath, RT_BUFFER + targetPath, (uint16_t)sourceDepth * 6) == 0)
        return 0xf5;

    RtBrowserFile sourceDirectory, targetDirectory, destination;
    uint8_t result = rt_folder_open(sourceDirectory, sourcePath, sourceDepth);
    if (!result)
        result = rt_folder_open(targetDirectory, targetPath, targetDepth);
    rt_upload_file.close();
    if (!result && (!rt_upload_file.open(&sourceDirectory, index, FILE_READ) || rt_upload_file.isDir()))
        result = 0xf6;
    if (!result)
    {
        RtBrowserName name(0);
        size_t printed = rt_browser_print_name(name);
        if (!printed || printed != name.length || name.overflow || rt_upload_file.size() != size ||
            name.signature(size) != signature)
            result = 0xf6;
        else if (!rt_folder_valid_name(RT_BUFFER, name.length))
            result = 0xf9;
        else
        {
            RT_BUFFER[name.length] = 0;
            // Both native rename and exclusive creation refuse existing names.
            if (destination.open(&targetDirectory, (char *)RT_BUFFER, FILE_READ))
                result = 0xf5;
            destination.close();
            if (!result && command == 2)
            {
                if (!destination.open(&targetDirectory, "RPCOPY.$$$", O_WRONLY) ||
                    destination.size() != size || !destination.sync())
                    result = 0xf2;
                else
                    result = destination.rename(&targetDirectory, (char *)RT_BUFFER) ? 0 : 0xf2;
            }
            else if (!result)
            {
                // A leftover reserved work file is removed before a fresh copy.
                if (offset == 0 && destination.open(&targetDirectory, "RPCOPY.$$$", O_WRONLY))
                {
                    if (!destination.remove())
                        result = 0xf2;
                    destination.close();
                }
                if (!result && !destination.open(&targetDirectory,
                                                 "RPCOPY.$$$",
                                                 offset == 0 ? O_WRONLY | O_CREAT | O_EXCL : O_WRONLY))
                    result = 0xf2;
                if (!result && (destination.size() < offset || !destination.seek(offset) ||
                                !rt_upload_file.seek(offset)))
                    result = 0xf2;
                uint16_t count = size - offset > 256 ? 256 : (uint16_t)(size - offset);
                if (!result && (rt_upload_file.read(RT_BUFFER, count) != count ||
                                destination.write(RT_BUFFER, count) != count || !destination.sync()))
                    result = 0xf2;
            }
        }
    }
    destination.close();
    rt_upload_file.close();
    targetDirectory.close();
    sourceDirectory.close();
    return result;
}

static RT_HANDLER void rt_folder_request(uint16_t id, uint16_t length, bool allowed, uint16_t crc)
{
    uint8_t result = 0xf4;
    if (!allowed || rt_busy || rt_upload || rt_write_left || RT_BROWSER_WRITING || rt_line_active())
    {
        rt_browser_reply(0x7d, id, 0xf7, 0);
        return;
    }
    if (length < 2)
    {
        rt_browser_reply(0x7d, id, result, 0);
        return;
    }
    uint8_t command = RT_BUFFER[0];
    if (command > 5 || command == 3)
    {
        rt_browser_reply(0x7d, id, result, 0);
        return;
    }
    if (command != 0 && RT_BROWSER_DELETE_BUSY)
    {
        rt_browser_reply(0x7d, id, 0xf7, 0);
        return;
    }
    bool mutation = command >= 2;
    if (mutation && rt_folder_cached && id == rt_folder_id)
    {
        rt_browser_reply(0x7d, id, crc == rt_folder_crc ? rt_folder_result : 0xf4, 0);
        return;
    }
    rt_folder_cached = false;
    if (command != 0)
        rt_sd_delete_cached = false;
    if (!RT_BROWSER_READY)
    {
        bool ready = rt_sd_begin();
        RT_BROWSER_SET_READY(ready);
        if (!ready)
        {
            rt_browser_reply(0x7d, id, 0xf2, 0);
            return;
        }
    }
    rt_card_error_clear();
    rt_upload_file.close();
    if (command == 0)
    {
        uint8_t inner = RT_BUFFER[1], depth;
        uint16_t at = 2;
        if (inner >= 0x7a && inner <= 0x7f && inner != 0x7d && rt_folder_path(at, length, depth))
        {
            RtBrowserFile directory;
            result = rt_folder_open(directory, 3, depth);
            if (!result)
            {
                memmove(RT_BUFFER, RT_BUFFER + at, length - at);
                rt_sd_request(inner, id, length - at, allowed, crc, &directory, true, 0x7d);
                directory.close();
                return;
            }
            directory.close();
        }
    }
    else if (command <= 2)
        result = rt_folder_transfer(command, length);
    else
    {
        uint16_t at = 1;
        uint8_t depth;
        if (rt_folder_path(at, length, depth) && (command == 4 ? at < length : at == length))
        {
            RtBrowserFile directory, file;
            result = rt_folder_open(directory, 2, depth);
            rt_upload_file.close();
            if (!result && command == 4)
            {
                uint16_t nameLength = length - at;
                if (!rt_folder_valid_name(RT_BUFFER + at, nameLength))
                    result = 0xf9;
                else
                {
                    memmove(RT_BUFFER, RT_BUFFER + at, nameLength);
                    RT_BUFFER[nameLength] = 0;
                    if (file.open(&directory, (char *)RT_BUFFER, FILE_READ))
                        result = 0xf5;
                    else
                        result = file.mkdir(&directory, (char *)RT_BUFFER, false) ? 0 : 0xf2;
                }
            }
            else if (!result && file.open(&directory, "RPCOPY.$$$", O_WRONLY))
                result = file.remove() ? 0 : 0xf2;
            file.close();
            directory.close();
        }
    }
    rt_upload_file.close();
    if (mutation)
    {
        rt_folder_cached = true;
        rt_folder_id = id;
        rt_folder_crc = crc;
        rt_folder_result = result;
    }
    if (result && RT_BROWSER_SD.sdErrorCode())
        RT_BROWSER_SET_READY(false);
    rt_browser_reply(0x7d, id, result, 0);
}
#endif
