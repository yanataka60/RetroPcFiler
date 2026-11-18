// Runtime SD / PC backend for MSX; original machine protocol is retained.
#include <Arduino.h>
#define RETRO_PROFILE 8
#define RT_PROFILE 8
#define RETRO_IDLE_LEVEL LOW
#define CHKPIN 15
#define RT_BUFFER bridgeBuffer
#include "RetroStorage.h"
#include "RetroUart.h"
// 2024. 3.13 sd-card再挿入時の初期化処理を追加
//
#include "SdFat.h"
#include <SPI.h>
BridgeSd SD;
unsigned long r_count = 0;
unsigned long f_length = 0;
char m_name[40];
char f_name[40];
char w_name[40];
char c_name[40];
// Store directory entry indexes instead of ten filename copies.
uint16_t sdir[10];
uint16_t sdirValid = 0;
BridgeFile file_r, file_w;
unsigned int s_adrs, e_adrs, g_adrs, s_adrs1, s_adrs2;
boolean eflg;

#define CABLESELECTPIN (10)
#define PB0PIN (2)
#define PB1PIN (3)
#define PB2PIN (4)
#define PB3PIN (5)
#define PB4PIN (6)
#define PB5PIN (7)
#define PB6PIN (8)
#define PB7PIN (9)
#define FLGPIN (14)
#define PA0PIN (16)
#define PA1PIN (17)
#define PA2PIN (18)
#define PA3PIN (19)

// ファイル名は、ロングファイルネーム形式対応

// SdFat 1.x accepts an SPI divider; SdFat 2.x accepts a frequency in Hz.
// Keep command-local buffers out of the idle polling stack under AVR LTO.
#if defined(__AVR__)
#define RT_HANDLER __attribute__((noinline, noclone))
#else
#define RT_HANDLER
#endif

static bool rt_sd_begin()
{
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    return bridgeNativeSD.begin(CABLESELECTPIN, (uint32_t)F_CPU / 8UL);
#else
    return bridgeNativeSD.begin(CABLESELECTPIN, 8);
#endif
}

// Runtime SD/PC selection v3. Starts in SD on every reset; no EEPROM fallback.
// RT_PROFILE=0: MZ2500 CHK idle HIGH; =1: MZ80K/MZ2000 idle LOW.
static uint8_t rt_backend = 0;
// Host -> Arduino SD copy. Uses the existing transfer buffer only while idle.
static BridgeNativeFile rt_upload_file;
static bool rt_upload = false, rt_temp_owned = false, rt_replace = false;
static bool rt_diag_requested = false;
static char rt_upload_name[37];
static uint32_t rt_upload_size = 0, rt_upload_at = 0;
static unsigned long rt_upload_tick = 0;
static bool rt_cached = false;
static uint16_t rt_cached_id = 0, rt_cached_crc = 0;
static uint8_t rt_cached_op = 0, rt_cached_result = 0;
static const char rt_temp_name[] = "MZPC_TMP.$$$", rt_backup_name[] = "MZPC_BAK.$$$";
static bool rt_busy = false;
static uint32_t rt_write_left = 0;
void sdinit(void);
static bool rt_line_active();

static uint32_t rt_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

// Capture the failing operation before close, rollback or diagnostics change it.
static uint8_t rt_fault_code = 0, rt_fault_data = 0, rt_fault_stage = 0;
#if defined(__AVR__)
extern "C"
{
    extern char __heap_start;
    extern void *__brkval;
}
static uint8_t *rt_ram_end = 0;

static uint8_t *rt_heap_end()
{
    return (uint8_t *)(__brkval ? __brkval : &__heap_start);
}
#endif

static void rt_ram_mark()
{
#if defined(__AVR__)
    // Paint only unused RAM below the active stack, with a safety margin.
    uint8_t saved = SREG;
    noInterrupts();
    uint8_t *begin = rt_heap_end();
    uintptr_t stack = SP;
    rt_ram_end = stack > 32 ? (uint8_t *)(stack - 32) : begin;
    for (uint8_t *p = begin; p < rt_ram_end; ++p)
    {
        *p = 0xa5;
    }
    SREG = saved;
#endif
}

static uint16_t rt_ram_remaining()
{
#if defined(__AVR__)
    uint8_t *begin = rt_heap_end();
    if (!rt_ram_end || begin >= rt_ram_end)
    {
        return 0;
    }
    uint8_t *p = begin;
    while (p < rt_ram_end && *p == 0xa5)
    {
        ++p;
    }
    return (uint16_t)(p - begin);
#else
    return 0xffff;
#endif
}

static void rt_card_error_clear()
{
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000 && !HAS_SDIO_CLASS
    if (bridgeNativeSD.card())
    {
        bridgeNativeSD.card()->error(0);
    }
#endif
}

static void rt_fault_capture(uint8_t stage)
{
    rt_fault_stage = stage;
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    rt_fault_code = bridgeNativeSD.sdErrorCode();
    rt_fault_data = bridgeNativeSD.sdErrorData();
#endif
}

static bool rt_rename(const char *from, const char *to)
{
    rt_card_error_clear();
    BridgeNativeFile f;
    bridgeNativeOpen(f, from, O_WRONLY);
    if (!f)
    {
        rt_fault_capture(3);
        return false;
    }
    rt_card_error_clear();
    bool ok = f.rename(to);
    if (!ok)
    {
        rt_fault_capture(4);
    }
    f.close();
    return ok;
}

static void rt_upload_abort()
{
    if (rt_upload)
        rt_upload_file.close();
    rt_upload = false;
    if (rt_temp_owned)
    {
        bridgeNativeSD.remove(rt_temp_name);
        rt_temp_owned = false;
    }
}

static bool rt_valid_name(const char *s)
{
    if (!*s || strcmp(s, ".") == 0 || strcmp(s, "..") == 0)
        return false;
    uint8_t n = 0;
    char last = 0;
    while (*s)
    {
        uint8_t c = (uint8_t)*s++;
        if (c < 32 || c > 126 || strchr("\\/:*?\"<>|", c))
            return false;
        last = c;
        if (++n > 36)
            return false;
    }
    if (last == ' ' || last == '.')
        return false;
    // Reserve transaction files, irrespective of case.
    const char *names[2] = {rt_temp_name, rt_backup_name};
    for (uint8_t k = 0; k < 2; k++)
    {
        const char *a = rt_upload_name;
        const char *b = names[k];
        while (*a && *b)
        {
            char x = *a++, y = *b++;
            if (x >= 'a' && x <= 'z')
                x -= 32;
            if (x != y)
                break;
            if (!*a && !*b)
                return false;
        }
    }
    return true;
}

static uint8_t rt_upload_request(uint8_t op, uint16_t n, bool allowed)
{
    if (!allowed || rt_busy || rt_write_left || bridgeWriters || bridgeSplitLeft || rt_line_active())
        return 0xf7;
    uint8_t *p = RT_BUFFER;
    if (op == 0x76)
    {
        if (n)
            return 0xf4;
        rt_upload_abort();
        return 0;
    }
    if (op == 0x73)
    {
        if (rt_upload)
            return 0xf7;
        if (n < 6 || n > 41 || (p[0] & 0x7e) != 0)
            return 0xf4;
        rt_diag_requested = (p[0] & 0x80) != 0;
        rt_fault_code = rt_fault_data = rt_fault_stage = 0;
        rt_ram_mark();
        uint8_t count = (uint8_t)(n - 5);
        memcpy(rt_upload_name, p + 5, count);
        rt_upload_name[count] = 0;
        if (memchr(p + 5, 0, count) || !rt_valid_name(rt_upload_name))
            return 0xf4;
        bridgeSdReady = rt_sd_begin();
        if (!bridgeSdReady)
            return 0xf2;
        // A previous attempt may have left the disposable staging file.
        // Recreate it for this new request; never remove the backup here.
        if (bridgeNativeSD.exists(rt_temp_name))
        {
            rt_card_error_clear();
            if (!bridgeNativeSD.remove(rt_temp_name))
            {
                rt_fault_capture(7);
                return 0xef;
            }
        }
        if (bridgeNativeSD.exists(rt_backup_name))
        {
            return 0xf9;
        }
        rt_replace = (p[0] & 1) != 0;
        if (!rt_replace && bridgeNativeSD.exists(rt_upload_name))
            return 0xf6;
        rt_upload_size = rt_u32(p + 1);
        rt_upload_at = 0;
        bridgeNativeOpen(rt_upload_file, rt_temp_name, O_WRONLY | O_CREAT | O_TRUNC);
        if (!rt_upload_file)
            return 0xf2;
        rt_temp_owned = true;
        rt_upload = true;
        rt_upload_tick = millis();
        return 0;
    }
    if (!rt_upload)
        return 0xf4;
    rt_upload_tick = millis();
    if (op == 0x74)
    {
        if (n < 5 || rt_u32(p) != rt_upload_at || n - 4 > rt_upload_size - rt_upload_at)
            return 0xf4;
        if (rt_upload_file.write(p + 4, n - 4) != (size_t)(n - 4))
        {
            rt_upload_abort();
            return 0xfa;
        }
        rt_upload_at += n - 4;
        return 0;
    }
    if (op == 0x75)
    {
        if (n || rt_upload_at != rt_upload_size)
            return 0xf4;
        rt_card_error_clear();
        if (!rt_upload_file.sync())
        {
            rt_fault_capture(1);
            rt_upload_file.close();
            rt_upload = false;
            rt_temp_owned = false;
            return 0xfb;
        }
        // Keep the synced temporary file open for rename. Avoid reopening it
        // and allocating an extra File on the rename call stack.
        bool backup = false;
        rt_card_error_clear();
        bool destinationExists = bridgeNativeSD.exists(rt_upload_name);
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
        if (bridgeNativeSD.sdErrorCode() != 0)
        {
            rt_fault_capture(2);
            rt_upload_file.close();
            rt_upload = false;
            rt_temp_owned = false;
            return 0xfd;
        }
#endif
        if (destinationExists)
        {
            if (!rt_replace)
            {
                rt_upload_abort();
                return 0xf6;
            }
            if (!rt_rename(rt_upload_name, rt_backup_name))
            {
                rt_upload_file.close();
                rt_upload = false;
                rt_temp_owned = false;
                return 0xfc;
            }
            backup = true;
        }
        rt_card_error_clear();
        if (!rt_upload_file.rename(rt_upload_name))
        {
            rt_fault_capture(5);
            rt_upload_file.close();
            uint8_t originalCode = rt_fault_code;
            uint8_t originalData = rt_fault_data;
            bool restored = !backup || rt_rename(rt_backup_name, rt_upload_name);
            if (restored)
            {
                rt_fault_code = originalCode;
                rt_fault_data = originalData;
                rt_fault_stage = 5;
            }
            else
            {
                rt_fault_stage = 6;
            }
            rt_upload = false;
            rt_temp_owned = false;
            return restored ? 0xfd : 0xfe;
        }
        rt_upload_file.close();
        rt_temp_owned = false;
        rt_upload = false;
        if (backup)
            bridgeNativeSD.remove(rt_backup_name); // On failure leave the old file recoverable.
        return 0;
    }
    return 0xf4;
}

static bool rt_rx_allowed = false;

static uint8_t rt_rx_state = 0, rt_header[7], rt_header_at = 0, rt_value = 0, rt_crc_low = 0;
static uint16_t rt_rx_length = 0, rt_rx_at = 0, rt_rx_crc = 0xffff;
static unsigned long rt_last_rx = 0;
void sdinit(void);

static uint16_t rt_crc_byte(uint16_t c, uint8_t v)
{
    c ^= (uint16_t)v << 8;
    for (uint8_t i = 0; i < 8; i++)
        c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    return c;
}

static RT_HANDLER uint16_t rt_write_crc(uint16_t crc, uint8_t value)
{
    Serial.write(value);
    return rt_crc_byte(crc, value);
}

static RT_HANDLER uint16_t rt_frame_bytes(uint16_t crc, const uint8_t *data, uint16_t length)
{
    for (uint16_t index = 0; index < length; ++index)
        crc = rt_write_crc(crc, data[index]);
    return crc;
}

static RT_HANDLER uint16_t rt_frame_begin(uint8_t op, uint16_t id, uint8_t status, uint16_t length)
{
    uint8_t header[7] = {1,      (uint8_t)id,     (uint8_t)(id >> 8),    op,
                         status, (uint8_t)length, (uint8_t)(length >> 8)};
    Serial.write((uint8_t)0x4d);
    Serial.write((uint8_t)0x5a);
    return rt_frame_bytes(0xffff, header, sizeof(header));
}

static RT_HANDLER void rt_frame_end(uint16_t crc)
{
    Serial.write((uint8_t)crc);
    Serial.write((uint8_t)(crc >> 8));
}

static bool rt_line_active()
{
    return digitalRead(CHKPIN) != (RETRO_IDLE_LEVEL);
}

static uint8_t rt_flags()
{
    return (rt_busy || rt_line_active() ? 1 : 0) |
           ((rt_write_left || bridgeWriters || bridgeSplitLeft) ? 2 : 0) | (bridgeSdReady ? 4 : 0) |
           (rt_upload ? 8 : 0);
}

static void rt_emit(uint8_t op, uint16_t id, uint8_t status)
{
    bool diagnostic = rt_diag_requested && (op == 0xf3 || op == 0xf4 || op == 0xf5) &&
                      (status == 0xef || status == 0xf6 || (status >= 0xf8 && status <= 0xfe));
    uint8_t error = 0, detail = 0;
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    error = bridgeNativeSD.sdErrorCode();
    detail = bridgeNativeSD.sdErrorData();
#endif
    if (status == 0xef || (status >= 0xfb && status <= 0xfe))
    {
        error = rt_fault_code;
        detail = rt_fault_data;
    }
    uint16_t ram = diagnostic ? rt_ram_remaining() : 0xffff;
    char *actualName = (char *)RT_BUFFER;
    if (diagnostic)
    {
        actualName[0] = 0;
        const char *subject = (status == 0xef || status == 0xf8) ? rt_temp_name
                              : status == 0xf9                   ? rt_backup_name
                                                                 : rt_upload_name;
        // Error replies have no pending data block. Reuse the existing
        // file object and transfer buffer to avoid another AVR stack buffer.
        if (status == 0xef || status == 0xf6 || status == 0xf8 || status == 0xf9)
        {
            bridgeNativeOpen(rt_upload_file, subject, FILE_READ);
            if (rt_upload_file)
            {
                rt_upload_file.getName(actualName, 37);
                rt_upload_file.close();
            }
        }
    }
    uint8_t uploadNameLength = diagnostic ? (uint8_t)strlen(rt_upload_name) : 0;
    uint8_t actualNameLength = diagnostic ? (uint8_t)strlen(actualName) : 0;
    uint16_t length = diagnostic ? (uint16_t)(19 + uploadNameLength + actualNameLength) : 4;
    uint8_t state[4] = {rt_backend, RT_PROFILE, rt_flags(), 127};
    uint16_t crc = rt_frame_begin(op, id, status, length);
    crc = rt_frame_bytes(crc, state, sizeof(state));
    if (diagnostic)
    {
        crc = rt_write_crc(crc, 3);
        crc = rt_frame_bytes(crc, (const uint8_t *)rt_upload_name, uploadNameLength + 1);
        crc = rt_frame_bytes(crc, (const uint8_t *)actualName, actualNameLength + 1);
    }
    if (diagnostic)
    {
        uint8_t fat = 0;
        uint16_t total = 0, freeEntries = 0xffff, longest = 0xffff;
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
        fat = bridgeNativeSD.fatType();
#if SDFAT_FILE_TYPE == 1
        if (fat == 12 || fat == 16)
        {
            total = bridgeNativeSD.rootDirEntryCount();
            bridgeNativeOpen(rt_upload_file, "/", FILE_READ);
            if (rt_upload_file)
            {
                uint16_t count = 0, run = 0;
                freeEntries = 0;
                longest = 0;
                while (count < total && rt_upload_file.read(RT_BUFFER, 32) == 32)
                {
                    ++count;
                    if (RT_BUFFER[0] == 0 || RT_BUFFER[0] == 0xe5)
                    {
                        ++freeEntries;
                        ++run;
                        if (run > longest)
                        {
                            longest = run;
                        }
                    }
                    else
                    {
                        run = 0;
                    }
                }
                if (count != total)
                {
                    freeEntries = longest = 0xffff;
                }
                rt_upload_file.close();
            }
        }
#endif
#endif
        uint8_t fields[9] = {fat,
                             error,
                             detail,
                             (uint8_t)total,
                             (uint8_t)(total >> 8),
                             (uint8_t)freeEntries,
                             (uint8_t)(freeEntries >> 8),
                             (uint8_t)longest,
                             (uint8_t)(longest >> 8)};
        crc = rt_frame_bytes(crc, fields, sizeof(fields));
        uint8_t extra[3] = {rt_fault_stage, (uint8_t)ram, (uint8_t)(ram >> 8)};
        crc = rt_frame_bytes(crc, extra, sizeof(extra));
    }
    rt_frame_end(crc);
}

static void rt_restart_request(uint16_t id)
{
#if defined(__AVR__)
    // Explicit user-requested application restart. No watchdog is enabled,
    // avoiding the watchdog boot loop of older Pro Mini bootloaders.
    rt_upload_abort();
    rt_emit(0xf8, id, 0);
    Serial.flush();
    Serial.end();
    digitalWrite(CABLESELECTPIN, HIGH);
    SPI.end();
    digitalWrite(FLGPIN, LOW);
    noInterrupts();
    asm volatile("jmp 0");
    for (;;)
    {
    }
#else
    rt_emit(0xf8, id, 0xf4);
#endif
}

static void rt_handle(uint8_t op, uint16_t id, uint8_t value, bool allowSwitch)
{
    if (op == 0x78)
    {
        rt_restart_request(id);
        return;
    }
    uint8_t result = 0;
    if (op == 0x71 && !rt_upload)
    {
        rt_cached = false;
    }
    if (op == 0x70)
    {
        if (value > 1)
        {
            result = 0xf4;
        }
        else if (!allowSwitch || rt_busy || rt_upload || rt_write_left || bridgeWriters || bridgeSplitLeft ||
                 rt_line_active())
        {
            result = 0xf7;
        }
        else
        {
            bridgeReset();
            rt_backend = value;
            if (value == 0)
            {
                sdinit();
            }
            else
            {
                result = bridgeHandshake();
                if (result != 0)
                {
                    rt_backend = 0;
                    sdinit();
                }
                else
                {
                    eflg = false;
                }
            }
        }
    }
    rt_emit(op | 0x80, id, result);
}

static RT_HANDLER uint8_t msx_open_read(const char *name)
{
    if (!SD.exists(name))
        return 0xf1;
    file_r = SD.open(name, FILE_READ);
    if (!file_r)
        return 0xf2;
    f_length = file_r.size();
    r_count = 0;
    return 0;
}

// Windows PC-list selection, while the machine-side protocol is idle.
static uint8_t rt_pc_open_request(uint16_t length, bool allowed)
{
    if (!allowed || rt_busy || rt_upload || rt_write_left || bridgeWriters || bridgeSplitLeft ||
        rt_line_active())
        return 0xf7;
    if (rt_backend != 1 || length < 5 || length > 31)
        return 0xf4;
    for (uint16_t index = 0; index < length; ++index)
    {
        uint8_t value = RT_BUFFER[index];
        if (value < 32 || value > 126 || strchr("\\/:*?\"<>|", value))
            return 0xf4;
    }
    // Nested PC file RPCs reuse RT_BUFFER, so retain the selected name first.
    memcpy(m_name, RT_BUFFER, length);
    m_name[length] = 0;
    const char *extension = m_name + length - 4;
    if (extension[0] != '.' || (extension[1] != 'C' && extension[1] != 'c') ||
        (extension[2] != 'A' && extension[2] != 'a') || (extension[3] != 'S' && extension[3] != 's'))
        return 0xf4;
    rt_busy = true;
    uint8_t result = msx_open_read(m_name);
    rt_busy = false;
    return result;
}

#define RT_BROWSER_SD bridgeNativeSD
#define RT_BROWSER_READY bridgeSdReady
#define RT_BROWSER_SET_READY(value)                                                                          \
    do                                                                                                       \
    {                                                                                                        \
        bridgeSdReady = (value);                                                                             \
        if (rt_backend == 0)                                                                                 \
            eflg = !(value);                                                                                 \
    } while (0)
#define RT_BROWSER_WRITING (bridgeWriters || bridgeSplitLeft)
#define RT_BROWSER_DELETE_BUSY                                                                               \
    (rt_backend == 0 &&                                                                                      \
     (bridgeSlots[0].used || bridgeSlots[1].used || bridgeSlots[2].used || bridgeSlots[3].used))
static uint8_t rt_prepare_sd_read(BridgeNativeFile &directory, uint16_t index);
#define RT_BROWSER_PREPARE_READ(directory, index) rt_prepare_sd_read(directory, index)
#include "RetroSdBrowser.h"
#include "RetroSdFolders.h"

static uint8_t rt_prepare_sd_read(BridgeNativeFile &directory, uint16_t index)
{
    BridgeFile selected = SD.openNativeEntry(directory, index);
    if (!selected)
        return 0xf0;
    file_r = selected;
    f_length = file_r.size();
    r_count = 0;
    return 0;
}

// Observe every received UART byte, including normal RPC replies. Entire
// non-control frames are skipped, so magic bytes inside file data cannot switch.
static void rt_feed(uint8_t v, bool allowSwitch)
{
    unsigned long now = millis();
    if ((unsigned long)(now - rt_last_rx) > 500)
        rt_rx_state = 0;
    rt_last_rx = now;
    if (rt_rx_state == 0)
    {
        if (v == 0x4d)
        {
            rt_rx_state = 1;
            rt_rx_allowed = allowSwitch && !rt_busy;
        }
        return;
    }
    rt_rx_allowed = rt_rx_allowed && allowSwitch && !rt_busy;
    if (rt_rx_state == 1)
    {
        rt_rx_state = v == 0x5a ? 2 : (v == 0x4d ? 1 : 0);
        rt_header_at = 0;
        rt_rx_crc = 0xffff;
        return;
    }
    if (rt_rx_state == 2)
    {
        rt_header[rt_header_at++] = v;
        rt_rx_crc = rt_crc_byte(rt_rx_crc, v);
        if (rt_header_at < 7)
            return;
        rt_rx_length = rt_header[5] | (uint16_t)rt_header[6] << 8;
        rt_rx_at = 0;
        if (rt_header[0] != 1 || rt_rx_length > 256)
        {
            rt_rx_state = 0;
            return;
        }
        rt_rx_state = rt_rx_length ? 3 : 4;
        return;
    }
    if (rt_rx_state == 3)
    {
        if (rt_rx_allowed && ((rt_header[3] >= 0x73 && rt_header[3] <= 0x76) || rt_header[3] == 0x79 ||
                              (rt_header[3] >= 0x7a && rt_header[3] <= 0x7f)))
        {
            bridgeCacheKind = 0;
            RT_BUFFER[rt_rx_at] = v;
        }
        rt_rx_crc = rt_crc_byte(rt_rx_crc, v);
        rt_value = v;
        if (++rt_rx_at == rt_rx_length)
            rt_rx_state = 4;
        return;
    }
    if (rt_rx_state == 4)
    {
        rt_crc_low = v;
        rt_rx_state = 5;
        return;
    }
    rt_rx_state = 0;
    if ((uint16_t)(rt_crc_low | (uint16_t)v << 8) != rt_rx_crc || rt_header[4] != 0)
        return;
    uint8_t op = rt_header[3];
    if (op >= 0x7a && op <= 0x7f)
    {
        uint16_t id = rt_header[1] | (uint16_t)rt_header[2] << 8;
        if (op == 0x7d)
            rt_folder_request(id, rt_rx_length, allowSwitch && rt_rx_allowed, rt_rx_crc);
        else
            rt_sd_request(op, id, rt_rx_length, allowSwitch && rt_rx_allowed, rt_rx_crc);
        return;
    }

    if ((op >= 0x73 && op <= 0x76) || op == 0x79)
    {
        uint16_t id = rt_header[1] | (uint16_t)rt_header[2] << 8;
        uint8_t result;
        if (rt_cached && id == rt_cached_id)
        {
            if (op != rt_cached_op || rt_rx_crc != rt_cached_crc)
            {
                result = 0xf4;
            }
            else if (op == 0x79)
            {
                // Every repeated PC read-open starts a fresh read from byte zero.
                result = rt_pc_open_request(rt_rx_length, allowSwitch && rt_rx_allowed);
            }
            else
            {
                result = rt_cached_result;
            }
        }
        else
        {
            // Nested RPC replies also pass through rt_feed: snapshot parser fields.
            uint16_t requestCrc = rt_rx_crc;
            uint16_t length = rt_rx_length;
            bool allowed = allowSwitch && rt_rx_allowed;
            result =
                op == 0x79 ? rt_pc_open_request(length, allowed) : rt_upload_request(op, length, allowed);
            rt_cached = true;
            rt_cached_id = id;
            rt_cached_op = op;
            rt_cached_crc = requestCrc;
            rt_cached_result = result;
        }
        rt_emit(op | 0x80, id, result);
        return;
    }
    if ((op == 0x70 && rt_rx_length == 1) || ((op == 0x71 || op == 0x78) && rt_rx_length == 0))
        rt_handle(op, rt_header[1] | (uint16_t)rt_header[2] << 8, rt_value, allowSwitch && rt_rx_allowed);
}

static void rt_poll(bool allowSwitch)
{
    if (rt_upload && ((unsigned long)(millis() - rt_upload_tick) > 10000UL || rt_line_active()))
    {
        rt_upload_abort();
        rt_emit(0x72, 0, 0);
    }
    while (Serial.available())
        rt_feed((uint8_t)Serial.read(), allowSwitch);
}

static void rt_hold_begin(uint32_t n)
{
    rt_write_left = n ? n : 65536UL;
}

static void rt_hold_end()
{
    rt_write_left = 0;
}

static void rt_hold_data(uint32_t n)
{
    if (n >= rt_write_left)
        rt_write_left = 0;
    else
        rt_write_left -= n;
}

static void rt_wait_command()
{
    while (!rt_line_active())
        rt_poll(true);
    rt_busy = true;
    rt_poll(false);
    rt_emit(0x72, 0, 0);
}

static void rt_prepare(uint8_t cmd)
{
    if (RT_PROFILE && cmd != 0x92)
        rt_hold_end();
}

static void rt_finish()
{
    if (!bridgeFlush())
        bridgeFatal();
    rt_poll(false);
    rt_busy = false;
    rt_emit(0x72, 0, 0);
}

// Windows PC link: UART 115200 / 8N1, stop-and-wait framed protocol.
// bridgeBuffer is shared with SD/MZ transfer; no second 256-byte packet buffer.
static uint16_t pcRequestId = 0;

static uint16_t pc_crc_byte(uint16_t c, uint8_t v)
{
    return rt_crc_byte(c, v);
}

static bool pc_read_byte(uint8_t &v, unsigned long start)
{
    while (!Serial.available())
        if ((unsigned long)(millis() - start) >= 2000UL)
            return false;
    v = (uint8_t)Serial.read();
    rt_feed(v, false);
    return true;
}

static bool pc_response(uint16_t id, uint8_t op, uint16_t limit, uint8_t &status, uint16_t &length)
{
    unsigned long start = millis();
    uint8_t v, magic = 0;
    while (pc_read_byte(v, start))
    {
        if (magic == 0)
        {
            magic = v == 0x4d ? 1 : 0;
            continue;
        }
        if (v != 0x5a)
        {
            magic = v == 0x4d ? 1 : 0;
            continue;
        }
        magic = 0;
        uint8_t h[7];
        for (uint8_t i = 0; i < 7; i++)
            if (!pc_read_byte(h[i], start))
                return false;
        uint16_t n = h[5] | (uint16_t)h[6] << 8;
        if (h[0] != 1 || n > 256)
            continue;
        bool match =
            (h[1] == (uint8_t)id && h[2] == (uint8_t)(id >> 8) && h[3] == (uint8_t)(op | 0x80) && n <= limit);
        uint16_t crc = 0xffff;
        for (uint8_t i = 0; i < 7; i++)
            crc = pc_crc_byte(crc, h[i]);
        for (uint16_t i = 0; i < n; i++)
        {
            if (!pc_read_byte(v, start))
                return false;
            crc = pc_crc_byte(crc, v);
            if (match)
                bridgeBuffer[i] = v;
        }
        uint8_t lo, hi;
        if (!pc_read_byte(lo, start) || !pc_read_byte(hi, start))
            return false;
        if (crc != (uint16_t)(lo | (uint16_t)hi << 8) || !match)
            continue;
        status = h[4];
        length = n;
        return true;
    }
    return false;
}

static uint8_t pc_call(uint8_t op, const uint8_t *payload, uint16_t n, uint16_t maxReply, uint16_t &got)
{
    const uint16_t id = ++pcRequestId;
    // Payload must remain unchanged on retry. WRITE's response has no payload;
    // other requests use locals/globals outside s_data.
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        uint16_t crc = rt_frame_begin(op, id, 0, n);
        crc = rt_frame_bytes(crc, payload, n);
        rt_frame_end(crc);
        uint8_t status;
        if (pc_response(id, op, maxReply, status, got))
            return status;
    }
    got = 0;
    return 0xf2;
}

#include "RetroStorageImpl.h"

void sdinit(void)
{
    // SD初期化
    if (!SD.begin(CABLESELECTPIN, 8))
    {
        ////    Serial.println("Failed : SD.begin");
        eflg = true;
    }
    else
    {
        ////    Serial.println("OK : SD.begin");
        eflg = false;
    }
    ////    Serial.println("START");
}

void setup()
{
    Serial.begin(115200);
    ////    Serial.begin(9600);
    // CS=pin10
    // pin10 output

    pinMode(CABLESELECTPIN, OUTPUT);
    pinMode(CHKPIN, INPUT); // CHK
    for (uint8_t pin = PB0PIN; pin <= PB7PIN; ++pin)
        pinMode(pin, OUTPUT);
    pinMode(FLGPIN, OUTPUT); // FLG

    for (uint8_t pin = PA0PIN; pin <= PA3PIN; ++pin)
        pinMode(pin, INPUT_PULLUP);

    for (uint8_t pin = PB0PIN; pin <= PB7PIN; ++pin)
        digitalWrite(pin, LOW);
    digitalWrite(FLGPIN, LOW);

    delay(500);

    // SETSコマンドでSAVE用ファイル名を指定なくSAVEされた場合のデフォルトファイル名を設定
    strcpy(w_name, "default.cas");

    sdinit();
    // Boot-only event; Windows clears interrupted file and SD operations.
    rt_emit(0x73, 0, 0);
}

// 4BIT受信
byte rcv4bit(void)
{
    // HIGHになるまでループ
    while (digitalRead(CHKPIN) != HIGH)
    {
        rt_poll(false);
    }
    // 受信
    byte j_data =
        digitalRead(PA0PIN) + digitalRead(PA1PIN) * 2 + digitalRead(PA2PIN) * 4 + digitalRead(PA3PIN) * 8;
    // FLGをセット
    digitalWrite(FLGPIN, HIGH);
    // LOWになるまでループ
    while (digitalRead(CHKPIN) == HIGH)
    {
        rt_poll(false);
    }
    // FLGをリセット
    digitalWrite(FLGPIN, LOW);
    return (j_data);
}

// 1BYTE受信
byte rcv1byte(void)
{
    byte i_data = 0;
    i_data = rcv4bit() * 16;
    i_data = i_data + rcv4bit();
    return (i_data);
}

// 1BYTE送信
void snd1byte(byte i_data)
{
    // 下位ビットから8ビット分をセット
    for (uint8_t pin = PB0PIN; pin <= PB7PIN; ++pin)
    {
        digitalWrite(pin, i_data & 1);
        i_data >>= 1;
    }
    digitalWrite(FLGPIN, HIGH);
    // HIGHになるまでループ
    while (digitalRead(CHKPIN) != HIGH)
    {
        rt_poll(false);
    }
    digitalWrite(FLGPIN, LOW);
    // LOWになるまでループ
    while (digitalRead(CHKPIN) == HIGH)
    {
        rt_poll(false);
    }
}

// 小文字->大文字
char upper(char c)
{
    if ('a' <= c && c <= 'z')
    {
        c = c - ('a' - 'A');
    }
    return c;
}

// ファイル名の最後が「.cas」でなければ付加
RT_HANDLER void addcas(char *f_name, char *m_name)
{
    unsigned int lp1 = 0;
    while (f_name[lp1] != 0x00)
    {
        m_name[lp1] = f_name[lp1];
        lp1++;
    }
    if (f_name[lp1 - 4] != '.' || (f_name[lp1 - 3] != 'c' && f_name[lp1 - 3] != 'C') ||
        (f_name[lp1 - 2] != 'a' && f_name[lp1 - 3] != 'A') ||
        (f_name[lp1 - 1] != 's' && f_name[lp1 - 1] != 'S'))
    {
        m_name[lp1++] = '.';
        m_name[lp1++] = 'c';
        m_name[lp1++] = 'a';
        m_name[lp1++] = 's';
    }
    m_name[lp1] = 0x00;
}

// 比較文字列取得 32+1文字まで取得、ただしダブルコーテーションは無視する
static RT_HANDLER void msx_receive_name(char *name, uint8_t limit)
{
    uint8_t at = 0;
    for (uint8_t count = 0; count < 33; ++count)
    {
        uint8_t value = rcv1byte();
        if (at >= limit)
            name[at++] = 0;
        else if (value != 0x22)
            name[at++] = value;
    }
}

RT_HANDLER void receive_name(char *name)
{
    msx_receive_name(name, 33);
}

RT_HANDLER void receive_name6(char *name)
{
    msx_receive_name(name, 6);
}

// f_nameとc_nameをc_nameに0x00が出るまで比較
// FILENAME COMPARE
boolean f_match(char *f_name, char *c_name)
{
    boolean flg1 = true;
    unsigned int lp1 = 0;
    while (lp1 <= 32 && c_name[0] != 0x00 && flg1 == true)
    {
        if (upper(f_name[lp1]) != c_name[lp1])
        {
            flg1 = false;
        }
        lp1++;
        if (c_name[lp1] == 0x00)
        {
            break;
        }
    }
    return flg1;
}

// SD-CARDのFILELIST
RT_HANDLER void dirlist(void)
{
    sdirValid = 0;
    // 比較文字列取得 32+1文字まで
    receive_name(c_name);
    //
    BridgeFile file2 = SD.open("/");
    if (file2 == true)
    {
        // 状態コード送信(OK)
        snd1byte(0x00);

        BridgeFile entry = file2.openNextFile();
        int cntl2 = 0;
        unsigned int br_chk = 0;
        int page = 1;
        // 全件出力の場合には10件出力したところで一時停止、キー入力により継続、打ち切りを選択
        while (br_chk == 0)
        {
            if (entry)
            {
                entry.getName(f_name, 36);
                unsigned int lp1 = 0;
                // 一件送信
                // 比較文字列でファイルネームを先頭から比較して一致するものだけを出力
                if (f_match(f_name, c_name))
                {
                    // sdir[]にf_nameを保存
                    sdir[cntl2] = entry.directoryEntryIndex();
                    sdirValid |= (uint16_t)1 << cntl2;
                    snd1byte(0x30 + cntl2);
                    snd1byte(0x20);
                    while (lp1 <= 36 && f_name[lp1] != 0x00)
                    {
                        snd1byte(upper(f_name[lp1]));
                        lp1++;
                    }
                    snd1byte(0x0D);
                    snd1byte(0x00);
                    cntl2++;
                }
            }
            // CNTL2 > 表示件数-1
            if (!entry || cntl2 > 9)
            {
                // 継続・打ち切り選択指示要求
                snd1byte(0xfe);

                // 選択指示受信(0:継続 B:前ページ 以外:打ち切り)
                br_chk = rcv1byte();
                // 前ページ処理
                if (br_chk == 0x42)
                {
                    // 先頭ファイルへ
                    file2.rewindDirectory();
                    // entry値更新
                    entry = file2.openNextFile();
                    // もう一度先頭ファイルへ
                    file2.rewindDirectory();
                    if (page <= 2)
                    {
                        // 現在ページが1ページ又は2ページなら1ページ目に戻る処理
                        page = 0;
                    }
                    else
                    {
                        // 現在ページが3ページ以降なら前々ページまでのファイルを読み飛ばす
                        page = page - 2;
                        cntl2 = 0;
                        // page*表示件数
                        while (cntl2 < page * 10)
                        {
                            entry = file2.openNextFile();
                            if (f_match(f_name, c_name))
                            {
                                cntl2++;
                            }
                        }
                    }
                    br_chk = 0;
                }
                // 1～0までの数字キーが押されたらsdir[]から該当するファイル名を送信
                if (br_chk >= 0x30 && br_chk <= 0x39 && (sdirValid & ((uint16_t)1 << (br_chk - 0x30))))
                {
                    file2.getEntryName(sdir[br_chk - 0x30], f_name, 36);
                    file_r = SD.open(f_name, FILE_READ);
                    if (file_r == true)
                    {
                        // f_length設定、r_count初期化
                        f_length = file_r.size();
                        r_count = 0;
                        unsigned int lp2 = 0;
                        snd1byte(0xFD);
                        while (lp2 <= 36 && f_name[lp2] != 0x00)
                        {
                            snd1byte(upper(f_name[lp2]));
                            lp2++;
                        }
                        snd1byte(0x0A);
                        snd1byte(0x0D);
                        snd1byte(0x00);
                    }
                }
                page++;
                cntl2 = 0;
                sdirValid = 0;
            }
            // ファイルがまだあるなら次読み込み、なければ打ち切り指示
            if (entry)
            {
                entry = file2.openNextFile();
            }
            else
            {
                br_chk = 1;
            }
        }
        // 処理終了指示
        snd1byte(0xFF);
        snd1byte(0x00);
    }
    else
    {
        snd1byte(0xf1);
    }
    sdirValid = 0;
}

// LOADFILEOPEN 読み込み用のファイル名を設定する
RT_HANDLER void loadopen(void)
{
    receive_name(m_name);
    addcas(m_name, f_name);
    uint8_t result = msx_open_read(f_name);
    snd1byte(result);
    if (result != 0)
        sdinit();
}

// SAVEFILEOPEN 書き込み用のファイル名を設定する
RT_HANDLER void saveopen(void)
{
    boolean flg = false;
    // DOSファイル名取得
    receive_name(m_name);
    addcas(m_name, f_name);
    strcpy(w_name, f_name);
    // ファイルオープン
    if (file_w == true)
    {
        file_w.close();
    }
    file_w = SD.open(w_name, FILE_WRITE);
    // 状態コード送信(OK)
    snd1byte(0x00);
}

// sloadコマンド
static RT_HANDLER uint8_t msx_read_byte()
{
    uint8_t value = file_r.read();
    r_count++;
    return value;
}

static RT_HANDLER bool msx_load_header(uint8_t marker)
{
    boolean flg = false;
    int wk1 = 0;
    unsigned char rdata;
    uint8_t lp1 = 0;
    // 比較文字列取得 32+1文字まで
    receive_name6(c_name);
    if (file_r == true)
    {
        snd1byte(0x00);
        // 0xd3が10個連続するまで読み飛ばし、読み飛ばしでファイルエンドになってしまったらエラー終了
        while (flg == false && f_length >= r_count)
        {
            rdata = msx_read_byte();
            if (rdata == marker)
            {
                wk1++;
            }
            else
            {
                wk1 = 0;
            }
            if (wk1 >= 10)
            {
                flg = true;
            }
            // ファイルネームが指定された場合、読み出したファイルネームが一致するまで読み飛ばし
            if (flg == true)
            {
                for (lp1 = 0; lp1 <= 5; lp1++)
                {
                    f_name[lp1] = msx_read_byte();
                }
                f_name[6] = 0x00;
                if (c_name[0] != 0x00)
                {
                    if (f_match(f_name, c_name) == false)
                    {
                        flg = false;
                    }
                }
            }
        }
        if (flg == true)
        {
            snd1byte(0x00);
            // ファイルネームを送信
            for (lp1 = 0; lp1 <= 5; lp1++)
            {
                snd1byte(f_name[lp1]);
            }
            // ヘッダを読み飛ばし
            for (lp1 = 1; lp1 <= 8; lp1++)
            {
                rdata = msx_read_byte();
            }
            return true;
        }
    }
    snd1byte(0xf1);
    sdinit();
    r_count = 0;
    return false;
}

static RT_HANDLER unsigned int msx_read_address()
{
    s_adrs2 = msx_read_byte();
    s_adrs1 = msx_read_byte();
    return s_adrs1 * 256 + s_adrs2;
}

RT_HANDLER void sload(void)
{
    if (!msx_load_header(0xd3))
        return;
    bool firstLine = true;
    for (;;)
    {
        e_adrs = msx_read_address();
        if (e_adrs == 0)
            break;
        unsigned int count = firstLine ? s_adrs2 - 3 : e_adrs - s_adrs - 2;
        firstLine = false;
        s_adrs = e_adrs;
        snd1byte(count);
        for (unsigned int index = 1; index <= count; ++index)
            snd1byte(msx_read_byte());
    }
    snd1byte(0x00);
}

// sbloadコマンド
static RT_HANDLER unsigned int msx_read_send_address()
{
    unsigned int address = msx_read_address();
    snd1byte(s_adrs2);
    snd1byte(s_adrs1);
    return address;
}

RT_HANDLER void sbload(void)
{
    unsigned char rdata;
    unsigned int lp1 = 0;
    int wk1 = 0;
    if (!msx_load_header(0xd0))
    {
        return;
    }
    // スタートアドレスを読み出して送信
    s_adrs = msx_read_send_address();
    // エンドアドレスを読み出して送信
    e_adrs = msx_read_send_address();
    // 実行アドレスを読み出して送信
    g_adrs = msx_read_send_address();
    // スタートアドレスからエンドアドレスまでのデータを読み出して送信
    for (lp1 = s_adrs; lp1 <= e_adrs; lp1++)
    {
        rdata = msx_read_byte();
        snd1byte(rdata);
    }
}

// ヘッダ書き込み
RT_HANDLER void header_write(void)
{
    static const uint8_t header[] = {0x1f, 0xa6, 0xde, 0xba, 0xcc, 0x13, 0x7d, 0x74};
    file_w.write(header, sizeof(header));
}

// ファイルヘッダ書き込み
RT_HANDLER void file_header(char hdata)
{
    uint8_t lp1 = 0;
    // ロングヘッダ書き込み
    header_write();
    // hdata x 10
    for (lp1 = 1; lp1 <= 10; lp1++)
    {
        file_w.write(hdata);
    }
    // ファイルネーム送信及び書き込み
    for (lp1 = 0; lp1 <= 5; lp1++)
    {
        snd1byte(c_name[lp1]);
        file_w.write(c_name[lp1]);
    }
    // ショートヘッダ書き込み
    header_write();
}

// ssaveコマンド
static RT_HANDLER uint8_t msx_receive_write_byte()
{
    uint8_t value = rcv1byte();
    file_w.write(value);
    return value;
}

static RT_HANDLER unsigned int msx_receive_write_address()
{
    s_adrs1 = msx_receive_write_byte();
    s_adrs2 = msx_receive_write_byte();
    return s_adrs2 * 256 + s_adrs1;
}

static RT_HANDLER void msx_receive_write_bytes(unsigned int last)
{
    for (unsigned int index = 0; index <= last; ++index)
        msx_receive_write_byte();
}

#if defined(__AVR__)
#define MSX_SAVE_SHARED __attribute__((noinline, noclone))
#else
#define MSX_SAVE_SHARED
#endif
static MSX_SAVE_SHARED void msx_save_record(bool binary)
{
    uint8_t lp1 = 0;
    unsigned int wk1 = 0;
    unsigned char rdata;
    receive_name6(c_name);
    // The binary command consumes its parameter status even for an empty name.
    if (binary && rcv1byte() != 0x00)
        return;
    if (c_name[0] == 0x00)
    {
        snd1byte(0xf1);
        sdinit();
        return;
    }
    if (file_w)
        file_w.close();
    file_w = SD.open(w_name, FILE_WRITE);
    if (!file_w)
    {
        snd1byte(0xf1);
        sdinit();
        return;
    }
    snd1byte(0x00);
    if (binary)
    {
        // ファイルヘッダ書き込み
        file_header(char(0xd0));
        // スタートアドレス受信及び書き込み
        s_adrs = msx_receive_write_address();
        // エンドアドレス受信及び書き込み
        e_adrs = msx_receive_write_address();
        // 実行アドレス受信及び書き込み
        g_adrs = msx_receive_write_address();
        // スタートアドレスからエンドアドレスまでのデータを受信して書き込み
        msx_receive_write_bytes(e_adrs - s_adrs);
    }
    else
    {
        // ファイルヘッダ書き込み
        file_header(char(0xd3));
        wk1 = rcv1byte();
        // 1行Byte数が0になるまでループ
        while (wk1 != 0)
        {
            // Two pointer bytes followed by the original line bytes.
            msx_receive_write_bytes(wk1 + 1);
            wk1 = rcv1byte();
        }
        // 終了マークを書き込み
        for (lp1 = 1; lp1 <= 12; lp1++)
        {
            file_w.write(char(0x00));
        }
    }
    file_w.close();
    snd1byte(0x00);
}

RT_HANDLER void ssave(void)
{
    msx_save_record(false);
}

RT_HANDLER void sbsave(void)
{
    msx_save_record(true);
}

RT_HANDLER void loop()
{
    for (uint8_t pin = PB0PIN; pin <= PB7PIN; ++pin)
        digitalWrite(pin, LOW);
    digitalWrite(FLGPIN, LOW);
    // コマンド取得待ち
    ////    Serial.println("COMMAND WAIT");
    rt_wait_command();
    byte cmd = rcv1byte();
    rt_prepare(cmd);
    ////    Serial.println(cmd,HEX);
    if (eflg == false)
    {
        // All seven supported commands begin with the same acknowledgment.
        if (cmd >= 0x41 && cmd <= 0x47)
            snd1byte(0x00);
        switch (cmd)
        {
            // 41hでファイルリスト出力
        case 0x41:
            ////    Serial.println("FILE LIST START");
            // 状態コード送信(OK)

            sdinit();
            dirlist();
            break;
            // 42hでLOADFILEOPEN
        case 0x42:
            ////    Serial.println("LOADFILEOPEN");
            // 状態コード送信(OK)

            loadopen();
            break;
            // 43hでSAVEFILEOPEN
        case 0x43:
            ////    Serial.println("SAVEFILEOPEN");
            // 状態コード送信(OK)

            saveopen();
            break;
            // 44h:sload
        case 0x44:
            ////    Serial.println("sload START");
            // 状態コード送信(OK)

            sload();
            ////  delay(1500);
            break;
            // 45h:sbload
        case 0x45:
            ////    Serial.println("sbload START");
            // 状態コード送信(OK)

            sbload();
            ////  delay(1500);
            break;
            // 46h:ssave
        case 0x46:
            ////    Serial.println("ssave START");
            // 状態コード送信(OK)

            ssave();
            ////  delay(1500);
            break;
            // 47h:sbload
        case 0x47:
            ////    Serial.println("sbsave START");
            // 状態コード送信(OK)

            sbsave();
            ////  delay(1500);
            break;
        default:
            // 状態コード送信(CMD ERROR)
            snd1byte(0xF4);
        }
    }
    else
    {
        // 状態コード送信(ERROR)
        snd1byte(0xF0);
        sdinit();
    }
    rt_finish();
}
