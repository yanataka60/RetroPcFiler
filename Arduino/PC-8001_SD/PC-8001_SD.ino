// Runtime SD / PC backend for PC-8001; original machine protocol is retained.
#include <Arduino.h>
#define RETRO_PROFILE 3
#define RT_PROFILE 3
#define RETRO_IDLE_LEVEL LOW
#define CHKPIN 15
#define RT_BUFFER bridgeBuffer
#include "RetroStorage.h"
#include "RetroUart.h"
// 2024. 3.12 sd-card再挿入時の初期化処理を追加
//
#include "SdFat.h"
#include <SPI.h>
BridgeSd SD;
unsigned long r_count = 0;
unsigned long f_length = 0;
char m_name[40];
byte s_data[260];
char f_name[40];
char c_name[40];
char new_name[40];
char w_name[40];
BridgeFile file, w_file;
unsigned int s_adrs, e_adrs, w_length, w_len1, w_len2, s_adrs1, s_adrs2, b_length;

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
boolean eflg;

// SdFat 1.x accepts an SPI divider; SdFat 2.x accepts a frequency in Hz.
// Keep command-local buffers out of the idle polling stack under AVR LTO.
#if defined(__AVR__)
#define RT_HANDLER __attribute__((noinline))
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
// Directory filtering and an idle host-to-SD copy cannot run together.
// Reuse the existing filter buffer; the next directory command replaces it.
#define rt_upload_name c_name
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
static uint8_t pc8001_open_read(const char *name);

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
    if (!bridgeNativeOpen(f, from, O_WRONLY))
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

// Share the byte write and CRC update across all frame serializers.
static RT_HANDLER uint16_t rt_write_crc(uint16_t crc, uint8_t value)
{
    Serial.write(value);
    return rt_crc_byte(crc, value);
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
    uint16_t length = diagnostic ? (uint16_t)(19 + strlen(rt_upload_name) + strlen(actualName)) : 4;
    uint8_t header[7] = {
        1, (uint8_t)id, (uint8_t)(id >> 8), op, status, (uint8_t)length, (uint8_t)(length >> 8)};
    uint8_t state[4] = {rt_backend, RT_PROFILE, rt_flags(), 127};
    uint16_t crc = 0xffff;
    Serial.write((uint8_t)0x4d);
    Serial.write((uint8_t)0x5a);
    for (uint8_t i = 0; i < sizeof(header); ++i)
    {
        crc = rt_write_crc(crc, header[i]);
    }
    for (uint8_t i = 0; i < sizeof(state); ++i)
    {
        crc = rt_write_crc(crc, state[i]);
    }
    if (diagnostic)
    {
        crc = rt_write_crc(crc, 3);
        const char *names[2] = {rt_upload_name, actualName};
        for (uint8_t name = 0; name < 2; ++name)
        {
            const char *p = names[name];
            do
            {
                uint8_t value = (uint8_t)*p;
                crc = rt_write_crc(crc, value);
            } while (*p++);
        }
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
        for (uint8_t i = 0; i < sizeof(fields); ++i)
        {
            crc = rt_write_crc(crc, fields[i]);
        }
        uint8_t extra[3] = {rt_fault_stage, (uint8_t)ram, (uint8_t)(ram >> 8)};
        for (uint8_t i = 0; i < sizeof(extra); ++i)
        {
            crc = rt_write_crc(crc, extra[i]);
        }
    }
    Serial.write((uint8_t)crc);
    Serial.write((uint8_t)(crc >> 8));
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

// Open from the PC only while the machine-side protocol is idle.
static uint8_t rt_pc_open_request(uint16_t length, bool allowed)
{
    if (!allowed || rt_busy || rt_upload || rt_write_left || bridgeWriters || bridgeSplitLeft ||
        rt_line_active())
        return 0xf7;
    if (rt_backend != 1 || length < 5 || length > 31)
        return 0xf4;
    for (uint16_t index = 0; index < length; ++index)
    {
        uint8_t character = RT_BUFFER[index];
        if (character < 32 || character > 126 || strchr("\\/:*?\"<>|", character))
            return 0xf4;
    }
    // Remote file RPCs reuse RT_BUFFER, so retain the name in existing scratch.
    memcpy(f_name, RT_BUFFER, length);
    f_name[length] = 0;
    const char *extension = f_name + length - 4;
    if (extension[0] != '.' || (extension[1] != 'c' && extension[1] != 'C') ||
        (extension[2] != 'm' && extension[2] != 'M') || (extension[3] != 't' && extension[3] != 'T'))
        return 0xf4;
    rt_busy = true;
    uint8_t result = pc8001_open_read(f_name);
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
    file = selected;
    f_length = file.size();
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
    uint8_t h[7] = {1, (uint8_t)id, (uint8_t)(id >> 8), op, 0, (uint8_t)n, (uint8_t)(n >> 8)};
    // Payload must remain unchanged on retry. WRITE's response has no payload;
    // other requests use locals/globals outside bridgeBuffer.
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        uint16_t crc = 0xffff;
        Serial.write((uint8_t)0x4d);
        Serial.write((uint8_t)0x5a);
        for (uint8_t i = 0; i < 7; i++)
        {
            crc = rt_write_crc(crc, h[i]);
        }
        for (uint16_t i = 0; i < n; i++)
        {
            crc = rt_write_crc(crc, payload[i]);
        }
        Serial.write((uint8_t)crc);
        Serial.write((uint8_t)(crc >> 8));
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
    pinMode(CHKPIN, INPUT);  //CHK
    pinMode(PB0PIN, OUTPUT); //送信データ
    pinMode(PB1PIN, OUTPUT); //送信データ
    pinMode(PB2PIN, OUTPUT); //送信データ
    pinMode(PB3PIN, OUTPUT); //送信データ
    pinMode(PB4PIN, OUTPUT); //送信データ
    pinMode(PB5PIN, OUTPUT); //送信データ
    pinMode(PB6PIN, OUTPUT); //送信データ
    pinMode(PB7PIN, OUTPUT); //送信データ
    pinMode(FLGPIN, OUTPUT); //FLG

    pinMode(PA0PIN, INPUT_PULLUP); //受信データ
    pinMode(PA1PIN, INPUT_PULLUP); //受信データ
    pinMode(PA2PIN, INPUT_PULLUP); //受信データ
    pinMode(PB3PIN, OUTPUT);       //送信データ
    pinMode(PB4PIN, OUTPUT);       //送信データ
    pinMode(PB5PIN, OUTPUT);       //送信データ
    pinMode(PB6PIN, OUTPUT);       //送信データ
    pinMode(PB7PIN, OUTPUT);       //送信データ
    pinMode(FLGPIN, OUTPUT);       //FLG

    pinMode(PA0PIN, INPUT_PULLUP); //受信データ
    pinMode(PA1PIN, INPUT_PULLUP); //受信データ
    pinMode(PA2PIN, INPUT_PULLUP); //受信データ
    pinMode(PA3PIN, INPUT_PULLUP); //受信データ

    digitalWrite(PB0PIN, LOW);
    digitalWrite(PB1PIN, LOW);
    digitalWrite(PB2PIN, LOW);
    digitalWrite(PB3PIN, LOW);
    digitalWrite(PB4PIN, LOW);
    digitalWrite(PB5PIN, LOW);
    digitalWrite(PB6PIN, LOW);
    digitalWrite(PB7PIN, LOW);
    digitalWrite(FLGPIN, LOW);

    delay(1500);
    //sd_waopen sd_wnopen sd_wdirectでSAVE用ファイル名を指定なくSAVEされた場合のデフォルトファイル名を設定
    strcpy(w_name, "default.dat");

    sdinit();
    // Boot-only event: distinct from ordinary status updates.
    rt_emit(0x73, 0, 0);
}

//4BIT受信
byte rcv4bit(void)
{
    //HIGHになるまでループ
    while (digitalRead(CHKPIN) != HIGH)
    {
    }
    //受信
    byte j_data =
        digitalRead(PA0PIN) + digitalRead(PA1PIN) * 2 + digitalRead(PA2PIN) * 4 + digitalRead(PA3PIN) * 8;
    //FLGをセット
    digitalWrite(FLGPIN, HIGH);
    //LOWになるまでループ
    while (digitalRead(CHKPIN) == HIGH)
    {
    }
    //FLGをリセット
    digitalWrite(FLGPIN, LOW);
    return (j_data);
}

//1BYTE受信
byte rcv1byte(void)
{
    byte i_data = 0;
    i_data = rcv4bit() * 16;
    i_data = i_data + rcv4bit();
    return (i_data);
}

//1BYTE送信
void snd1byte(byte i_data)
{
    //下位ビットから8ビット分をセット
    digitalWrite(PB0PIN, (i_data) & 0x01);
    digitalWrite(PB1PIN, (i_data >> 1) & 0x01);
    digitalWrite(PB2PIN, (i_data >> 2) & 0x01);
    digitalWrite(PB3PIN, (i_data >> 3) & 0x01);
    digitalWrite(PB4PIN, (i_data >> 4) & 0x01);
    digitalWrite(PB5PIN, (i_data >> 5) & 0x01);
    digitalWrite(PB6PIN, (i_data >> 6) & 0x01);
    digitalWrite(PB7PIN, (i_data >> 7) & 0x01);
    digitalWrite(FLGPIN, HIGH);
    //HIGHになるまでループ
    while (digitalRead(CHKPIN) != HIGH)
    {
    }
    digitalWrite(FLGPIN, LOW);
    //LOWになるまでループ
    while (digitalRead(CHKPIN) == HIGH)
    {
    }
}

//小文字->大文字
char upper(char c)
{
    if ('a' <= c && c <= 'z')
    {
        c = c - ('a' - 'A');
    }
    return c;
}

//ファイル名の最後が「.cmt」でなければ付加
RT_HANDLER void addcmt(char *f_name, char *m_name)
{
    unsigned int lp1 = 0;
    while (f_name[lp1] != 0x00)
    {
        m_name[lp1] = f_name[lp1];
        lp1++;
    }
    if (f_name[lp1 - 4] != '.' || (f_name[lp1 - 3] != 'C' && f_name[lp1 - 3] != 'c') ||
        (f_name[lp1 - 2] != 'M' && f_name[lp1 - 2] != 'm') ||
        (f_name[lp1 - 1] != 'T' && f_name[lp1 - 1] != 't'))
    {
        m_name[lp1++] = '.';
        m_name[lp1++] = 'c';
        m_name[lp1++] = 'm';
        m_name[lp1++] = 't';
    }
    m_name[lp1] = 0x00;
}

//比較文字列取得 32+1文字まで取得、ただしダブルコーテーションは無視する
RT_HANDLER void receive_name(char *f_name)
{
    char r_data;
    unsigned int lp2 = 0;
    for (unsigned int lp1 = 0; lp1 <= 32; lp1++)
    {
        r_data = rcv1byte();
        if (r_data != 0x22)
        {
            f_name[lp2] = r_data;
            lp2++;
        }
    }
}

// MONITOR Lコマンド .CMT LOAD
// Keep byte accounting in one out-of-line function on AVR.
// EOF and all caller-specific framing/close decisions are unchanged.
static RT_HANDLER int pc8001_read_byte()
{
    int value = file.read();
    ++r_count;
    return value;
}

static RT_HANDLER int pc8001_read_send()
{
    int value = pc8001_read_byte();
    snd1byte(value);
    return value;
}

static RT_HANDLER bool pc8001_prepare_cmt_read()
{
    boolean flg = false;
    receive_name(m_name);
    if (m_name[0] != 0x00)
    {
        addcmt(m_name, f_name);
        uint8_t result = pc8001_open_read(f_name);
        snd1byte(result);
        if (result != 0)
            sdinit();
        flg = result == 0;
    }
    else
    {
        flg = f_length > r_count;
        snd1byte(flg ? 0x00 : 0xf1);
    }
    return flg;
}

RT_HANDLER void cmt_load(void)
{
    boolean flg = pc8001_prepare_cmt_read();

    //良ければファイルエンドまで読み込みを続行する
    if (flg == true)
    {
        int rdata = 0;

        //ヘッダーが出てくるまで読み飛ばし
        while (rdata != 0x3a && rdata != 0xd3)
        {
            rdata = pc8001_read_byte();
        }
        //ヘッダー送信
        snd1byte(rdata);

        //ヘッダーが0x3aなら続行、違えばエラー
        if (rdata == 0x3a)
        {
            //START ADDRESS HIを送信
            s_adrs1 = pc8001_read_send();

            //START ADDRESS LOを送信
            s_adrs2 = pc8001_read_send();
            s_adrs = s_adrs1 * 256 + s_adrs2;

            //CHECK SUMを送信
            rdata = pc8001_read_send();

            //HEADERを送信
            rdata = pc8001_read_send();

            //データ長を送信
            b_length = pc8001_read_send();

            //データ長が0x00でない間ループする
            while (b_length != 0x00)
            {
                for (unsigned int lp1 = 0; lp1 <= b_length; lp1++)
                {
                    //実データを読み込んで送信
                    rdata = pc8001_read_send();
                }
                //CHECK SUMを送信
                rdata = pc8001_read_send();
                //データ長を送信
                b_length = pc8001_read_send();
            }
            //データ長が0x00だった時の後処理
            //CHECK SUMを送信
            rdata = pc8001_read_send();

            //ファイルエンドに達していたらFILE CLOSE
            if (f_length == r_count)
            {
                file.close();
            }
        }
        else
        {
            file.close();
        }
    }
}

//BASICプログラムのLOAD処理
RT_HANDLER void bas_load(void)
{
    boolean flg = pc8001_prepare_cmt_read();

    //良ければファイルエンドまで読み込みを続行する
    if (flg == true)
    {
        int rdata = 0;

        //ヘッダーが出てくるまで読み飛ばし
        while (rdata != 0x3a && rdata != 0xd3)
        {
            rdata = pc8001_read_byte();
        }
        //ヘッダー送信
        snd1byte(rdata);
        //ヘッダーが0xd3なら続行、違えばエラー
        if (rdata == 0xd3)
        {
            while (rdata == 0xd3)
            {
                rdata = pc8001_read_byte();
            }

            //実データ送信
            int zcnt = 0;
            int zdata = 1;

            snd1byte(rdata);

            //0x00が11個続くまで読み込み、送信
            while (zcnt < 11)
            {
                rdata = pc8001_read_send();
                if (rdata == 0x00)
                {
                    zcnt++;
                    if (zdata != 0)
                    {
                        zcnt = 0;
                    }
                }
                zdata = rdata;
            }

            //ファイルエンドに達していたらFILE CLOSE
            if (f_length == r_count)
            {
                file.close();
            }
        }
        else
        {
            file.close();
        }
    }
}

RT_HANDLER void w_body(void)
{
    byte r_data, csum;
    //ヘッダー 0x3A書き込み
    w_file.write(char(0x3A));
    //スタートアドレス取得、書き込み
    s_adrs1 = rcv1byte();
    s_adrs2 = rcv1byte();
    w_file.write(s_adrs2);
    w_file.write(s_adrs1);
    //CHECK SUM計算、書き込み
    csum = 0 - (s_adrs1 + s_adrs2);
    w_file.write(csum);
    //スタートアドレス算出
    s_adrs = s_adrs1 + s_adrs2 * 256;
    //エンドアドレス取得
    s_adrs1 = rcv1byte();
    s_adrs2 = rcv1byte();
    //エンドアドレス算出
    e_adrs = s_adrs1 + s_adrs2 * 256;
    //ファイル長算出、ブロック数算出
    w_length = e_adrs - s_adrs + 1;
    w_len1 = w_length / 255;
    w_len2 = w_length % 255;
    //実データ受信、書き込み
    //0xFFブロック
    while (w_len1 > 0)
    {
        w_file.write(char(0x3A));
        w_file.write(char(0xFF));
        csum = 0xff;
        for (unsigned int lp1 = 1; lp1 <= 255; lp1++)
        {
            r_data = rcv1byte();
            w_file.write(r_data);
            csum = csum + r_data;
        }
        //CHECK SUM計算、書き込み
        csum = 0 - csum;
        w_file.write(csum);
        w_len1--;
    }

    //端数ブロック処理
    if (w_len2 > 0)
    {
        w_file.write(char(0x3A));
        w_file.write(w_len2);
        csum = w_len2;
        for (unsigned int lp1 = 1; lp1 <= w_len2; lp1++)
        {
            r_data = rcv1byte();
            w_file.write(r_data);
            csum = csum + r_data;
        }
        //CHECK SUM計算、書き込み
        csum = 0 - csum;
        w_file.write(csum);
    }
    w_file.write(char(0x3A));
    w_file.write(char(0x00));
    w_file.write(char(0x00));
}

// MONITOR Wコマンド .CMT SAVE
// Share native/PC writer opening while preserving each command's replies.
static RT_HANDLER bool pc8001_open_write(const char *name, bool replace)
{
    if (true == w_file)
        w_file.close();
    if (replace && SD.exists(name) == true)
        SD.remove(name);
    w_file = SD.open(name, FILE_WRITE);
    return true == w_file;
}

RT_HANDLER void cmt_save(void)
{
    byte r_data, csum;
    //DOSファイル名取得
    receive_name(m_name);
    //ファイル名の指定が無ければエラー
    if (m_name[0] != 0x00)
    {
        addcmt(m_name, f_name);

        pc8001_open_write(f_name, true);
        if (true == w_file)
        {
            //状態コード送信(OK)
            snd1byte(0x00);
            w_body();
            w_file.close();
        }
        else
        {
            snd1byte(0xf0);
            sdinit();
        }
    }
    else
    {
        snd1byte(0xf6);
        sdinit();
    }
}

//BASICプログラムのSAVE処理
RT_HANDLER void bas_save(void)
{
    unsigned int lp1;

    //DOSファイル名取得
    receive_name(m_name);
    //ファイル名の指定が無ければエラー
    if (m_name[0] != 0x00)
    {
        addcmt(m_name, f_name);

        pc8001_open_write(f_name, true);
        if (true == w_file)
        {
            //状態コード送信(OK)
            snd1byte(0x00);

            //スタートアドレス取得
            s_adrs1 = rcv1byte();
            s_adrs2 = rcv1byte();
            //スタートアドレス算出
            s_adrs = s_adrs1 + s_adrs2 * 256;
            //エンドアドレス取得
            s_adrs1 = rcv1byte();
            s_adrs2 = rcv1byte();
            //エンドアドレス算出
            e_adrs = s_adrs1 + s_adrs2 * 256;
            //ヘッダー 0xD3 x 9回書き込み
            for (lp1 = 0; lp1 <= 9; lp1++)
            {
                w_file.write(char(0xD3));
            }
            //DOSファイル名の先頭6文字をファイルネームとして書き込み
            for (lp1 = 0; lp1 <= 5; lp1++)
            {
                w_file.write(m_name[lp1]);
            }
            //実データ (e_adrs - s_adrs +1)を受信、書き込み
            for (lp1 = s_adrs; lp1 <= e_adrs; lp1++)
            {
                w_file.write(rcv1byte());
            }
            //終了 0x00 x 9回書き込み
            for (lp1 = 1; lp1 <= 9; lp1++)
            {
                w_file.write(char(0x00));
            }
            w_file.close();
        }
        else
        {
            snd1byte(0xf0);
            sdinit();
        }
    }
    else
    {
        snd1byte(0xf1);
        sdinit();
    }
}

//BASICプログラムのKILL処理
RT_HANDLER void bas_kill(void)
{
    unsigned int lp1;

    //DOSファイル名取得
    receive_name(m_name);
    //ファイル名の指定が無ければエラー
    if (m_name[0] != 0x00)
    {
        addcmt(m_name, f_name);

        //状態コード送信(OK)
        snd1byte(0x00);
        //ファイルが存在すればdelete
        if (SD.exists(f_name) == true)
        {
            SD.remove(f_name);
            //状態コード送信(OK)
            snd1byte(0x00);
        }
        else
        {
            snd1byte(0xf1);
            sdinit();
        }
    }
    else
    {
        snd1byte(0xf1);
        sdinit();
    }
}

//5F9EH CMT 1Byte読み込み処理の代替、OPENしているファイルの続きから1Byteを読み込み、送信
RT_HANDLER void cmt_5f9e(void)
{
    int rdata = pc8001_read_send();
    //ファイルエンドまで達していればFILE CLOSE
    if (f_length == r_count)
    {
        file.close();
    }
}

//read file open
static uint8_t pc8001_open_read(const char *name)
{
    if (!SD.exists(name))
        return 0xf1;
    file = SD.open(name, FILE_READ);
    if (!file)
        return 0xf0;
    f_length = file.size();
    r_count = 0;
    return 0;
}

RT_HANDLER void sd_ropen(void)
{
    receive_name(f_name);
    uint8_t result = pc8001_open_read(f_name);
    snd1byte(result);
    if (result != 0)
        sdinit();
}

//sd check
RT_HANDLER void sd_check(void)
{
    f_name[0] = 0x00;
    sdinit();
    SD.exists(f_name);
    sdinit();
    SD.exists(f_name);
    sdinit();
    SD.exists(f_name);
    sdinit();
    if (eflg == false)
    {
        //状態コード送信(OK)
        snd1byte(0x00);
    }
    else
    {
        //状態コード送信(SDなし)
        snd1byte(0xf0);
    }
}

//file exist check
RT_HANDLER void sd_exist(void)
{
    receive_name(f_name);
    if (eflg == false)
    {
        //ファイルが存在しなければERROR
        if (SD.exists(f_name) == true)
        {
            //状態コード送信(OK)
            snd1byte(0x00);
        }
        else
        {
            snd1byte(0xf1);
        }
    }
    else
    {
        snd1byte(0xf0);
    }
}

//file delete
RT_HANDLER void sd_delete(void)
{
    receive_name(f_name);
    //ファイルが存在しなければERROR
    if (SD.exists(f_name) == true)
    {
        SD.remove(f_name);
        //ファイルが存在しなければOK
        if (SD.exists(f_name) == false)
        {
            //状態コード送信(OK)
            snd1byte(0x00);
        }
        else
        {
            snd1byte(0xf2);
            sdinit();
        }
    }
    else
    {
        snd1byte(0xf1);
        sdinit();
    }
}

//write file append open
RT_HANDLER void sd_waopen(void)
{
    receive_name(w_name);
    //ファイルオープン
    pc8001_open_write(w_name, false);
    if (true == w_file)
    {
        //状態コード送信(OK)
        snd1byte(0x00);
    }
    else
    {
        snd1byte(0xf0);
        sdinit();
    }
}

//write file new open
RT_HANDLER void sd_wnopen(void)
{
    receive_name(w_name);
    pc8001_open_write(w_name, true);
    if (true == w_file)
    {
        //状態コード送信(OK)
        snd1byte(0x00);
    }
    else
    {
        snd1byte(0xf0);
        sdinit();
    }
}

//write 1byte 5F2FH代替
RT_HANDLER void sd_w1byte(void)
{
    int rdata = rcv1byte();
    if (true == w_file)
    {
        w_file.write(rdata);
        //状態コード送信(OK)
        snd1byte(0x00);
    }
    else
    {
        snd1byte(0xf0);
        sdinit();
    }
}

//5ED9H代替
RT_HANDLER void sd_wdirect(void)
{
    if (true == w_file)
    {
        w_body();
        w_file.close();
        //状態コード送信(OK)
        snd1byte(0x00);
    }
    else
    {
        snd1byte(0xf0);
        sdinit();
    }
}

//write file close
RT_HANDLER void sd_wclose(void)
{
    w_file.close();
}

// SD-CARDのFILELIST
RT_HANDLER void dirlist(void)
{
    //比較文字列取得 32+1文字まで

    for (unsigned int lp1 = 0; lp1 <= 32; lp1++)
    {
        c_name[lp1] = rcv1byte();
    }
    //
    BridgeFile file = SD.open("/");
    if (true == file)
    {
        //状態コード送信(OK)
        snd1byte(0x00);

        BridgeFile entry = file.openNextFile();
        int cntl2 = 0;
        unsigned int br_chk = 0;
        int page = 1;
        //全件出力の場合には16件出力したところで一時停止、キー入力により継続、打ち切りを選択
        while (br_chk == 0)
        {
            if (entry)
            {
                entry.getName(f_name, 36);
                unsigned int lp1 = 0;
                //一件送信
                //比較文字列でファイルネームを先頭10文字まで比較して一致するものだけを出力
                if (f_match(f_name, c_name))
                {
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
            if (!entry || cntl2 > 15)
            {
                //継続・打ち切り選択指示要求
                snd1byte(0xfe);

                //選択指示受信(0:継続 B:前ページ 以外:打ち切り)
                br_chk = rcv1byte();
                //前ページ処理
                if (br_chk == 0x42)
                {
                    //先頭ファイルへ
                    file.rewindDirectory();
                    //entry値更新
                    entry = file.openNextFile();
                    //もう一度先頭ファイルへ
                    file.rewindDirectory();
                    if (page <= 2)
                    {
                        //現在ページが1ページ又は2ページなら1ページ目に戻る処理
                        page = 0;
                    }
                    else
                    {
                        //現在ページが3ページ以降なら前々ページまでのファイルを読み飛ばす
                        page = page - 2;
                        cntl2 = 0;
                        while (cntl2 < page * 16)
                        {
                            entry = file.openNextFile();
                            if (f_match(f_name, c_name))
                            {
                                cntl2++;
                            }
                        }
                    }
                    br_chk = 0;
                }
                page++;
                cntl2 = 0;
            }
            //ファイルがまだあるなら次読み込み、なければ打ち切り指示
            if (entry)
            {
                entry = file.openNextFile();
            }
            else
            {
                br_chk = 1;
            }
            //FDLの結果が18件未満なら継続指示要求せずにそのまま終了
            if (!entry && cntl2 < 16 && page == 1)
            {
                break;
            }
        }
        //処理終了指示
        snd1byte(0xFF);
        snd1byte(0x00);
    }
    else
    {
        snd1byte(0xf1);
    }
}

//f_nameとc_nameをc_nameに0x00が出るまで比較
//FILENAME COMPARE
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

RT_HANDLER void loop()
{
    digitalWrite(PB0PIN, LOW);
    digitalWrite(PB1PIN, LOW);
    digitalWrite(PB2PIN, LOW);
    digitalWrite(PB3PIN, LOW);
    digitalWrite(PB4PIN, LOW);
    digitalWrite(PB5PIN, LOW);
    digitalWrite(PB6PIN, LOW);
    digitalWrite(PB7PIN, LOW);
    digitalWrite(FLGPIN, LOW);
    //コマンド取得待ち
    ////    Serial.println("COMMAND WAIT");
    rt_wait_command();
    byte cmd = rcv1byte();
    rt_prepare(cmd);
    ////    Serial.println(cmd,HEX);
    if (eflg == false)
    {
        switch (cmd)
        {
            //70h:PC-8001 CMTファイル SAVE
        case 0x70:
            ////    Serial.println("CMT SAVE START");
            //状態コード送信(OK)
            snd1byte(0x00);
            cmt_save();
            break;
            //71h:PC-8001 CMTファイル LOAD
        case 0x71:
            ////    Serial.println("CMT LOAD START");
            //状態コード送信(OK)
            snd1byte(0x00);
            cmt_load();
            break;
            //72h:PC-8001 5F9EH READ ONE BYTE FROM CMT
        case 0x72:
            ////    Serial.println("CMT_5F9E START");
            //状態コード送信(OK)
            snd1byte(0x00);
            cmt_5f9e();
            break;
            //73h:PC-8001 CMTファイル BASIC LOAD
        case 0x73:
            ////    Serial.println("CMT BASIC LOAD START");
            //状態コード送信(OK)
            snd1byte(0x00);
            bas_load();
            break;
            //74h:PC-8001 CMTファイル BASIC SAVE
        case 0x74:
            ////    Serial.println("CMT BASIC SAVE START");
            //状態コード送信(OK)
            snd1byte(0x00);
            bas_save();
            break;
            //75h:PC-8001 CMTファイル KILL
        case 0x75:
            ////    Serial.println("CMT KILL START");
            //状態コード送信(OK)
            snd1byte(0x00);
            bas_kill();
            break;
            //76h:PC-8001 SD FILE READ OPEN
        case 0x76:
            ////    Serial.println("SD FILE READ OPEN START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_ropen();
            break;
            //77h:PC-8001 SD FILE WRITE APPEND OPEN
        case 0x77:
            ////    Serial.println("SD FILE WRITE APPEND OPEN START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_waopen();
            break;
            //78h:PC-8001 SD FILE WRITE 1Byte
        case 0x78:
            ////    Serial.println("SD FILE WRITE 1Byte START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_w1byte();
            break;
            //79h:PC-8001 SD FILE WRITE NEW OPEN
        case 0x79:
            ////    Serial.println("SD FILE WRITE NEW OPEN START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_wnopen();
            break;
            //7Ah:PC-8001 SD WRITE 5ED9H
        case 0x7A:
            ////    Serial.println("SD WRITE 5ED9H START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_wdirect();
            break;
            //7Bh:PC-8001 SD FILE WRITE CLOSE
        case 0x7B:
            ////    Serial.println("SD FILE WRITE CLOSE START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_wclose();
            break;

            //7Dh:PC-8001 SD CHECK
        case 0x7D:
            ////    Serial.println("SD CHECK START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_check();
            break;

            //7Eh:PC-8001 SD FILE EXIST CHECK
        case 0x7E:
            ////    Serial.println("SD FILE EXIST CHECK START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_exist();
            break;

            //7Fh:PC-8001 SD FILE DELETE
        case 0x7F:
            ////    Serial.println("SD FILE DELETE START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sd_delete();
            break;

            //83hでファイルリスト出力
        case 0x83:
            ////    Serial.println("FILE LIST START");
            //状態コード送信(OK)
            snd1byte(0x00);
            sdinit();
            dirlist();
            break;
        default:
            //状態コード送信(CMD ERROR)
            snd1byte(0xF4);
        }
    }
    else
    {
        //状態コード送信(ERROR)
        snd1byte(0xF0);
        sdinit();
    }
    rt_finish();
}
