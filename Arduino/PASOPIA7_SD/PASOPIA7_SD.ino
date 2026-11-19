// Runtime SD / PC backend for PASOPIA7; original machine protocol is retained.
#include <Arduino.h>
#define RETRO_PROFILE 7
#define RT_PROFILE 7
#define RETRO_IDLE_LEVEL LOW
#define CHKPIN 15
#define RT_BUFFER bridgeBuffer
#include "RetroStorage.h"
#include "RetroUart.h"
#include "SdFat.h"
#include <SPI.h>
BridgeSd SD;
char m_name[40];
char f_name[40];
char c_name[40];
// Store directory entry indexes instead of ten filename copies.
uint16_t sdir[10];
uint16_t sdirValid = 0;
BridgeFile file_r, file_w;
byte f_length2, f_length1;
unsigned long r_count = 0;
unsigned int f_length = 0;

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
    uint8_t state[4] = {rt_backend, RT_PROFILE, rt_flags(), 87};
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
#include "RetroSdBrowser.h"
#include "RetroSdFolders.h"

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

    if (op >= 0x73 && op <= 0x76)
    {
        uint16_t id = rt_header[1] | (uint16_t)rt_header[2] << 8;
        uint8_t result;
        if (rt_cached && id == rt_cached_id)
        {
            result = (op == rt_cached_op && rt_rx_crc == rt_cached_crc) ? rt_cached_result : 0xf4;
        }
        else
        {
            result = rt_upload_request(op, rt_rx_length, allowSwitch && rt_rx_allowed);
            rt_cached = true;
            rt_cached_id = id;
            rt_cached_op = op;
            rt_cached_crc = rt_rx_crc;
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
    ////  Serial.begin(9600);
    // CS=pin10
    // pin10 output

    pinMode(CABLESELECTPIN, OUTPUT);
    pinMode(CHKPIN, INPUT);  // CHK
    pinMode(PB0PIN, OUTPUT); // 送信データ
    pinMode(PB1PIN, OUTPUT); // 送信データ
    pinMode(PB2PIN, OUTPUT); // 送信データ
    pinMode(PB3PIN, OUTPUT); // 送信データ
    pinMode(PB4PIN, OUTPUT); // 送信データ
    pinMode(PB5PIN, OUTPUT); // 送信データ
    pinMode(PB6PIN, OUTPUT); // 送信データ
    pinMode(PB7PIN, OUTPUT); // 送信データ
    pinMode(FLGPIN, OUTPUT); // FLG

    pinMode(PA0PIN, INPUT_PULLUP); // 受信データ
    pinMode(PA1PIN, INPUT_PULLUP); // 受信データ
    pinMode(PA2PIN, INPUT_PULLUP); // 受信データ
    pinMode(PA3PIN, INPUT_PULLUP); // 受信データ

    digitalWrite(PB0PIN, LOW);
    digitalWrite(PB1PIN, LOW);
    digitalWrite(PB2PIN, LOW);
    digitalWrite(PB3PIN, LOW);
    digitalWrite(PB4PIN, LOW);
    digitalWrite(PB5PIN, LOW);
    digitalWrite(PB6PIN, LOW);
    digitalWrite(PB7PIN, LOW);
    digitalWrite(FLGPIN, LOW);

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
    digitalWrite(PB0PIN, (i_data) & 0x01);
    digitalWrite(PB1PIN, (i_data >> 1) & 0x01);
    digitalWrite(PB2PIN, (i_data >> 2) & 0x01);
    digitalWrite(PB3PIN, (i_data >> 3) & 0x01);
    digitalWrite(PB4PIN, (i_data >> 4) & 0x01);
    digitalWrite(PB5PIN, (i_data >> 5) & 0x01);
    digitalWrite(PB6PIN, (i_data >> 6) & 0x01);
    digitalWrite(PB7PIN, (i_data >> 7) & 0x01);
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

// ファイル名の最後が「.bin」なら0,「.bas」なら1,どちらでもなければ0xFF
RT_HANDLER byte extchk(char *f_name)
{
    unsigned int lp1 = 0;
    byte r_data = 0xFF;
    while (f_name[lp1] != 0x00)
    {
        //    m_name[lp1] = f_name[lp1];
        lp1++;
    }
    if (f_name[lp1 - 4] == '.' && (f_name[lp1 - 3] == 'B' || f_name[lp1 - 3] == 'b') &&
        (f_name[lp1 - 2] == 'I' || f_name[lp1 - 2] == 'i') &&
        (f_name[lp1 - 1] == 'N' || f_name[lp1 - 1] == 'n'))
    {
        r_data = 0;
    }
    if (f_name[lp1 - 4] == '.' && (f_name[lp1 - 3] == 'B' || f_name[lp1 - 3] == 'b') &&
        (f_name[lp1 - 2] == 'A' || f_name[lp1 - 2] == 'a') &&
        (f_name[lp1 - 1] == 'S' || f_name[lp1 - 1] == 's'))
    {
        r_data = 1;
    }
    if (f_name[lp1 - 4] == '.' && (f_name[lp1 - 3] == 'F' || f_name[lp1 - 3] == 'f') &&
        (f_name[lp1 - 2] == 'P' || f_name[lp1 - 2] == 'p') &&
        (f_name[lp1 - 1] == 'T' || f_name[lp1 - 1] == 't'))
    {
        r_data = 2;
    }
    return (r_data);
}

// ファイル名の最後が「.bin」でなければ付加
RT_HANDLER void addbin(char *f_name, char *m_name)
{
    unsigned int lp1 = 0;
    while (f_name[lp1] != 0x00)
    {
        m_name[lp1] = f_name[lp1];
        ////  Serial.println(f_name[lp1],HEX);
        lp1++;
    }
    if (f_name[lp1 - 4] != '.' || (f_name[lp1 - 3] != 'B' && f_name[lp1 - 3] != 'b') ||
        (f_name[lp1 - 2] != 'I' && f_name[lp1 - 2] != 'i') ||
        (f_name[lp1 - 1] != 'N' && f_name[lp1 - 1] != 'n'))
    {
        m_name[lp1++] = '.';
        m_name[lp1++] = 'b';
        m_name[lp1++] = 'i';
        m_name[lp1++] = 'n';
    }
    m_name[lp1] = 0x00;
}

// ファイル名の最後が「.bas」でなければ付加
RT_HANDLER void addbas(char *f_name, char *m_name)
{
    unsigned int lp1 = 0;
    while (f_name[lp1] != 0x00)
    {
        m_name[lp1] = f_name[lp1];
        lp1++;
    }
    if (f_name[lp1 - 4] != '.' || (f_name[lp1 - 3] != 'B' && f_name[lp1 - 3] != 'b') ||
        (f_name[lp1 - 2] != 'A' && f_name[lp1 - 2] != 'a') ||
        (f_name[lp1 - 1] != 'S' && f_name[lp1 - 1] != 's'))
    {
        m_name[lp1++] = '.';
        m_name[lp1++] = 'b';
        m_name[lp1++] = 'a';
        m_name[lp1++] = 's';
    }
    m_name[lp1] = 0x00;
}

// 比較文字列取得 32+1文字まで取得、ただしダブルコーテーションは無視する
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
    ////  Serial.println(f_name);
}

// SDカードから機械語を読込
RT_HANDLER void fm_load(void)
{
    int wk1 = 0;
    unsigned int lp1;
    // ファイルネーム取得
    receive_name(f_name);
    addbin(f_name, m_name);
    ////  Serial.println(m_name);
    // ファイルが存在しなければERROR
    if (SD.exists(m_name) == true)
    {
        // ファイルオープン
        file_r = SD.open(m_name, FILE_READ);
        if (true == file_r)
        {
            // 状態コード送信(OK)
            snd1byte(0x00);

            // ファイルサイズ取得
            f_length = file_r.size() - 4;
            f_length2 = f_length % 256;
            f_length1 = f_length / 256;
            snd1byte(f_length2);
            snd1byte(f_length1);

            //          if(rcv1byte()==0x00){
            // データ送信
            wk1 = file_r.read();
            snd1byte(wk1);
            wk1 = file_r.read();
            snd1byte(wk1);
            wk1 = file_r.read();
            wk1 = file_r.read();
            if (f_length > 0)
            {
                for (lp1 = 1; lp1 <= f_length; lp1++)
                {
                    wk1 = file_r.read();
                    snd1byte(wk1);
                }
                //            }
            }
            file_r.close();
            sdinit();
        }
        else
        {
            // 状態コード送信(ERROR)
            snd1byte(0xFF);
            sdinit();
        }
    }
    else
    {
        // 状態コード送信(FILE NOT FIND ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

// SDカードからBASICプログラムを読込
RT_HANDLER void fb_load(void)
{
    int wk1 = 0;
    unsigned int lp1;
    // ファイルネーム取得
    receive_name(f_name);
    addbas(f_name, m_name);
    ////  Serial.println(m_name);
    // ファイルが存在しなければERROR
    if (SD.exists(m_name) == true)
    {
        // ファイルオープン
        file_r = SD.open(m_name, FILE_READ);
        if (true == file_r)
        {
            // 状態コード送信(OK)
            snd1byte(0x00);

            // ファイルサイズ取得
            f_length = file_r.size();
            f_length2 = f_length % 256;
            f_length1 = f_length / 256;
            snd1byte(f_length2);
            snd1byte(f_length1);

            if (rcv1byte() == 0x00)
            {
                // データ送信
                if (f_length > 0)
                {
                    for (lp1 = 1; lp1 <= f_length; lp1++)
                    {
                        wk1 = file_r.read();
                        snd1byte(wk1);
                    }
                }
            }
            file_r.close();
            sdinit();
        }
        else
        {
            // 状態コード送信(ERROR)
            snd1byte(0xFF);
            sdinit();
        }
    }
    else
    {
        // 状態コード送信(FILE NOT FIND ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

// SDカードにBASICプログラムを書き込み
RT_HANDLER void fb_save(void)
{
    int wk1 = 0;
    unsigned long lp1;
    // ファイルネーム取得
    receive_name(f_name);
    addbas(f_name, m_name);
    //
    // ファイルが存在すればdelete
    if (SD.exists(m_name) == true)
    {
        SD.remove(m_name);
    }
    // ファイルオープン
    file_w = SD.open(m_name, FILE_WRITE);
    if (true == file_w)
    {
        // 状態コード送信(OK)
        snd1byte(0x00);
        f_length1 = rcv1byte();
        f_length2 = rcv1byte();
        f_length = f_length2 * 256 + f_length1;

        if (f_length > 0)
        {
            for (lp1 = 1; lp1 <= f_length; lp1++)
            {
                wk1 = rcv1byte();
                file_w.write(wk1);
            }
        }
        file_w.close();
    }
    else
    {
        // 状態コード送信(ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

// SDカードに機械語を書き込み
RT_HANDLER void fm_save(void)
{
    int wk1 = 0;
    unsigned int lp1, s_adrs, g_adrs;
    byte s_adrs1, s_adrs2, g_adrs1, g_adrs2;
    // ファイルネーム取得
    receive_name(f_name);
    addbin(f_name, m_name);
    //
    // ファイルが存在すればdelete
    if (SD.exists(m_name) == true)
    {
        SD.remove(m_name);
    }
    // ファイルオープン
    file_w = SD.open(m_name, FILE_WRITE);
    if (true == file_w)
    {
        // 状態コード送信(OK)
        snd1byte(0x00);
        f_length1 = rcv1byte();
        f_length2 = rcv1byte();
        f_length = f_length2 * 256 + f_length1;

        s_adrs1 = rcv1byte();
        s_adrs2 = rcv1byte();
        s_adrs = s_adrs2 * 256 + s_adrs1;

        f_length = f_length - s_adrs + 1;
        f_length2 = f_length / 256;
        f_length1 = f_length % 256;
        file_w.write(s_adrs1);
        file_w.write(s_adrs2);
        file_w.write(f_length1);
        file_w.write(f_length2);

        if (f_length > 0)
        {
            for (lp1 = 1; lp1 <= f_length; lp1++)
            {
                wk1 = rcv1byte();
                file_w.write(wk1);
            }
        }
        file_w.close();
    }
    else
    {
        // 状態コード送信(ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

// f_nameとc_nameをc_nameに0x00が出るまで比較
// FILENAME COMPARE
boolean f_match(char *f_name, char *c_name)
{
    boolean flg1 = true;
    unsigned int lp1 = 0;
    while (lp1 <= 32 && c_name[0] != 0x00 && flg1 == true)
    {
        if (upper(f_name[lp1]) != upper(c_name[lp1]))
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

// 小文字->大文字
char upper(char c)
{
    if ('a' <= c && c <= 'z')
    {
        c = c - ('a' - 'A');
    }
    return c;
}

// SD-CARDのFILELIST
RT_HANDLER void dirlist(void)
{
    byte extflg = 0;
    sdirValid = 0;
    // 比較文字列取得 32+1文字まで
    receive_name(c_name);
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
                        snd1byte(f_name[lp1]);
                        lp1++;
                    }
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
                        extflg = extchk(f_name);
                        unsigned int lp2 = 0;
                        snd1byte(0xFD);
                        while (lp2 <= 36 && f_name[lp2] != 0x00)
                        {
                            snd1byte(f_name[lp2]);
                            lp2++;
                        }
                        snd1byte(0x00);
                        file_r.close();
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
        snd1byte(extflg);
    }
    else
    {
        snd1byte(0xf1);
    }
    sdirValid = 0;
}

RT_HANDLER void loop()
{
    digitalWrite(PB2PIN, LOW);
    digitalWrite(PB3PIN, LOW);
    digitalWrite(FLGPIN, LOW);
    // コマンド取得待ち
    ////  Serial.print("cmd:");
    rt_wait_command();
    byte cmd = rcv1byte();
    rt_prepare(cmd);
    ////  Serial.println(cmd,HEX);
    if (eflg == false)
    {
        switch (cmd)
        {
            // 22hでSDカードからBinary Load
        case 0x22:
            // 状態コード送信(OK)
            ////  Serial.println("BINARY LOAD START");
            // 状態コード送信(OK)
            snd1byte(0x00);
            fm_load();
            break;
            // 23hでSDカードからBASIC Program Load
        case 0x23:
            // 状態コード送信(OK)
            ////  Serial.println("BASIC LOAD START");
            // 状態コード送信(OK)
            snd1byte(0x00);
            fb_load();
            break;
            // 24hでSDカードにBinary save
        case 0x24:
            // 状態コード送信(OK)
            ////  Serial.println("Binary SAVE START");
            // 状態コード送信(OK)
            snd1byte(0x00);
            fm_save();
            break;
            // 25hでSDカードにBASIC Program save
        case 0x25:
            // 状態コード送信(OK)
            ////  Serial.println("BASIC Program SAVE START");
            // 状態コード送信(OK)
            snd1byte(0x00);
            fb_save();
            break;
            // 21hでSDカードファイル一覧
        case 0x21:
            // 状態コード送信(OK)
            ////  Serial.println("SD DIR START");
            // 状態コード送信(OK)
            snd1byte(0x00);
            sdinit();
            dirlist();
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
