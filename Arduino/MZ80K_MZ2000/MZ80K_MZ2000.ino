// MZ-80K PC LINK: same MZ wire protocol / unmodified ROM.
#include <Arduino.h>
// 2022. 1.24 ファイルネームの後ろの20h詰めを0dhに修正するための処理をArduino側からMZ-80K側に修正
//           比較演算子の記述を見直し
// 2022. 1.25 各コマンド受信時のdelay()を廃止
// 2022. 1.26 FDコマンドでロード可能なファイル種類コードは0x01のみとしていた制限を撤廃
// 2022. 1.29 FDPコマンドのbug修正
// 2022. 1.30 FDLコマンド仕様変更 FDL A～Zの場合、ファイル名先頭一文字を比較して一致したものだけを出力
// 2022. 1.31 FDLコマンド仕様変更 FDL xの場合、ファイル名先頭一文字を比較して一致したものだけを出力
//            Bキーで前の20件を表示
// 2022. 2. 2 DOSファイル名がアルファベット小文字でもFDL xで検索できるように修正
// 2022. 2. 4 MZ-1200対策　初期化時にdelay(1000)を追加
// 2022. 2. 8 FDLコマンド仕様変更 FDL xの場合、ファイル名先頭1文字～32文字までに拡張
// 2023. 6.19 MZ-2000_SDの起動方式追加によりBOOT LOADER読み込みを追加。MZ-80K_SDには影響なし。
// 2024. 3. 4 sd-card再挿入時の初期化処理を追加
// 2025.12. 4 mon_ldata処理で不要なaddmztが入っていたため削除
// 2026. 8. 8 連番複数ファイルの多段ロード対応
// 2026. 8. 8 IFB、DATA、DATA…方式多段ロード対応
// 2026. 8. 9 IFB、ダミーヘッダ、DATA、ダミーヘッダ、DATA…方式多段ロード対応(本来要らないダミーヘッダがついたもの)
//            IFB、DATA、IFB、DATA…方式多段ロード対応(MZTファイルを単純にマージしたもの)
//
#include "SdFat.h"
#include "RetroNativeFile.h"
#include <SPI.h>
// Parse library headers before replacing the sketch UART name.
#include "RetroUart.h"
SdFat SD;

static MZNativeFile mz_open_native(const char *name, int flags = FILE_READ)
{
    MZNativeFile result;
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    result.open(SD.vol(), name, flags);
#else
    result.open(name, flags);
#endif
    return result;
}

unsigned long m_lop = 128;
char m_name[40];
char k_name[40];
byte s_data[260];
char f_name[40];
char c_name[40];
char new_name[40];
int kflg = 0;
unsigned long f_end;

#define CABLESELECTPIN (10)
#define CHKPIN (15)
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

#define RT_PROFILE 1
#define RT_BUFFER s_data

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
    return SD.begin(CABLESELECTPIN, (uint32_t)F_CPU / 8UL);
#else
    return SD.begin(CABLESELECTPIN, 8);
#endif
}

// Runtime SD/PC selection v3. Starts in SD on every reset; no EEPROM fallback.
// RT_PROFILE=0: MZ2500 CHK idle HIGH; =1: MZ80K/MZ2000 idle LOW.
static uint8_t rt_backend = 0;
// Host -> Arduino SD copy. Uses the existing transfer buffer only while idle.
static MZNativeFile rt_upload_file;
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
    if (SD.card())
    {
        SD.card()->error(0);
    }
#endif
}

static void rt_fault_capture(uint8_t stage)
{
    rt_fault_stage = stage;
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    rt_fault_code = SD.sdErrorCode();
    rt_fault_data = SD.sdErrorData();
#endif
}

static bool rt_rename(const char *from, const char *to)
{
    rt_card_error_clear();
    MZNativeFile f = mz_open_native(from, O_WRONLY);
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
        SD.remove(rt_temp_name);
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
    if (!allowed || rt_busy || rt_write_left || rt_line_active())
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
        sdinit();
        if (eflg)
            return 0xf2;
        // A previous attempt may have left the disposable staging file.
        // Recreate it for this new request; never remove the backup here.
        if (SD.exists(rt_temp_name))
        {
            rt_card_error_clear();
            if (!SD.remove(rt_temp_name))
            {
                rt_fault_capture(7);
                return 0xef;
            }
        }
        if (SD.exists(rt_backup_name))
        {
            return 0xf9;
        }
        rt_replace = (p[0] & 1) != 0;
        if (!rt_replace && SD.exists(rt_upload_name))
            return 0xf6;
        rt_upload_size = rt_u32(p + 1);
        rt_upload_at = 0;
        rt_upload_file = mz_open_native(rt_temp_name, O_WRONLY | O_CREAT | O_TRUNC);
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
        // and allocating an extra MZNativeFile on the rename call stack.
        bool backup = false;
        rt_card_error_clear();
        bool destinationExists = SD.exists(rt_upload_name);
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
        if (SD.sdErrorCode() != 0)
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
            SD.remove(rt_backup_name); // On failure leave the old file recoverable.
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

// Shared frame-byte serialization; CRC and wire bytes are unchanged.
static RT_HANDLER uint16_t rt_write_crc(uint16_t crc, uint8_t value)
{
    Serial.write(value);
    return rt_crc_byte(crc, value);
}

// One serializer is shared by status replies, browser replies and PC RPCs.
static RT_HANDLER uint16_t rt_frame_bytes(uint16_t crc, const uint8_t *data, uint16_t length)
{
    for (uint16_t index = 0; index < length; ++index)
        crc = rt_write_crc(crc, data[index]);
    return crc;
}

static RT_HANDLER uint16_t rt_frame_begin(uint8_t op, uint16_t id, uint8_t status, uint16_t length)
{
    uint8_t header[7] = {
        1, (uint8_t)id, (uint8_t)(id >> 8), op, status, (uint8_t)length, (uint8_t)(length >> 8)};
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
    return digitalRead(CHKPIN) != (RT_PROFILE ? LOW : HIGH);
}

static uint8_t rt_flags()
{
    return (rt_busy || rt_line_active() ? 1 : 0) | (rt_write_left ? 2 : 0) | (!eflg ? 4 : 0) |
           (rt_upload ? 8 : 0);
}

static void rt_emit(uint8_t op, uint16_t id, uint8_t status)
{
    bool diagnostic = rt_diag_requested && (op == 0xf3 || op == 0xf4 || op == 0xf5) &&
                      (status == 0xef || status == 0xf6 || (status >= 0xf8 && status <= 0xfe));
    uint8_t error = 0, detail = 0;
#if defined(SD_FAT_VERSION) && SD_FAT_VERSION >= 20000
    error = SD.sdErrorCode();
    detail = SD.sdErrorData();
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
            rt_upload_file = mz_open_native(subject, FILE_READ);
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
        fat = SD.fatType();
#if SDFAT_FILE_TYPE == 1
        if (fat == 12 || fat == 16)
        {
            total = SD.rootDirEntryCount();
            rt_upload_file = mz_open_native("/", FILE_READ);
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
        rt_cached = false;
    if (op == 0x70)
    {
        if (value > 1)
            result = 0xf4;
        else if (!allowSwitch || rt_busy || rt_write_left || rt_upload || rt_line_active())
            result = 0xf7;
        else
        {
            rt_backend = value;
            if (value == 0)
                sdinit();
        }
    }
    rt_emit(op | 0x80, id, result);
}

static bool rt_browser_sd_ready = false;
#define RT_BROWSER_SD SD
#define RT_BROWSER_READY (rt_backend ? rt_browser_sd_ready : !eflg)
#define RT_BROWSER_SET_READY(value)                                                                          \
    do                                                                                                       \
    {                                                                                                        \
        rt_browser_sd_ready = (value);                                                                       \
        if (rt_backend == 0)                                                                                 \
            eflg = !(value);                                                                                 \
    } while (0)
#define RT_BROWSER_WRITING false
#define RT_BROWSER_DELETE_BUSY false
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
            RT_BUFFER[rt_rx_at] = v;
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
    rt_poll(false);
    rt_busy = false;
    rt_emit(0x72, 0, 0);
}

void sdinit(void)
{
    // SD初期化
    if (!rt_sd_begin())
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

static RT_HANDLER void mz_clear_outputs()
{
#if PB1PIN == PB0PIN + 1 && PB2PIN == PB0PIN + 2 && PB3PIN == PB0PIN + 3 && PB4PIN == PB0PIN + 4 &&          \
    PB5PIN == PB0PIN + 5 && PB6PIN == PB0PIN + 6 && PB7PIN == PB0PIN + 7
    for (uint8_t pin = PB0PIN; pin <= PB7PIN; ++pin)
        digitalWrite(pin, LOW);
#else
    digitalWrite(PB0PIN, LOW);
    digitalWrite(PB1PIN, LOW);
    digitalWrite(PB2PIN, LOW);
    digitalWrite(PB3PIN, LOW);
    digitalWrite(PB4PIN, LOW);
    digitalWrite(PB5PIN, LOW);
    digitalWrite(PB6PIN, LOW);
    digitalWrite(PB7PIN, LOW);
#endif
    digitalWrite(FLGPIN, LOW);
}

void setup()
{
    Serial.begin(115200);
    ////  Serial.begin(9600);
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
    pinMode(PA3PIN, INPUT_PULLUP); //受信データ

    mz_clear_outputs();

    // 2022. 2. 4 MZ-1200対策
    delay(1500);

    sdinit();
    // Boot-only event; Windows clears interrupted file and SD operations.
    rt_emit(0x73, 0, 0);
}

//4BIT受信
byte rcv4bit(void)
{
    //HIGHになるまでループ
    while (digitalRead(CHKPIN) != HIGH)
    {
        rt_poll(false);
    }
    //受信
    byte j_data =
        digitalRead(PA0PIN) + digitalRead(PA1PIN) * 2 + digitalRead(PA2PIN) * 4 + digitalRead(PA3PIN) * 8;
    //FLGをセット
    digitalWrite(FLGPIN, HIGH);
    //LOWになるまでループ
    while (digitalRead(CHKPIN) == HIGH)
    {
        rt_poll(false);
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
        rt_poll(false);
    }
    digitalWrite(FLGPIN, LOW);
    //LOWになるまでループ
    while (digitalRead(CHKPIN) == HIGH)
    {
        rt_poll(false);
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

//多段ロードチェック「.mzt」を除いたファイル名の最後に「#?」(?=1～9)が付加されていれば多段ロード(kflg=1)
//                                  ファイル名の最後が「#」なら複数MZTファイルをマージしたmzt(kflg=2)
//                                  ファイル名の最後が「$」ならダミーヘッダ付き複合mzt(kflg=3)
void kchk(char *r_name, char *w_name)
{
    unsigned int lp1 = 0;

    strcpy(r_name, w_name);
    while (w_name[lp1] != 0x0D)
    {
        lp1++;
    }
    kflg = 0;
    if (w_name[lp1 - 7] == '#')
    {
        if (w_name[lp1 - 6] >= '1' && w_name[lp1 - 6] <= '9')
        {
            r_name[lp1 - 6] = w_name[lp1 - 6] + 1;
            //次番号のファイルが存在すれば継続ロードフラグ kflg=1
            if (SD.exists(k_name) == true)
            {
                kflg = 1;
            }
        }
    }
    else if (w_name[lp1 - 6] == '#')
    {
        kflg = 2;
    }
    else if (w_name[lp1 - 6] == '$')
    {
        kflg = 3;
    }
}

//ファイル名の最後が「.mzt」でなければ付加
static RT_HANDLER bool mz_has_mzt_extension(const char *name, uint8_t length)
{
    return length >= 4 && name[length - 4] == '.' && ((uint8_t)name[length - 3] & 0xdf) == 'M' &&
           ((uint8_t)name[length - 2] & 0xdf) == 'Z' && ((uint8_t)name[length - 1] & 0xdf) == 'T';
}

void addmzt(char *w_name)
{
    unsigned int lp1 = 0;
    while (w_name[lp1] != 0x0D)
    {
        lp1++;
    }
    if (!mz_has_mzt_extension(w_name, (uint8_t)lp1))
    {
        w_name[lp1++] = '.';
        w_name[lp1++] = 'm';
        w_name[lp1++] = 'z';
        w_name[lp1++] = 't';
    }
    w_name[lp1] = 0x00;
}

//SDカードにSAVE
static RT_HANDLER void mz_receive_name(char *name)
{
    for (unsigned int index = 0; index <= 32; ++index)
        name[index] = rcv1byte();
}

// Receive machine-side save data in bounded 256-byte blocks.
// On AVR the original zero 16-bit length denotes 65536 bytes here.
#if defined(__AVR__)
#define MZ_SHARED __attribute__((noinline, noclone))
#else
#define MZ_SHARED
#endif
static MZ_SHARED void mz_transfer_file(MZNativeFile &writer, uint16_t remaining)
{
    do
    {
        uint16_t count = remaining == 0 || remaining > 256 ? 256 : remaining;
        for (uint16_t index = 0; index < count; ++index)
            s_data[index] = rcv1byte();
        writer.write(s_data, count);
        remaining = (uint16_t)(remaining - count);
    } while (remaining != 0);
}

static void mz_receive_file(MZNativeFile &file, unsigned int f_length)
{
    mz_transfer_file(file, (uint16_t)f_length);
}

static RT_HANDLER int mz_read_send(MZNativeFile &file)
{
    int value = file.read();
    snd1byte(value);
    return value;
}

RT_HANDLER void f_save(void)
{
    char p_name[20];

    //保存ファイルネーム取得
    mz_receive_name(f_name);
    addmzt(f_name);
    //プログラムネーム取得
    for (unsigned int lp1 = 0; lp1 <= 16; lp1++)
    {
        p_name[lp1] = rcv1byte();
    }
    p_name[15] = 0x0D;
    p_name[16] = 0x00;
    //スタートアドレス取得
    int s_adrs1 = rcv1byte();
    int s_adrs2 = rcv1byte();
    //スタートアドレス算出
    unsigned int s_adrs = s_adrs1 + s_adrs2 * 256;
    //エンドアドレス取得
    int e_adrs1 = rcv1byte();
    int e_adrs2 = rcv1byte();
    //エンドアドレス算出
    unsigned int e_adrs = e_adrs1 + e_adrs2 * 256;
    //実行アドレス取得
    int g_adrs1 = rcv1byte();
    int g_adrs2 = rcv1byte();
    //実行アドレス算出
    unsigned int g_adrs = g_adrs1 + g_adrs2 * 256;
    //ファイルサイズ算出
    unsigned int f_length = e_adrs - s_adrs + 1;
    unsigned int f_length1 = f_length % 256;
    unsigned int f_length2 = f_length / 256;
    //ファイルが存在すればdelete
    if (SD.exists(f_name) == true)
    {
        SD.remove(f_name);
    }
    //ファイルオープン
    MZNativeFile file = mz_open_native(f_name, FILE_WRITE);
    if (true == file)
    {
        //状態コード送信(OK)
        snd1byte(0x00);
        //ファイルモード設定(01)
        file.write((uint8_t)(0x01));
        //プログラムネーム
        file.write(p_name);
        file.write((uint8_t)(0x00));
        //ファイルサイズ
        file.write(f_length1);
        file.write(f_length2);
        //スタートアドレス
        file.write(s_adrs1);
        file.write(s_adrs2);
        //実行アドレス
        file.write(g_adrs1);
        file.write(g_adrs2);
        //7Fまで00埋め
        for (unsigned int lp1 = 0; lp1 <= 103; lp1++)
        {
            file.write((uint8_t)(0x00));
        }
        //実データ
        mz_receive_file(file, f_length);
        file.close();
    }
    else
    {
        //状態コード送信(ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

//SDカードから読込
RT_HANDLER void f_load(void)
{
    //ファイルネーム取得
    mz_receive_name(m_name);
    addmzt(m_name);
    //多段ロードチェック
    kchk(k_name, m_name);
    //ファイルが存在しなければERROR
    if (SD.exists(m_name) == true)
    {
        //ファイルオープン
        MZNativeFile file = mz_open_native(m_name, FILE_READ);
        if (true == file)
        {
            //ファイル種類コードの判別を撤廃
            //      if( file.read() == 0x01){
            //状態コード送信(OK)
            snd1byte(0x00);
            //DOSファイルのファイルサイズを更新
            f_end = file.size();
            int wk1 = 0;
            wk1 = file.read();
            for (unsigned int lp1 = 0; lp1 <= 16; lp1++)
            {
                wk1 = mz_read_send(file);
            }
            //ファイルサイズ取得
            int f_length2 = file.read();
            int f_length1 = file.read();
            unsigned int f_length = f_length1 * 256 + f_length2;
            //スタートアドレス取得
            int s_adrs2 = file.read();
            int s_adrs1 = file.read();
            unsigned int s_adrs = s_adrs1 * 256 + s_adrs2;
            //実行アドレス取得
            int g_adrs2 = file.read();
            int g_adrs1 = file.read();
            unsigned int g_adrs = g_adrs1 * 256 + g_adrs2;
            snd1byte(s_adrs2);
            snd1byte(s_adrs1);
            snd1byte(f_length2);
            snd1byte(f_length1);
            snd1byte(g_adrs2);
            snd1byte(g_adrs1);
            file.seek(128);
            //データ送信
            for (unsigned int lp1 = 0; lp1 < f_length; lp1++)
            {
                byte i_data = mz_read_send(file);
            }
            file.close();
            //ランチャーからのLOADでもIFB、DATA、DATAが正しく機能するように修正
            m_lop = 128 + f_length;
            //ダミーヘッダ付きの場合ダミーヘッダは読み飛ばし
            if (kflg == 3)
            {
                m_lop = m_lop + 128;
            }
            //       } else {
            //状態コード送信(ERROR)
            //        snd1byte(0xF2);
            sdinit();
            //      }
        }
        else
        {
            //状態コード送信(ERROR)
            snd1byte(0xFF);
            sdinit();
        }
    }
    else
    {
        //状態コード送信(FILE NOT FIND ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

// SD-CARDのFILELIST
RT_HANDLER void dirlist(void)
{
    //比較文字列取得 32+1文字まで
    for (unsigned int lp1 = 0; lp1 <= 32; lp1++)
    {
        c_name[lp1] = rcv1byte();
        ////  Serial.print(c_name[lp1],HEX);
        ////  Serial.println("");
    }
    //
    MZNativeFile file = mz_open_native("/");
    MZNativeFile entry = file.openNextFile();
    int cntl2 = 0;
    unsigned int br_chk = 0;
    int page = 1;
    //全件出力の場合には20件出力したところで一時停止、キー入力により継続、打ち切りを選択
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
        if (!entry || cntl2 > 19)
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
                    while (cntl2 < page * 20)
                    {
                        entry = file.openNextFile();
                        //          if (upper(f_name[0]) == pg0 || pg0 == 0x20){
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
        //FDLの結果が20件未満なら継続指示要求せずにそのまま終了
        if (!entry && cntl2 < 20 && page == 1)
        {
            break;
        }
    }
    //処理終了指示
    snd1byte(0xFF);
    snd1byte(0x00);
}

//f_nameとc_nameをc_nameに0x00が出るまで比較
//FILENAME COMPARE
boolean f_match(char *f_name, char *c_name)
{
    boolean flg1 = true;
    unsigned int lp1 = 0;
    ////  Serial.print(f_name);
    ////  Serial.print(" ");
    ////  Serial.print(c_name);
    ////  Serial.print(" ");
    while (lp1 <= 32 && c_name[0] != 0x00 && flg1 == true)
    {
        ////  Serial.print(f_name[lp1],HEX);
        ////  Serial.print("-");
        ////  Serial.print(c_name[lp1+1],HEX);
        ////  Serial.print(" ");
        if (upper(f_name[lp1]) != c_name[lp1 + 1])
        {
            flg1 = false;
        }
        lp1++;
        if (c_name[lp1 + 1] == 0x00)
        {
            break;
        }
    }
    ////  if (flg1){
    ////    Serial.println("true");
    ////  }else{
    ////    Serial.println("false");
    ////  }
    return flg1;
}

//91hで0436H MONITOR ライト インフォメーション代替処理
RT_HANDLER void mon_whead(void)
{
    char m_info[130];
    //インフォメーションブロック受信
    for (unsigned int lp1 = 0; lp1 < 128; lp1++)
    {
        m_info[lp1] = rcv1byte();
    }
    //S-OS SWORDからは最後が20hのファイルネームが送られて来るため0dhを付加
    //8080用テキスト・エディタ＆アセンブラからファイルネームの後ろに20hが送られて来るため0dhに修正
    //MZ-80K側で対処
    //  int lp2 = 17;
    //  while (lp2>0 && (m_info[lp2] ==0x20 || m_info[lp2] ==0x0d)){
    //    m_info[lp2]=0x0d;
    //    lp2--;
    //  }
    //ファイルネーム取り出し
    for (unsigned int lp1 = 0; lp1 < 17; lp1++)
    {
        m_name[lp1] = m_info[lp1 + 1];
    }
    //DOSファイルネーム用に.MZTを付加
    addmzt(m_name);
    m_info[16] = 0x0d;
    //ファイルが存在すればdelete
    if (SD.exists(m_name) == true)
    {
        SD.remove(m_name);
    }
    //ファイルオープン
    MZNativeFile file = mz_open_native(m_name, FILE_WRITE);
    if (true == file)
    {
        //状態コード送信(OK)
        snd1byte(0x00);
        //インフォメーションブロックwrite
        for (unsigned int lp1 = 0; lp1 < 128; lp1++)
        {
            file.write(m_info[lp1]);
        }
        file.close();
        rt_hold_begin((uint8_t)m_info[18] | (uint16_t)(uint8_t)m_info[19] << 8);
    }
    else
    {
        //状態コード送信(ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

//92hで0475H MONITOR ライト データ代替処理
RT_HANDLER void mon_wdata(void)
{
    //ファイルサイズ取得
    int f_length1 = rcv1byte();
    int f_length2 = rcv1byte();
    //ファイルサイズ算出
    unsigned int f_length = f_length1 + f_length2 * 256;
    //ファイルオープン
    MZNativeFile file = mz_open_native(m_name, FILE_WRITE);
    if (true == file)
    {
        //状態コード送信(OK)
        snd1byte(0x00);
        //実データ
        mz_receive_file(file, f_length);
        file.close();
        rt_hold_data(f_length ? f_length : 65536UL);
    }
    else
    {
        //状態コード送信(ERROR)
        snd1byte(0xF1);
    }
}

//04D8H MONITOR リード インフォメーション代替処理
RT_HANDLER void mon_lhead(void)
{
    //リード データ POINTクリア
    m_lop = 128;
    //ファイルネーム取得
    mz_receive_name(m_name);
    addmzt(m_name);
    //多段ロードチェック
    kchk(k_name, m_name);
    //ファイルが存在しなければERROR
    if (SD.exists(m_name) == true)
    {
        snd1byte(0x00);
        //ファイルオープン
        MZNativeFile file = mz_open_native(m_name, FILE_READ);
        if (true == file)
        {
            snd1byte(0x00);
            //DOSファイルのファイルサイズを更新
            f_end = file.size();
            for (unsigned int lp1 = 0; lp1 < 128; lp1++)
            {
                byte i_data = mz_read_send(file);
            }
            file.close();
            snd1byte(0x00);
        }
        else
        {
            //状態コード送信(ERROR)
            snd1byte(0xFF);
            sdinit();
        }
    }
    else
    {
        //状態コード送信(FILE NOT FIND ERROR)
        snd1byte(0xF1);
        sdinit();
    }
}

//継続ロード処理
RT_HANDLER void kflgchk(void)
{
    //リード データ POINTクリア
    //kflg=2の時は次のIFBまでスキップできるように設定
    if (kflg != 2)
    {
        m_lop = 128;
    }
    else
    {
        m_lop = m_lop + 128;
    }
    if (kflg == 1 || kflg == 2)
    {
        //多段ロードフラグ送信
        snd1byte(0x00);
        strcpy(m_name, k_name);
        //多段ロードチェック
        kchk(k_name, m_name);
        //ファイルが存在しなければERROR
        if (SD.exists(m_name) == true)
        {
            snd1byte(0x00);
            //ファイルオープン
            MZNativeFile file = mz_open_native(m_name, FILE_READ);
            if (true == file)
            {
                //DOSファイルのファイルサイズを更新
                f_end = file.size();
                snd1byte(0x00);
                //kflg=2の時だけm_lop-128読み飛ばし
                if (kflg == 2)
                {
                    file.seek(m_lop - 128);
                }

                for (unsigned int lp1 = 0; lp1 < 128; lp1++)
                {
                    byte i_data = mz_read_send(file);
                }
                file.close();
                snd1byte(0x00);
            }
            else
            {
                //状態コード送信(ERROR)
                snd1byte(0xFF);
                sdinit();
            }
        }
        else
        {
            //状態コード送信(FILE NOT FIND ERROR)
            snd1byte(0xF1);
            sdinit();
        }
    }
    else
    {
        snd1byte(0xFF);
    }
}

//04F8H MONITOR リード データ代替処理
RT_HANDLER void mon_ldata(void)
{
    // 2025.12.4 addmztが不要な処理であったため削除
    //  addmzt(m_name);
    //ファイルが存在しなければERROR
    ////  Serial.print("m_name:");
    ////  Serial.println(m_name);
    if (SD.exists(m_name) == true)
    {
        snd1byte(0x00);
        //ファイルオープン
        MZNativeFile file = mz_open_native(m_name, FILE_READ);
        ////  Serial.print("m_lop:");
        ////  Serial.println(m_lop);
        ////  Serial.print("f_end:");
        ////  Serial.println(f_end);
        //ファイルのオープン成功及び
        //DOSフィルのファイルサイズがファイル読み出しポイントより大きいことが必要
        if (true == file && f_end > m_lop)
        {
            snd1byte(0x00);
            file.seek(m_lop);
            ////  Serial.print("m_lop:");
            ////  Serial.println(m_lop);
            //読み出しサイズ取得
            int f_length2 = rcv1byte();
            int f_length1 = rcv1byte();
            unsigned int f_length = f_length1 * 256 + f_length2;
            ////  Serial.print("f_length:");
            ////  Serial.println(f_length);
            for (unsigned int lp1 = 0; lp1 < f_length; lp1++)
            {
                byte i_data = mz_read_send(file);
            }
            file.close();
            m_lop = m_lop + f_length;
            //kflg=3の時だけダミーヘッダ分読み飛ばし
            if (kflg == 3)
            {
                m_lop = m_lop + 128;
            }
            snd1byte(0x00);
        }
        else
        {
            //状態コード送信(ERROR)
            snd1byte(0xFF);
        }
    }
    else
    {
        //状態コード送信(FILE NOT FIND ERROR)
        snd1byte(0xF1);
    }
}

//BOOT処理(MZ-2000_SD専用)
RT_HANDLER void boot(void)
{
    //ファイルネーム取得
    mz_receive_name(m_name);
    ////  Serial.print("m_name:");
    ////  Serial.println(m_name);
    //ファイルが存在しなければERROR
    if (SD.exists(m_name) == true)
    {
        snd1byte(0x00);
        //ファイルオープン
        MZNativeFile file = mz_open_native(m_name, FILE_READ);
        if (true == file)
        {
            snd1byte(0x00);
            //ファイルサイズ送信
            unsigned long f_length = file.size();
            unsigned int f_len1 = f_length / 256;
            unsigned int f_len2 = f_length % 256;
            snd1byte(f_len2);
            snd1byte(f_len1);
            ////  Serial.println(f_length,HEX);
            ////  Serial.println(f_len2,HEX);
            ////  Serial.println(f_len1,HEX);

            //実データ送信
            for (unsigned long lp1 = 1; lp1 <= f_length; lp1++)
            {
                byte i_data = mz_read_send(file);
            }
        }
        else
        {
            //状態コード送信(ERROR)
            snd1byte(0xFF);
        }
    }
    else
    {
        //状態コード送信(FILE NOT FIND ERROR)
        snd1byte(0xF1);
    }
}

// Windows PC link: UART 115200 / 8N1, stop-and-wait framed protocol.
// s_data is shared with SD/MZ transfer; no second 256-byte packet buffer.
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
                s_data[i] = v;
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

static uint8_t pc_simple(uint8_t op)
{
    uint16_t got;
    return pc_call(op, 0, 0, 0, got);
}

static uint8_t pc_hello()
{
    uint8_t profile = 1;
    uint16_t got;
    uint8_t s = pc_call(0x01, &profile, 1, 1, got);
    return s == 0 && got == 1 && s_data[0] == 1 ? 0 : 0xf2;
}

static bool pc_read_block(uint16_t n)
{
    uint8_t req[2] = {(uint8_t)n, (uint8_t)(n >> 8)};
    uint16_t got;
    return pc_call(0x11, req, 2, 256, got) == 0 && got == n;
}

static bool pc_write_block(uint16_t n)
{
    uint16_t got;
    return pc_call(0x21, s_data, n, 0, got) == 0;
}

// PC backend for the UNCHANGED MZ-80K ROM protocol.
// UART framing functions pc_call/pc_hello/pc_read_block/pc_write_block precede this.
static uint32_t pc80WriteLeft = 0;
static bool pc80Writing = false;
static uint8_t pc80LastStatus = 0;

static uint16_t pc80_u16(const uint8_t *p)
{
    return p[0] | (uint16_t)p[1] << 8;
}

static uint32_t pc80_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void pc80_put32(uint8_t *p, uint32_t v)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        p[i] = (uint8_t)v;
        v >>= 8;
    }
}

static void pc80_halt()
{
    for (;;)
        rt_poll(false);
} // No fabricated data / false final ACK on I/O failure.

static void pc80_fault(uint8_t status)
{
    pc80LastStatus = status;
    pc80Writing = false;
    pc_simple(0x23);
    pc_simple(0x12);
    pc80_halt();
}

static uint8_t pc80_new()
{
    rt_hold_end();
    pc80Writing = false;
    pc80WriteLeft = 0;
    return pc_hello();
}

static bool pc80_ext(const char *p)
{
    return mz_has_mzt_extension(p, (uint8_t)strlen(p));
}

static bool pc80_suffix(char *p)
{
    uint8_t n = (uint8_t)strlen(p);
    if (!n || n > 32)
        return false;
    if (!pc80_ext(p))
        strcat(p, ".mzt");
    return true;
}

static bool pc80_get_name(char *dest, bool suffix)
{
    uint8_t n = 0;
    bool end = false, valid = true;
    memset(dest, 0, 40);
    for (uint8_t i = 0; i < 33; i++)
    {
        uint8_t v = rcv1byte();
        if (v == 0 || v == 13)
            end = true;
        if (!end)
        {
            if (n < 32 && v >= 32 && v < 127)
                dest[n++] = (char)v;
            else
                valid = false;
        }
    }
    while (n && dest[n - 1] == ' ')
        dest[--n] = 0;
    return valid && (suffix ? pc80_suffix(dest) : n != 0);
}

static bool pc80_name(char *dest)
{
    return pc80_get_name(dest, true);
}

static uint8_t pc80_open()
{
    uint16_t got;
    uint8_t s = pc_call(0x13, (const uint8_t *)m_name, (uint16_t)strlen(m_name), 4, got);
    if (s == 0 && got == 4)
        f_end = pc80_u32(s_data);
    else if (s == 0)
        s = 0xf2;
    return s;
}

static uint8_t pc80_seek(uint32_t offset)
{
    uint8_t req[4];
    pc80_put32(req, offset);
    uint16_t got;
    return pc_call(0x14, req, 4, 0, got);
}

static uint8_t pc80_classify()
{
    // Windows inspects the real filename, so aliases retain #n/#/$ semantics.
    kflg = 0;
    uint16_t got = 0;
    uint8_t status = pc_call(0x16, (const uint8_t *)m_name, (uint16_t)strlen(m_name), 37, got);
    if (status != 0)
        return status;
    if (got < 1 || s_data[0] > 3)
        return 0xf2;
    kflg = s_data[0];
    if (kflg == 1)
    {
        if (got < 2 || got > 37)
            return 0xf2;
        memcpy(k_name, s_data + 1, got - 1);
        k_name[got - 1] = 0;
    }
    else if (got != 1)
        return 0xf2;
    return 0;
}

static uint8_t pc80_header(uint32_t position)
{
    uint8_t s = pc80_new();
    if (s == 0)
        s = pc80_open();
    if (s != 0)
        return s;
    s = pc80_classify();
    if (s != 0)
        return s;
    if (position > f_end || f_end - position < 128)
        return 0xf2;
    s = pc80_seek(position);
    if (s != 0)
        return s;
    if (!pc_read_block(128))
        return 0xf2;
    uint16_t n = pc80_u16(s_data + 18);
    if (n == 0 || (uint32_t)n > f_end - position - 128)
        return 0xf2;
    m_lop = position + 128;
    return 0;
}

static uint8_t pc80_open_write(uint32_t size)
{
    uint8_t req[44];
    pc80_put32(req, size);
    uint8_t len = (uint8_t)strlen(m_name);
    memcpy(req + 4, m_name, len);
    uint16_t got;
    return pc_call(0x24, req, 4 + len, 0, got);
}

// Same four-phase nibble handshake. Only the FINAL low-nibble ACK is delayed
// until the PC accepts the last block / COMMIT. MZ holds CHK/data while waiting.
// Original rcv4bit(), rcv1byte() and snd1byte() are not changed.
static uint8_t pc80_final_byte()
{
    uint8_t high = rcv4bit();
    while (digitalRead(CHKPIN) != HIGH)
    {
        rt_poll(false);
    }
    uint8_t low =
        digitalRead(PA0PIN) + digitalRead(PA1PIN) * 2 + digitalRead(PA2PIN) * 4 + digitalRead(PA3PIN) * 8;
    return (uint8_t)((high << 4) | low);
}

static void pc80_final_ack()
{
    digitalWrite(FLGPIN, HIGH);
    while (digitalRead(CHKPIN) == HIGH)
    {
        rt_poll(false);
    }
    digitalWrite(FLGPIN, LOW);
}

static void pc80_receive_data(uint16_t length, bool commit)
{
    uint16_t left = length;
    while (left)
    {
        uint16_t n = left > 256 ? 256 : left;
        for (uint16_t i = 0; i < n; i++)
            s_data[i] = (left == n && i == n - 1) ? pc80_final_byte() : rcv1byte();
        if (!pc_write_block(n))
            pc80_fault(0xf2);
        if (left == n)
        {
            if (commit)
            {
                uint8_t s = pc_simple(0x22);
                if (s != 0)
                    pc80_fault(s);
            }
            pc80_final_ack();
        }
        left -= n;
    }
}

static void pc80_send_data(uint16_t length)
{
    uint16_t left = length;
    while (left)
    {
        uint16_t n = left > 256 ? 256 : left;
        if (!pc_read_block(n))
            pc80_fault(0xf2);
        for (uint16_t i = 0; i < n; i++)
            snd1byte(s_data[i]);
        left -= n;
    }
}

static void pc80_advance(uint16_t length)
{
    m_lop += length;
    if (kflg == 3)
        m_lop = f_end - m_lop >= 128 ? m_lop + 128 : f_end;
}

// Existing 80H: 33-byte filename, 17-byte program name, start/end/exec LE.
static void pc80_save()
{
    uint8_t status = pc80_new();
    bool valid = pc80_name(m_name);
    memset(s_data, 0, 128);
    s_data[0] = 1;
    for (uint8_t i = 0; i < 17; i++)
        s_data[i + 1] = rcv1byte();
    s_data[16] = 13;
    s_data[17] = 0;
    uint8_t a[6];
    for (uint8_t i = 0; i < 6; i++)
        a[i] = rcv1byte();
    uint16_t start = pc80_u16(a), end = pc80_u16(a + 2);
    uint32_t size = end >= start ? (uint32_t)end - start + 1 : 0;
    if (!valid || size == 0 || size > 65535)
        status = 0xf2;
    s_data[18] = (uint8_t)size;
    s_data[19] = (uint8_t)(size >> 8);
    s_data[20] = a[0];
    s_data[21] = a[1];
    s_data[22] = a[4];
    s_data[23] = a[5];
    if (status == 0)
        status = pc80_open_write(128 + size);
    if (status == 0 && !pc_write_block(128))
        status = 0xf2;
    if (status != 0)
    {
        pc_simple(0x23);
        snd1byte(status);
        pc80LastStatus = status;
        return;
    }
    snd1byte(0);
    pc80_receive_data((uint16_t)size, true);
    pc80LastStatus = 0;
}

// Existing 81H: status, 17-byte program name, start/length/exec LE, body.
static void pc80_load()
{
    bool valid = pc80_name(m_name);
    uint8_t status = valid ? pc80_header(0) : 0xf2;
    if (status != 0)
    {
        pc_simple(0x12);
        snd1byte(status);
        pc80LastStatus = status;
        return;
    }
    uint16_t size = pc80_u16(s_data + 18);
    snd1byte(0);
    for (uint8_t i = 1; i <= 17; i++)
        snd1byte(s_data[i]);
    snd1byte(s_data[20]);
    snd1byte(s_data[21]);
    snd1byte(s_data[18]);
    snd1byte(s_data[19]);
    snd1byte(s_data[22]);
    snd1byte(s_data[23]);
    pc80_send_data(size);
    pc80_advance(size);
    pc_simple(0x12);
    pc80LastStatus = 0;
}

static void pc80_directory()
{
    for (uint8_t i = 0; i < 33; i++)
        c_name[i] = (char)rcv1byte();
    c_name[33] = 0;
    if (c_name[0] == 0)
        c_name[0] = 0;
    else
    {
        memmove(c_name, c_name + 1, 32);
        c_name[32] = 0;
        for (uint8_t i = 0; i < 32; i++)
            if (c_name[i] == 13)
                c_name[i] = 0;
    }
    uint8_t status = pc80_new();
    uint16_t got = 0;
    if (status == 0)
        status = pc_call(0x30, (const uint8_t *)c_name, (uint16_t)strlen(c_name), 2, got);
    if (status != 0 || got != 2)
    {
        snd1byte(0xff);
        snd1byte(status ? status : 0xf2);
        return;
    }
    uint16_t count = pc80_u16(s_data), page = 0;
    for (;;)
    {
        uint16_t first = page * 20;
        uint8_t shown = 0;
        for (uint8_t slot = 0; slot < 20 && (uint32_t)first + slot < count; slot++)
        {
            uint16_t index = first + slot;
            uint8_t req[2] = {(uint8_t)index, (uint8_t)(index >> 8)};
            status = pc_call(0x31, req, 2, 36, got);
            if (status != 0 || got == 0 || got > 36)
                break;
            // Omit .MZT so all 32 basename characters survive the ROM's 32-byte field.
            uint16_t displayLength = got;
            if (got >= 4 && s_data[got - 4] == '.' && upper((char)s_data[got - 3]) == 'M' &&
                upper((char)s_data[got - 2]) == 'Z' && upper((char)s_data[got - 1]) == 'T')
                displayLength -= 4;
            for (uint16_t j = 0; j < displayLength; j++)
                snd1byte((uint8_t)upper((char)s_data[j]));
            snd1byte(13);
            snd1byte(0);
            shown++;
        }
        if (status != 0 || shown < 20 && page == 0)
            break;
        snd1byte(0xfe);
        uint8_t key = rcv1byte();
        if (key == 0x42)
        {
            if (page)
                page--;
            continue;
        }
        if (key != 0 || (uint32_t)(page + 1) * 20 >= count)
            break;
        page++;
    }
    pc_simple(0x32);
    snd1byte(0xff);
    snd1byte(status);
}

static void pc80_whead()
{
    uint8_t status = pc80_new();
    for (uint16_t i = 0; i < 128; i++)
        s_data[i] = rcv1byte();
    memset(m_name, 0, 40);
    uint8_t n = 0;
    for (uint8_t i = 1; i <= 17 && s_data[i] != 13 && s_data[i] != 0; i++)
        m_name[n++] = (char)s_data[i];
    while (n && m_name[n - 1] == ' ')
        m_name[--n] = 0;
    if (!pc80_suffix(m_name))
        status = 0xf2;
    s_data[16] = 13;
    uint16_t size = pc80_u16(s_data + 18);
    if (size == 0)
        status = 0xf2;
    if (status == 0)
        status = pc80_open_write(128UL + size);
    if (status == 0 && !pc_write_block(128))
        status = 0xf2;
    if (status == 0)
    {
        pc80Writing = true;
        pc80WriteLeft = size;
        rt_hold_begin(size);
    }
    else
        pc_simple(0x23);
    pc80LastStatus = status;
    snd1byte(status);
}

static void pc80_wdata()
{
    uint8_t lo = rcv1byte(), hi = rcv1byte();
    uint16_t size = lo | (uint16_t)hi << 8;
    if (!pc80Writing || !size || size > pc80WriteLeft)
    {
        snd1byte(0xf2);
        pc80LastStatus = 0xf2;
        return;
    }
    snd1byte(0);
    pc80_receive_data(size, size == pc80WriteLeft);
    pc80WriteLeft -= size;
    if (!pc80WriteLeft)
    {
        pc80Writing = false;
        rt_hold_end();
    }
    pc80LastStatus = 0;
}

static void pc80_lhead()
{
    bool valid = pc80_name(m_name);
    uint8_t status = valid ? pc80_header(0) : 0xf2;
    if (status != 0)
    {
        pc_simple(0x12);
        snd1byte(status);
        pc80LastStatus = status;
        return;
    }
    snd1byte(0);
    snd1byte(0);
    for (uint16_t i = 0; i < 128; i++)
        snd1byte(s_data[i]);
    pc_simple(0x12);
    snd1byte(0);
    pc80LastStatus = 0;
}

static void pc80_ldata()
{
    uint8_t status = pc80_new();
    if (status == 0)
        status = pc80_open();
    if (status == 0 && m_lop >= f_end)
        status = 0xf2;
    if (status == 0)
        status = pc80_seek(m_lop);
    if (status != 0)
    {
        pc_simple(0x12);
        snd1byte(status);
        pc80LastStatus = status;
        return;
    }
    snd1byte(0);
    snd1byte(0);
    uint8_t lo = rcv1byte(), hi = rcv1byte();
    uint16_t size = lo | (uint16_t)hi << 8;
    if (!size || size > f_end - m_lop)
        pc80_fault(0xf2);
    pc80_send_data(size);
    pc80_advance(size);
    status = pc_simple(0x12);
    snd1byte(status);
    pc80LastStatus = status;
}

static void pc80_continue()
{
    uint32_t position = 0;
    if (kflg == 1)
        strcpy(m_name, k_name);
    else if (kflg == 2 && m_lop < f_end && f_end - m_lop >= 128)
        position = m_lop;
    else
    {
        snd1byte(0xff);
        return;
    }
    uint8_t status = pc80_header(position);
    snd1byte(0);
    if (status != 0)
    {
        snd1byte(status);
        pc_simple(0x12);
        return;
    }
    snd1byte(0);
    snd1byte(0);
    for (uint16_t i = 0; i < 128; i++)
        snd1byte(s_data[i]);
    pc_simple(0x12);
    snd1byte(0);
}

static uint8_t pc80_stat(const char *name)
{
    uint16_t got;
    return pc_call(0x15, (const uint8_t *)name, (uint16_t)strlen(name), 4, got);
}

// Existing 95H: raw filename (NO .mzt append), exists/open statuses, length LE,
// then the entire raw file including any header. 0000 length means 65536 bytes.
static void pc80_boot()
{
    bool valid = pc80_get_name(m_name, false);
    uint8_t status = pc80_new();
    if (!valid)
        status = 0xf2;
    if (status == 0)
        status = pc80_open();
    if (status == 0 && (!f_end || f_end > 65536UL))
        status = 0xf5;
    if (status != 0)
    {
        pc_simple(0x12);
        snd1byte(status);
        pc80LastStatus = status;
        return;
    }
    kflg = 0;
    snd1byte(0);
    snd1byte(0);
    snd1byte((uint8_t)f_end);
    snd1byte((uint8_t)(f_end >> 8));
    uint32_t left = f_end;
    while (left)
    {
        uint16_t n = left > 256 ? 256 : (uint16_t)left;
        if (!pc_read_block(n))
            pc80_fault(0xf2);
        for (uint16_t i = 0; i < n; i++)
            snd1byte(s_data[i]);
        left -= n;
    }
    pc_simple(0x12);
    pc80LastStatus = 0;
}

static bool mz_command_supported(uint8_t command)
{
    return ((command >= 0x80 && command <= 0x81) || command == 0x83) || (command >= 0x91 && command <= 0x96);
}

static RT_HANDLER void pc80_dispatch(uint8_t cmd)
{
    if (!mz_command_supported(cmd))
    {
        snd1byte(0xf4);
        return;
    }
    snd1byte(0);
    switch (cmd)
    {
    case 0x80:
        pc80_save();
        break;
    case 0x81:
        pc80_load();
        break;
    case 0x83:
        pc80_directory();
        break;

    case 0x91:
        pc80_whead();
        break;
    case 0x92:
        pc80_wdata();
        break;
    case 0x93:
        pc80_lhead();
        break;
    case 0x94:
        pc80_ldata();
        break;
    case 0x96:
        pc80_continue();
        break;

    case 0x95:
        pc80_boot();
        break;
    }
}

RT_HANDLER void loop()
{
    mz_clear_outputs();
    //コマンド取得待ち
    ////  Serial.print("cmd:");
    rt_wait_command();
    byte cmd = rcv1byte();
    rt_prepare(cmd);
    ////  Serial.println(cmd,HEX);
    if (rt_backend)
    {
        pc80_dispatch(cmd);
        rt_finish();
        return;
    }
    if (eflg == false)
    {
        // All supported commands retain their original initial OK reply.
        if (mz_command_supported(cmd))
            snd1byte(0x00);
        switch (cmd)
        {
            //80hでSDカードにsave
        case 0x80:
            ////  Serial.println("SAVE START");
            f_save();
            break;
            //81hでSDカードからload
        case 0x81:
            ////  Serial.println("LOAD START");
            f_load();
            break;

            //83hでファイルリスト出力
        case 0x83:
            ////  Serial.println("FILE LIST START");
            sdinit();
            dirlist();
            break;

        case 0x91:
            //91hで0436H MONITOR ライト インフォメーション代替処理
            ////  Serial.println("0436H START");
            mon_whead();
            break;
            //92hで0475H MONITOR ライト データ代替処理
        case 0x92:
            ////  Serial.println("0475H START");
            mon_wdata();
            break;
            //93hで04D8H MONITOR リード インフォメーション代替処理
        case 0x93:
            ////  Serial.println("04D8H START");
            mon_lhead();
            break;
            //94hで04F8H MONITOR リード データ代替処理
        case 0x94:
            ////  Serial.println("04F8H START");
            mon_ldata();
            break;
            //95hでBOOT LOAD(MZ-2000_SD専用)
        case 0x95:
            ////  Serial.println("BOOT LOAD START");
            boot();
            break;
            //96hで継続ロードCHECK
        case 0x96:
            ////  Serial.println("KFLG CHECK");
            kflgchk();
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
