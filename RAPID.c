#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "usb_descriptors.h"

#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};
static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

#define BUTTON_PIN 22
#define CHUNK_SIZE 128
#define CFG_TUD_CDC 1
#define MAX_LINE_LENGTH 128

// Create an array mapping ASCII characters to HID key constants
const uint8_t ascii_to_hid[128] = {
    [ 'a' ] = HID_KEY_A,
    [ 'b' ] = HID_KEY_B,
    [ 'c' ] = HID_KEY_C,
    [ 'd' ] = HID_KEY_D,
    [ 'e' ] = HID_KEY_E,
    [ 'f' ] = HID_KEY_F,
    [ 'g' ] = HID_KEY_G,
    [ 'h' ] = HID_KEY_H,
    [ 'i' ] = HID_KEY_I,
    [ 'j' ] = HID_KEY_J,
    [ 'k' ] = HID_KEY_K,
    [ 'l' ] = HID_KEY_L,
    [ 'm' ] = HID_KEY_M,
    [ 'n' ] = HID_KEY_N,
    [ 'o' ] = HID_KEY_O,
    [ 'p' ] = HID_KEY_P,
    [ 'q' ] = HID_KEY_Q,
    [ 'r' ] = HID_KEY_R,
    [ 's' ] = HID_KEY_S,
    [ 't' ] = HID_KEY_T,
    [ 'u' ] = HID_KEY_U,
    [ 'v' ] = HID_KEY_V,
    [ 'w' ] = HID_KEY_W,
    [ 'x' ] = HID_KEY_X,
    [ 'y' ] = HID_KEY_Y,
    [ 'z' ] = HID_KEY_Z,
    [ '0' ] = HID_KEY_0,
    [ '1' ] = HID_KEY_1,
    [ '2' ] = HID_KEY_2,
    [ '3' ] = HID_KEY_3,
    [ '4' ] = HID_KEY_4,
    [ '5' ] = HID_KEY_5,
    [ '6' ] = HID_KEY_6,
    [ '7' ] = HID_KEY_7,
    [ '8' ] = HID_KEY_8,
    [ '9' ] = HID_KEY_9,
    [ ' ' ] = HID_KEY_SPACE,
    [ '\n'] = HID_KEY_ENTER,
    [ '\t'] = HID_KEY_TAB,
    [ '-' ] = HID_KEY_MINUS,
    [ '=' ] = HID_KEY_EQUAL,
    [ '[' ] = HID_KEY_BRACKET_LEFT,
    [ ']' ] = HID_KEY_BRACKET_RIGHT,
    [ '\\'] = HID_KEY_BACKSLASH,
    [ ';' ] = HID_KEY_SEMICOLON,
    [ '\'' ] = HID_KEY_APOSTROPHE,
    [ '`' ] = HID_KEY_GRAVE,
    [ ',' ] = HID_KEY_COMMA,
    [ '.' ] = HID_KEY_PERIOD,
    [ '/' ] = HID_KEY_SLASH,
};

// Create an array mapping ASCII characters to HID key constants for shifted keys
const uint8_t ascii_to_hid_shifted[128] = {
    [ 'A' ] = HID_KEY_A,
    [ 'B' ] = HID_KEY_B,
    [ 'C' ] = HID_KEY_C,
    [ 'D' ] = HID_KEY_D,
    [ 'E' ] = HID_KEY_E,
    [ 'F' ] = HID_KEY_F,
    [ 'G' ] = HID_KEY_G,
    [ 'H' ] = HID_KEY_H,
    [ 'I' ] = HID_KEY_I,
    [ 'J' ] = HID_KEY_J,
    [ 'K' ] = HID_KEY_K,
    [ 'L' ] = HID_KEY_L,
    [ 'M' ] = HID_KEY_M,
    [ 'N' ] = HID_KEY_N,
    [ 'O' ] = HID_KEY_O,
    [ 'P' ] = HID_KEY_P,
    [ 'Q' ] = HID_KEY_Q,
    [ 'R' ] = HID_KEY_R,
    [ 'S' ] = HID_KEY_S,
    [ 'T' ] = HID_KEY_T,
    [ 'U' ] = HID_KEY_U,
    [ 'V' ] = HID_KEY_V,
    [ 'W' ] = HID_KEY_W,
    [ 'X' ] = HID_KEY_X,
    [ 'Y' ] = HID_KEY_Y,
    [ 'Z' ] = HID_KEY_Z,
    [ '!' ] = HID_KEY_1,   // Shift + 1 -> !
    [ '@' ] = HID_KEY_2,   // Shift + 2 -> @
    [ '#' ] = HID_KEY_3,   // Shift + 3 -> #
    [ '$' ] = HID_KEY_4,   // Shift + 4 -> $
    [ '%' ] = HID_KEY_5,   // Shift + 5 -> %
    [ '^' ] = HID_KEY_6,   // Shift + 6 -> ^
    [ '&' ] = HID_KEY_7,   // Shift + 7 -> &
    [ '*' ] = HID_KEY_8,   // Shift + 8 -> *
    [ '(' ] = HID_KEY_9,   // Shift + 9 -> (
    [ ')' ] = HID_KEY_0,   // Shift + 0 -> )
    [ '_' ] = HID_KEY_MINUS, // Shift + - -> _
    [ '+' ] = HID_KEY_EQUAL, // Shift + = -> +
    [ '{' ] = HID_KEY_BRACKET_LEFT, // Shift + [ -> {
    [ '}' ] = HID_KEY_BRACKET_RIGHT, // Shift + ] -> }
    [ '|' ] = HID_KEY_BACKSLASH, // Shift + \ -> |
    [ ':' ] = HID_KEY_SEMICOLON, // Shift + ; -> :
    [ '"' ] = HID_KEY_APOSTROPHE, // Shift + ' -> "
    [ '~' ] = HID_KEY_GRAVE, // Shift + ` -> ~
    [ '<' ] = HID_KEY_COMMA, // Shift + , -> <
    [ '>' ] = HID_KEY_PERIOD, // Shift + . -> >
    [ '?' ] = HID_KEY_SLASH, // Shift + / -> ?
};

uint8_t const shift = 0x02; // Left shift
static void open_cmd();
static void send_hid_report(uint8_t keycode_value, uint8_t shift, uint8_t modifier);
static bool button_is_pressed(int gpio);

static void send_string(const char *str);
bool needs_shift(char key);
static void send_string_se(const char *str);
void cdc_task(void);
void my_usb_log(const char* msg);
bool write_to_sd_file(const char* filename, const char* message);
bool read_sd_file(const char* filename);
void usb_sd_logger_once(const char* filename);

static char input_buffer[MAX_LINE_LENGTH] = {0};
static int input_pos = 0;


int main(void)
{
    board_init();

    // Initialize GPIO pin for button (GP22)
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN); // Set GPIO 22 as input
    gpio_pull_up(BUTTON_PIN);

    // Init TinyUSB driver (HID + CDC)
    tud_init(BOARD_TUD_RHPORT);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    while (1) {
        tud_task();                // Keep USB alive
        
        if (button_is_pressed(BUTTON_PIN)) {
          // Open powershell
          my_usb_log("IRQ Button Pressed\r\n");
          open_cmd(); // WIN + R to open r
          sleep_ms(1000);
          send_string("powershell.exe\n"); // Open Powershell
          sleep_ms(1000);
          // Download and inject script
          send_string("echo 'Hello World'\n");
          send_string("echo 'Writing to sd card'\n");
          /* Testing area for SD card*/
          if (write_to_sd_file("log.txt", "Hello from my_usb_log + SD writer")) {
            my_usb_log("Write successful!\r\n");
            send_string("echo 'Writing successful!'\n");
          } else {
            my_usb_log("Write failed!\r\n");
            send_string("echo 'Write failed!'\n");
          }

          sleep_ms(1000);
          my_usb_log("Reading log.txt from SD card...\r\n");
          read_sd_file("log.txt");

          /* End of SD Card Test*/
          my_usb_log("Main code executed.\r\n");
        }
        usb_sd_logger_once("input.txt");
    }
}

// SD Card
//--------------------------------------------------------------------+
void usb_sd_logger_once(const char* filename) {
    tud_task();  // must be called often in main loop

    if (!tud_cdc_connected() || !tud_cdc_available()) {
        return; // nothing to do
    }

    char c;
    if (tud_cdc_read(&c, 1)) {
    if (c == '\b' || c == 127) {  // Handle Backspace or DEL
        if (input_pos > 0) {
            input_pos--;
            tud_cdc_write_str("\b \b");  // Erase char on terminal
        }
    }
    else if (c == '\r' || c == '\n') {
        input_buffer[input_pos] = '\0';
        if (input_pos > 0) {
            if (write_to_sd_file(filename, input_buffer)) {
                my_usb_log("\r\n[✓] Saved to file\r\n");
            } else {
                my_usb_log("\r\n[✗] SD write failed\r\n");
            }
            input_pos = 0;
        } else {
            my_usb_log("\r\n[!] Empty line\r\n");
        }
    }
    else if (input_pos < MAX_LINE_LENGTH - 1) {
        input_buffer[input_pos++] = c;
        tud_cdc_write_char(c);  // Echo the typed char
    }
    tud_cdc_write_flush();
}

}

bool write_to_sd_file(const char* filename, const char* message) {
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        char buf[100];
        snprintf(buf, sizeof(buf), "f_mount error: %s (%d)\r\n", FRESULT_str(fr), fr);
        my_usb_log(buf);
        return false;
    }

    FIL fil;
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK && fr != FR_EXIST) {
        char buf[100];
        snprintf(buf, sizeof(buf), "f_open(%s) error: %s (%d)\r\n", filename, FRESULT_str(fr), fr);
        my_usb_log(buf);
        f_unmount("");
        return false;
    }

    if (f_printf(&fil, "%s\n", message) < 0) {
        my_usb_log("f_printf failed\r\n");
        f_close(&fil);
        f_unmount("");
        return false;
    }

    fr = f_close(&fil);
    if (fr != FR_OK) {
        char buf[100];
        snprintf(buf, sizeof(buf), "f_close error: %s (%d)\r\n", FRESULT_str(fr), fr);
        my_usb_log(buf);
        f_unmount("");
        return false;
    }

    f_unmount("");
    return true;
}

bool read_sd_file(const char* filename) {
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr != FR_OK) {
        char buf[100];
        snprintf(buf, sizeof(buf), "f_mount error: %s (%d)\r\n", FRESULT_str(fr), fr);
        my_usb_log(buf);
        return false;
    }

    FIL fil;
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        char buf[100];
        snprintf(buf, sizeof(buf), "f_open(%s) error: %s (%d)\r\n", filename, FRESULT_str(fr), fr);
        my_usb_log(buf);
        f_unmount("");
        return false;
    }

    my_usb_log("Contents of script.txt:\r\n");

    char buffer[128];
    while (f_gets(buffer, sizeof(buffer), &fil)) {
        my_usb_log(buffer);  // already includes newline if present
    }

    fr = f_close(&fil);
    if (fr != FR_OK) {
        char buf[100];
        snprintf(buf, sizeof(buf), "f_close error: %s (%d)\r\n", FRESULT_str(fr), fr);
        my_usb_log(buf);
        f_unmount("");
        return false;
    }

    f_unmount("");
    my_usb_log("\r\nFinished reading file.\r\n");
    return true;
}

// CDC
//--------------------------------------------------------------------+
// Instead of cdc_log, use:
void my_usb_log(const char* msg) {
    if (tud_cdc_connected()) {
        tud_cdc_write_str(msg);
        tud_cdc_write_flush();
    }
}

// BUTTON PRESSED
//--------------------------------------------------------------------+
static bool button_is_pressed(int gpio)
{
    // Replace this with your actual GPIO read logic
    return !gpio_get(gpio); // Assuming active high for the button
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+
// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+
/*
 * Local handler code to send a key press to the host PC
 */
static void send_hid_report(uint8_t keycode_value, uint8_t shift, uint8_t modifier)
{
    uint8_t keycode[6] = { 0 };
    keycode[0] = modifier;
    keycode [2] = keycode_value;

    tud_hid_keyboard_report(REPORT_ID_KEYBOARD, shift, keycode);

    uint8_t zero_keycode[6] = { 0 };
    tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, zero_keycode);
    board_delay(10);
}

static void open_cmd(){
  tud_task();
  send_hid_report(HID_KEY_R, 0x08, 0);
  send_hid_report(HID_KEY_NONE, 0, 0);
}

/*
 * Function to send a string over HID keyboard
 */
static void send_string(const char *str)
{
    const char *start = str;

    while (!tud_hid_ready()) {
        tud_task();
    }

    str = start;
    while (*str) {
        tud_task();
        uint8_t hid_code = (*str < 128) ? ascii_to_hid[(uint8_t)*str] : HID_KEY_NONE;
        uint8_t shift = 0; // No shift by default

        // Check if the character requires Shift
        if (needs_shift(*str)) {
            hid_code = (*str < 128) ? ascii_to_hid_shifted[(uint8_t)*str] : HID_KEY_NONE;
            shift = 0x02; // Set Left Shift modifier
        }

        if (hid_code != HID_KEY_NONE) {
            send_hid_report(hid_code, shift, 0);
        }

        // Send a release report
        send_hid_report(HID_KEY_NONE, 0, 0);
        for (volatile int i = 0; i < 150000; i++); // Optional delay
        str++;
    }
}

bool needs_shift(char key) {
    // Check if the key needs the Shift modifier
    // Add any other keys that require Shift here
    return (key >= 'A' && key <= 'Z') || 
           (key == '!' || key == '@' || key == '#' || key == '$' || 
            key == '%' || key == '^' || key == '&' || 
            key == '*' || key == '(' || key == ')' || 
            key == '_' || key == '+' || key == '{' || 
            key == '}' || key == '|' || key == ':' || 
            key == '"' || key == '<' || key == '>' || 
            key == '?' || key == '~');
}

/*
 * Function to send a string over HID keyboard (with shift+enter)
 */
static void send_string_se(const char *str)
{
    const char *start = str;

    while (!tud_hid_ready()) {
        tud_task();
    }

    str = start;
    while (*str) {
        tud_task();
        uint8_t hid_code = (*str < 128) ? ascii_to_hid[(uint8_t)*str] : HID_KEY_NONE;
        uint8_t shift = 0; // No shift by default

        // Check if the character requires Shift
        if (needs_shift(*str)) {
            hid_code = (*str < 128) ? ascii_to_hid_shifted[(uint8_t)*str] : HID_KEY_NONE;
            shift = 0x02; // Set Left Shift modifier
        }
        else if (*str == '\n'){
            hid_code = HID_KEY_ENTER;
            shift = 0x02; // Set Left Shift modifier
        }

        if (hid_code != HID_KEY_NONE) {
            send_hid_report(hid_code, shift, 0);
        }

        // Send a release report
        send_hid_report(HID_KEY_NONE, 0, 0);
        for (volatile int i = 0; i < 150000; i++); // Optional delay
        str++;
    }
}

//--------------------------------------------------------------------+
// USB HID Callbacks
//--------------------------------------------------------------------+
// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD)
    {
      // bufsize should be (at least) 1
      if ( bufsize < 1 ) return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
      {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        board_led_write(true);
      }else
      {
        // Caplocks Off: back to normal blink
        board_led_write(false);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }
}
