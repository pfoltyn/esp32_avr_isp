#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

// Pin definitions
#define MOSI_PIN 10
#define MISO_PIN 11
#define SCK_PIN 12
#define RESET_PIN 13

// SPI configuration
#define SPI_HOST SPI2_HOST
#define SPI_CLOCK_HZ 100000 // 100 kHz for ATmega default clock

// STK500v1 constants
#define STK_OK 0x10
#define STK_FAILED 0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC 0x14
#define STK_NOSYNC 0x15
#define CRC_EOP 0x20

#define PAGE_SIZE_BYTES 128
#define PAGE_SIZE_WORDS (PAGE_SIZE_BYTES / 2) // 64 for ATmega328P

// Programmer state
static uint8_t pmode = 0; // Programming mode
static uint16_t addr; // Current address
static uint8_t buff[256]; // Buffer for page programming
static uint8_t error = 0;
static const char *TAG = "STK500v1";

// SPI handle
static spi_device_handle_t spi;

// Helper: get a single byte (blocking)
uint8_t getch(void) {
    uint8_t ch;
    while (usb_serial_jtag_read_bytes(&ch, 1, 1000 / portTICK_PERIOD_MS) != 1);
    return ch;
}

// Helper: start programming mode
void start_pmode(void) {
    spi_transaction_t t = {
        .length = 8 * 4,
        .tx_buffer = (uint8_t[]){0xAC, 0x53, 0x00, 0x00},
        .rxlength = 8 * 4,
        .rx_buffer = buff
    };
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    if (buff[2] != 0x53) {
        ESP_LOGE(TAG, "Failed to enter programming mode");
        error++;
        pmode = 0;
        return;
    }
    pmode = 1;
}

// Helper: single-byte SPI transfer
uint8_t spi_transfer(uint8_t data) {
    uint8_t rx_data;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .rxlength = 8,
        .rx_buffer = &rx_data
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
    return rx_data;
}

// Polls AVR's RDY/BSY bit after page write
void wait_for_rdy(void) {
    uint8_t status;
    do {
        uint8_t tx[4] = {0xF0, 0x00, 0x00, 0x00};
        uint8_t rx[4];
        spi_transaction_t t = {
            .length = 32,
            .tx_buffer = tx,
            .rxlength = 32,
            .rx_buffer = rx
        };
        ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
        status = rx[3];
    } while (!(status & 0x80));
}

// Helper: get_parameter (STK500v1)
void get_parameter(void) {
    uint8_t param = getch();
    uint8_t ch;
    if (usb_serial_jtag_read_bytes(&ch, 1, 1000 / portTICK_PERIOD_MS) != 1 || ch != CRC_EOP) {
        error++;
        ESP_LOGW(TAG, "No EOP for get_parameter");
        usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
        return;
    }
    uint8_t value = 0x00;
    switch(param) {
        case 0x80: value = 0x42; break; // Hardware version
        case 0x81: value = 0x02; break; // Software major
        case 0x82: value = 0x0a; break; // Software minor
        case 0x98: value = 0x03; break; // Unknown, usually 0x03
        case 0x84: value = 0x64; break; // Vtarget (100 = 5.0V)
        case 0x85: value = 0x64; break; // Varef (100 = 5.0V)
        case 0x86: value = 0x19; break; // Oscillator (25 = 8 MHz)
        case 0x87: value = 0x19; break; // SCK duration (25 = 8 MHz)
        case 0x89: value = 0x19; break; // SCK duration (25 = 8 MHz)
        default: value = 0x01; break;
    }
    uint8_t reply[3] = {STK_INSYNC, value, STK_OK};
    usb_serial_jtag_write_bytes(reply, 3, 1000 / portTICK_PERIOD_MS);
}

void app_main(void) {
    // Initialize GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(RESET_PIN, 1)); // Keep AVR out of reset

    // Initialize SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLOCK_HZ,
        .mode = 0, // SPI mode 0
        .spics_io_num = -1, // No CS pin
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));

    // Initialize USB Serial/JTAG
    usb_serial_jtag_driver_config_t usb_config = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB Serial/JTAG: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "USB Serial/JTAG initialized");
    ESP_LOGI(TAG, "STK500v1 programmer started");

    // Main loop
    while (1) {
        uint8_t ch;
        if (usb_serial_jtag_read_bytes(&ch, 1, 0) == 1) {
            switch (ch) {
            case '0': // Sign-on
                if (getch() == CRC_EOP)
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                else
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            case '1': // Check programmer
                if (getch() == CRC_EOP)
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                else
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            case 'A': // Get parameter
                get_parameter();
                break;
            case 'B': { // Set parameters (20 bytes + CRC_EOP)
                uint8_t bytes[21];
                int got = 0;
                while (got < 21) got += usb_serial_jtag_read_bytes(bytes + got, 21 - got, 1000 / portTICK_PERIOD_MS);
                if (bytes[20] == CRC_EOP)
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                else
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            }
            case 'E': { // Set Extended Parameters (5 bytes + CRC_EOP)
                uint8_t bytes[6];
                int got = 0;
                while (got < 6) got += usb_serial_jtag_read_bytes(bytes + got, 6 - got, 1000 / portTICK_PERIOD_MS);
                if (bytes[5] == CRC_EOP)
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                else
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                pmode = 0;
                gpio_set_level(RESET_PIN, 1);
                break;
            }
            case 'P': // Enter programming mode (no params)
                if (getch() == CRC_EOP) {
                    start_pmode();
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                } else {
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                }
                break;
            case 'U': { // Set address (2 bytes + CRC_EOP)
                uint8_t bytes[3];
                int got = 0;
                while (got < 3) got += usb_serial_jtag_read_bytes(bytes + got, 3 - got, 1000 / portTICK_PERIOD_MS);
                addr = bytes[0] | (bytes[1] << 8);
                if (bytes[2] == CRC_EOP)
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                else
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            }
			case 'd': { // Write flash (paged)
				// --- 1. Read header: len_high, len_low, memtype
				uint8_t header[3];
				int got = 0;
				while (got < 3) got += usb_serial_jtag_read_bytes(header + got, 3 - got, 1000 / portTICK_PERIOD_MS);
				uint16_t len = (header[0] << 8) | header[1]; // Big-endian
				uint8_t memtype = header[2];

				// --- 2. Read data payload
				if (len > sizeof(buff)) len = sizeof(buff); // Prevent overflow
				got = 0;
				while (got < len) got += usb_serial_jtag_read_bytes(buff + got, len - got, 1000 / portTICK_PERIOD_MS);

				// --- 3. Read CRC_EOP
				uint8_t crc;
				if (usb_serial_jtag_read_bytes(&crc, 1, 1000 / portTICK_PERIOD_MS) != 1 || crc != CRC_EOP) {
					usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
					break;
				}

				// --- 4. Ensure programming mode
				if (!pmode) {
					start_pmode();
				}

				// --- 5. Program flash using proper SPI sequence
                for (int w = 0; w < len / 2; w++) {
                    uint16_t word_addr = addr + w;
                    uint8_t tx_low[4] = {0x40, (word_addr >> 8), (word_addr & 0xFF), buff[w * 2]};
                    spi_transaction_t t_low = { .length = 32, .tx_buffer = tx_low };
                    ESP_ERROR_CHECK(spi_device_transmit(spi, &t_low));

                    uint8_t tx_high[4] = {0x48, (word_addr >> 8), (word_addr & 0xFF), buff[w * 2 + 1]};
                    spi_transaction_t t_high = { .length = 32, .tx_buffer = tx_high };
                    ESP_ERROR_CHECK(spi_device_transmit(spi, &t_high));
                }

                // Page write (use page-aligned word address)
                uint16_t page_base = addr & ~(PAGE_SIZE_WORDS - 1);
                uint8_t tx_write[4] = {0x4C, (page_base >> 8), (page_base & 0xFF), 0x00};
                spi_transaction_t t_write = { .length = 32, .tx_buffer = tx_write };
                ESP_ERROR_CHECK(spi_device_transmit(spi, &t_write));
				vTaskDelay(10 / portTICK_PERIOD_MS); // Add 1 ms delay

				// --- 7. Poll for write completion (RDY/BSY)
				wait_for_rdy();

				// --- 8. Respond to avrdude
				usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
				break;
			}
			case 'c': // Chip erase
				if (getch() == CRC_EOP) {
					uint8_t tx[4] = {0xAC, 0x80, 0x00, 0x00};
					spi_transaction_t t = {
						.length = 32,
						.tx_buffer = tx
					};
					ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
					vTaskDelay(20 / portTICK_PERIOD_MS); // Wait for erase
					usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
				} else {
					usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
				}
				break;
            case 't': { // Read flash (paged)
                uint8_t bytes[4];
                int got = 0;
                while (got < 4) got += usb_serial_jtag_read_bytes(bytes + got, 4 - got, 1000 / portTICK_PERIOD_MS);
                uint16_t len = bytes[1] | (bytes[0] << 8);
                uint8_t memtype = bytes[2];
                uint8_t crc = bytes[3];
                if (crc != CRC_EOP) {
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                    break;
                }
                usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                for (uint16_t i = 0; i < len; i++) {
                    uint16_t word_addr = addr + (i / 2);
                    uint8_t tx_data[4];
                    uint8_t rx_data[4];
                    tx_data[0] = (i % 2 == 0) ? 0x20 : 0x28;
                    tx_data[1] = (word_addr >> 8) & 0xFF;
                    tx_data[2] = word_addr & 0xFF;
                    tx_data[3] = 0x00;
                    spi_transaction_t t = { .length = 8 * 4, .tx_buffer = tx_data, .rxlength = 8 * 4, .rx_buffer = rx_data };
                    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
                    usb_serial_jtag_write_bytes(&rx_data[3], 1, 1000 / portTICK_PERIOD_MS);
                }
                usb_serial_jtag_write_bytes((uint8_t[]){STK_OK}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            }
            case 'Q': // Quit (no params)
                pmode = 0;
                gpio_set_level(RESET_PIN, 1);
                if (getch() == CRC_EOP)
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_INSYNC, STK_OK}, 2, 1000 / portTICK_PERIOD_MS);
                else
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            case 'V': { // Universal SPI command (4 bytes + CRC_EOP)
                uint8_t bytes[5];
                int got = 0;
                while (got < 5) got += usb_serial_jtag_read_bytes(bytes + got, 5 - got, 1000 / portTICK_PERIOD_MS);
                if (bytes[4] == CRC_EOP) {
                    uint8_t rx = 0;
                    for (int i = 0; i < 4; i++) rx = spi_transfer(bytes[i]);
                    uint8_t reply[3] = {STK_INSYNC, rx, STK_OK};
                    usb_serial_jtag_write_bytes(reply, 3, 1000 / portTICK_PERIOD_MS);
                } else {
                    usb_serial_jtag_write_bytes((uint8_t[]){STK_NOSYNC}, 1, 1000 / portTICK_PERIOD_MS);
                }
                break;
            }
            default:
                usb_serial_jtag_write_bytes((uint8_t[]){STK_UNKNOWN}, 1, 1000 / portTICK_PERIOD_MS);
                break;
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); // Avoid CPU hogging
    }
}
