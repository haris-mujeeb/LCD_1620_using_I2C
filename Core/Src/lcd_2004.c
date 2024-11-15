/*
 * lcd_2004_i2c.c
 *
 *  Created on: Sep 20, 2024
 *      Author: Muhammad Haris Mujeeb
 */

#include "lcd_2004.h"

//static void DelayInit(void);
//static void DelayUS(void);
void send_cmd(lcd_2004_t *, uint8_t);
void send_chr(lcd_2004_t *, uint8_t);
void send(lcd_2004_t *, uint8_t,  uint8_t);
void write_4_bits(lcd_2004_t*, uint8_t);
void expander_writer(lcd_2004_t*, uint8_t);
void pulse_enable(lcd_2004_t*, uint8_t);

void LCD_2004_init(lcd_2004_t *lcd_instance){
//	lcd_instance->i2c_addr = 0x27 << 1; // for Texas Instr. PCF8574
//	lcd_instance->i2c_addr = 0x3F << 1; // for NXP PCF8574

	lcd_instance->backlight = LCD_2004_BACKLIGHT_ON;
	HAL_Delay(15);

	write_4_bits(lcd_instance, 0b00110000);
	HAL_Delay(5);
	write_4_bits(lcd_instance, 0b00110000);
	HAL_Delay(5);
	write_4_bits(lcd_instance, 0b00110000);
	HAL_Delay(1);
	write_4_bits(lcd_instance, 0b00100000);
	HAL_Delay(1);


	lcd_instance->function = LCD_2004_FUNTION_SET | LCD_2004_4BIT | LCD_2004_1LINE | LCD_2004_FONT_5x8DOTS;
	if (lcd_instance->rows > 1) {
	    lcd_instance->function |= LCD_2004_2LINE;
	} else {
	    lcd_instance->function |= LCD_2004_FONT_5x10DOTS;
	}
	send_cmd(lcd_instance, lcd_instance->function);

	lcd_instance->control = LCD_2004_DISPLAY_CONTROL | LCD_2004_DISPLAY_ON | LCD_2004_CURSOR_ON | LCD_2004_BLINK_ON;
	send_cmd(lcd_instance, lcd_instance->control);

	LCD_2004_clear_disp(lcd_instance);

	lcd_instance->mode = LCD_2004_ENTRY_MODE | LCD_2004_ENTRY_LEFT | LCD_2004_ENTRY_SHIFT_DECREMENT;
	send_cmd(lcd_instance, lcd_instance->mode);

	LCD_2004_return_home(lcd_instance);
}

void LCD_2004_backlight_ON(lcd_2004_t* lcd_instance) {
	lcd_instance->backlight = LCD_2004_BACKLIGHT_ON;
	expander_writer(lcd_instance, 0);
}

void LCD_2004_backlight_OFF(lcd_2004_t* lcd_instance) {
	lcd_instance->backlight &= ~LCD_2004_BACKLIGHT_ON;
	expander_writer(lcd_instance, 0);
}

void LCD_2004_clear_disp(lcd_2004_t* lcd_instance){
	send_cmd(lcd_instance, LCD_2004_CLEAR_DISPLAY);
	HAL_Delay(2);
}

void LCD_2004_return_home(lcd_2004_t* lcd_instance){
	send_cmd(lcd_instance, LCD_2004_RETURN_HOME);
	HAL_Delay(2);
}

// Turn the display on/off (quickly)
void LCD_2004_display_ON(lcd_2004_t* lcd_instance){
	lcd_instance->control |= LCD_2004_DISPLAY_ON;
	send_cmd(lcd_instance, LCD_2004_DISPLAY_CONTROL | lcd_instance->control );
}

void LCD_2004_display_OFF(lcd_2004_t* lcd_instance){
	lcd_instance->control &= ~LCD_2004_DISPLAY_ON;
	send_cmd(lcd_instance, LCD_2004_DISPLAY_CONTROL | lcd_instance->control );
}

// Turns the underline cursor on/off
void LCD_2004_cursor_ON (lcd_2004_t* lcd_instance){
	lcd_instance->control |= LCD_2004_CURSOR_ON;
	send_cmd(lcd_instance, lcd_instance->control);
}

void LCD_2004_cursor_OFF (lcd_2004_t* lcd_instance){
	lcd_instance->control &= ~LCD_2004_CURSOR_ON;
	send_cmd(lcd_instance, lcd_instance->control);
}

void LCD_2004_blink_ON(lcd_2004_t* lcd_instance){
	lcd_instance->control |= LCD_2004_BLINK_ON;
	send_cmd(lcd_instance, lcd_instance->control);
}

void LCD_2004_blink_OFF(lcd_2004_t* lcd_instance){
	lcd_instance->control &= ~LCD_2004_BLINK_ON;
	send_cmd(lcd_instance, lcd_instance->control);
}

void LCD_2004_print(lcd_2004_t* lcd_instance, char str[]) {
	while(*str) send_chr(lcd_instance, *str++);
}

void LCD_2004_set_cursor(lcd_2004_t *lcd_instance, uint8_t col, uint8_t row){
	uint8_t offset_row[] = {0x00, 0x40, 0x14, 0x54};
	send_cmd(lcd_instance, LCD_2004_SET_DDRAM_ADDR | (offset_row[col-1] + row-1));
}

inline void send_cmd(lcd_2004_t *lcd_instance, uint8_t cmd) {
	send(lcd_instance, cmd, 0);
}

inline void send_chr(lcd_2004_t *lcd_instance, uint8_t chr) {
	send(lcd_instance, chr, LCD_2004_RS);
}

void send(lcd_2004_t *lcd_instance, uint8_t data, uint8_t mode){
	uint8_t high_nibble = data & 0xF0;
	uint8_t low_nibble = (data << 4) & 0xF0;
	write_4_bits(lcd_instance, high_nibble | mode);
	write_4_bits(lcd_instance, low_nibble | mode);
}

inline void write_4_bits(lcd_2004_t* lcd_instance, uint8_t data){
	expander_writer(lcd_instance, data);
	pulse_enable(lcd_instance, data);
}

inline void expander_writer(lcd_2004_t* lcd_instance, uint8_t data){
	 data |= lcd_instance->backlight;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Master_Transmit(lcd_instance->i2c_handle, lcd_instance->i2c_addr, &data, I2C_MEMADD_SIZE_8BIT, 10);
	if (status != HAL_OK) Error_Handler();
}

inline void pulse_enable(lcd_2004_t* lcd_instance, uint8_t data){
	expander_writer(lcd_instance, data | LCD_2004_ENABLE);
	HAL_Delay(1);
	expander_writer(lcd_instance, data & ~LCD_2004_ENABLE);
	HAL_Delay(1);
}


//static void DelayInit(void)
//{
//  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
//  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;
//
//  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
//  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;
//
//  DWT->CYCCNT = 0;
//
//  /* 3 NO OPERATION instructions */
//  __ASM volatile ("NOP");
//  __ASM volatile ("NOP");
//  __ASM volatile ("NOP");
//}
//
//static void DelayUS(uint32_t us) {
//  uint32_t cycles = (SystemCoreClock/1000000L)*us;
//  uint32_t start = DWT->CYCCNT;
//  volatile uint32_t cnt;
//
//  do
//  {
//    cnt = DWT->CYCCNT - start;
//  } while(cnt < cycles);
//}
