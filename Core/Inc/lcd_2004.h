/*
 * lcd_2004_i2c.h
 *
 *  Created on: Sep 20, 2024
 *      Author: harri
 */

#ifndef INC_LCD_2004_H_
#define INC_LCD_2004_H_

#include "main.h"

// commands
#define LCD_2004_CLEAR_DISPLAY 0x01
#define LCD_2004_RETURN_HOME 0x02
#define LCD_2004_ENTRY_MODE 0x04
#define LCD_2004_DISPLAY_CONTROL 0x08
#define LCD_2004_CURSOR_SHIFT 0x10
#define LCD_2004_FUNTION_SET 0x20
#define LCD_2004_SET_CGRAM_ADDR 0x40
#define LCD_2004_SET_DDRAM_ADDR 0x80

// flags for ENTRY_MODE
#define LCD_2004_ENTRY_LEFT 0x02
#define LCD_2004_ENTRY_RIGHT 0x00
#define LCD_2004_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_2004_ENTRY_SHIFT_DECREMENT 0x00


// flags for DISPLAY_CONTROL
#define LCD_2004_DISPLAY_ON 0x04
#define LCD_2004_CURSOR_ON 0x02
#define LCD_2004_CURSOR_OFF 0x00
#define LCD_2004_BLINK_ON 0x01
#define LCD_2004_BLINK_OFF 0x00


// flags for CURSOR_SHIFT
#define LCD_2004_DISPLAY_MOVE 0x08
#define LCD_2004_CURSOR_MOVE 0x00
#define LCD_2004_MOVE_RIGHT 0x04
#define LCD_2004_MOVE_LEFT 0x00

// flags for FUNCTION_SET
#define LCD_2004_8BIT 0x10
#define LCD_2004_4BIT 0x0
#define LCD_2004_2LINE 0x08
#define LCD_2004_1LINE 0x00
#define LCD_2004_FONT_5x10DOTS 0x04
#define LCD_2004_FONT_5x8DOTS 0x00

// flags for BACKLIGHT
#define LCD_2004_BACKLIGHT_ON 0x08

// flags for ENABLE BIT
#define LCD_2004_ENABLE 0x04

// flags for R/W BIT
#define LCD_2004_RW 0x00

// flags for REGISTER SELECT BIT
#define LCD_2004_RS 0x01


typedef struct {
	I2C_HandleTypeDef *i2c_handle;
	uint8_t i2c_addr;
	uint8_t function;
	uint8_t control;
	uint8_t mode;
	uint8_t rows;
	uint8_t cols;
	uint8_t backlight;

}lcd_2004_t;

void LCD_2004_init(lcd_2004_t*);
void LCD_2004_clear_disp(lcd_2004_t*);
void LCD_2004_return_home(lcd_2004_t*);
void LCD_2004_entry_mode_set(lcd_2004_t*);
void LCD_2004_display_ON(lcd_2004_t*);
void LCD_2004_display_OFF(lcd_2004_t*);
void LCD_2004_cursor_ON(lcd_2004_t*);
void LCD_2004_cursor_OFF(lcd_2004_t*);
void LCD_2004_blink_ON(lcd_2004_t*);
void LCD_2004_blink_OFF(lcd_2004_t*);
void LCD_2004_backlight_ON(lcd_2004_t*);
void LCD_2004_backlight_OFF(lcd_2004_t*);
void LCD_2004_print(lcd_2004_t*, char*);
void LCD_2004_set_cursor(lcd_2004_t*, uint8_t, uint8_t);

#endif /* INC_LCD_2004_H_ */
