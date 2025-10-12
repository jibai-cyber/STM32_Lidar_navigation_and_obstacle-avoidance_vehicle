#ifndef _lcd1602_H
#define _lcd1602_H

//函数声明
void lcd1602_init(void);
void lcd1602_clear(void);
void lcd1602_show_string(uint8_t x,uint8_t y,uint8_t *str);

#endif
