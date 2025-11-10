#include "main.h"
#include "stdlib.h"
#include "font8x12_st7302.h"


extern SPI_HandleTypeDef hspi1;
extern uint8_t lcd_buffer[3813];
extern uint8_t cur_x;
extern uint8_t cur_y;

void lcd_send_command(uint8_t command){
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  SPI1->DR = command;
  while(SPI1->SR & SPI_SR_BSY){};
  if(SPI1->SR & SPI_SR_RXNE)SPI1->DR;
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_SET);
}

void lcd_send_param(uint8_t data){
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_RESET);
  SPI1->DR = data;
  while(SPI1->SR & SPI_SR_BSY){};
  if(SPI1->SR & SPI_SR_RXNE)SPI1->DR;
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_SET);
}

void lcd_clear(void){
  lcd_send_command(0xb9);  // enable CLR RAM
  lcd_send_param(0xe3);
  HAL_Delay(200);
  lcd_send_command(0xb9);  // disable CLR RAM
  lcd_send_param(0x23);
  //_clear_buffer();
}

//void convert_font(void){
//  uint16_t i;
//  uint8_t row, col, b12t, mask;
//  for(i=0;i<256;i++){
//    for(row=0;row<12;row++){
//      for(col=0;col<8;col++){
//        b12t = (row>>2)+((col>>1)*3);// Байт 1 из 12
//        mask = (0x80 >> (col&0x01)+((row&0x03)*2));
//        if((fnt8x12[(i*12)+row]<<col) & 0x80){
//          new_font[(i*12)+b12t] |= mask;
//        }else{
//          new_font[(i*12)+b12t] &= ~(mask);
//        }
//      }
//    }
//  } 
//}



void lcd_put_pixel(uint8_t x, uint8_t y){
  if(x>249 || y>121)return;
  uint8_t row = y/12;   //Строка из 12 пикселей
  uint8_t col = x/8;    //Колонка из 8 пикселей
  uint8_t row_s = y%12;    //Номер пикселя Y в знакоместе
  uint8_t col_s = x%8;    //Номер пикселя X в знакоместе
  uint8_t byte_y = row_s/4;     //Номер байта Y в знакоместе
  uint8_t byte_x = col_s/2;     //Номер байта X в знакоместе
  if(col_s & 0x01){
    //40_10_4_1
    lcd_buffer[(row*375)+(col*12)+byte_y+(3*byte_x)] |= 0x40>>((row_s%4)*2);
  }else{
    //80_20_8_2
    lcd_buffer[(row*375)+(col*12)+byte_y+(3*byte_x)] |= 0x80>>((row_s%4)*2);
  }
}

void _swap_int8_t(int8_t *a, int8_t *b){
  int8_t temp = *a;
  *a = *b;
  *b = temp;
}

void lcd_draw_line(int8_t x0, int8_t y0, int8_t x1, int8_t y1) {
  int16_t step = abs(y1 - y0) > abs(x1 - x0);
  if (step) {
    _swap_int8_t(&x0, &y0);
    _swap_int8_t(&x1, &y1);
  }

  if (x0 > x1) {
    _swap_int8_t(&x0, &x1);
    _swap_int8_t(&y0, &y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (step) {
      lcd_put_pixel(y0, x0);
    } else {
      lcd_put_pixel(x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

void lcd_draw_circle(int8_t x0, int8_t y0, int8_t radius) {
  int8_t x = 0, y = radius;
  int8_t dp = 1 - radius;
  do{
    if (dp < 0)
      dp = dp + (x++) * 2 + 3;
    else
      dp = dp + (x++) * 2 - (y--) * 2 + 5;

    lcd_put_pixel(x0 + x, y0 + y);     //For the 8 octants
    lcd_put_pixel(x0 - x, y0 + y);
    lcd_put_pixel(x0 + x, y0 - y);
    lcd_put_pixel(x0 - x, y0 - y);
    lcd_put_pixel(x0 + y, y0 + x);
    lcd_put_pixel(x0 - y, y0 + x);
    lcd_put_pixel(x0 + y, y0 - x);
    lcd_put_pixel(x0 - y, y0 - x);

  } while (x < y);

  lcd_put_pixel(x0 + radius, y0);
  lcd_put_pixel(x0, y0 + radius);
  lcd_put_pixel(x0 - radius, y0);
  lcd_put_pixel(x0, y0 - radius);
}

void lcd_put_char(uint8_t chr, uint8_t inv){
  uint16_t offset = chr*12;
  uint8_t i;
  for(i=0;i<12;i++){
    if(inv){                    //375 байт в текстовой строке
      lcd_buffer[((cur_x*12)+(cur_y*375))+i] = ~(fnt8x12[offset+i]);
    }else{
      lcd_buffer[((cur_x*12)+(cur_y*375))+i] = fnt8x12[offset+i];
    }
  }
  cur_x++;
  if(cur_x>30){
    cur_x=0;
    cur_y++;
  }       
}

void lcd_put_string(uint8_t * str, uint8_t inv){
  while(*str){
    lcd_put_char(*str, inv);
    str++;
  }
}

void lcd_init(void){
  SPI1->CR1 |= SPI_CR1_SPE;
  HAL_GPIO_WritePin(LCD_NRST_GPIO_Port, LCD_NRST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_NRST_GPIO_Port, LCD_NRST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  lcd_send_command(0x38);
  lcd_send_command(0xEB);//Enable OTP
  lcd_send_param(0x02);
  lcd_send_command(0xD7);//OTP Load Control
  lcd_send_param(0x68);
  lcd_send_command(0xD1);//Auto Power Control
  lcd_send_param(0x01);
  lcd_send_command(0xC0);//Gate Voltage Setting VGH=12V ; VGL=-5V
  lcd_send_param(0x80);
  lcd_send_command(0xC1);//VSH Setting
  lcd_send_param(0x28);//
  lcd_send_param(0x28);
  lcd_send_param(0x28);
  lcd_send_param(0x28);
  lcd_send_param(0x14);
  lcd_send_param(0x00);
  lcd_send_command(0xC2);//VSL Setting VSL=0
  lcd_send_param(0x00);
  lcd_send_param(0x00);
  lcd_send_param(0x00);
  lcd_send_param(0x00);
  lcd_send_command(0xCB);//VCOMH Setting
  lcd_send_param(0x14);//14  0C   7
  lcd_send_command(0xB4);//Gate EQ Setting HPM EQ LPM EQ
  lcd_send_param(0xE5);
  lcd_send_param(0x77);
  lcd_send_param(0xF1);
  lcd_send_param(0xFF);
  lcd_send_param(0xFF);
  lcd_send_param(0x4F);
  lcd_send_param(0xF1);
  lcd_send_param(0xFF);
  lcd_send_param(0xFF);
  lcd_send_param(0x4F);
  lcd_send_command(0x11);//Sleep out
  HAL_Delay(100);  // delay_ms 100ms
  lcd_send_command(0xC7);//OSC Setting
  lcd_send_param(0xA6);
  lcd_send_param(0xE9);
  lcd_send_command(0xB0);   //Duty Setting
  lcd_send_param(0x64);  //250duty/4=63
  lcd_send_command(0x36);//Memory Data Access Control
  lcd_send_param(0x20);
  lcd_send_command(0x3A);//Data Format Select 3 write for 24 bit
  lcd_send_param(0x11);
  lcd_send_command(0xB9);//Source Setting
  lcd_send_param(0x23);
  lcd_send_command(0xB8);//Panel Setting Frame inversion
  lcd_send_param(0x09);
  lcd_send_command(0x2A);////Column Address Setting S61~S182
  lcd_send_param(0x05);
  lcd_send_param(0x36);
  lcd_send_command(0x2B);////Row Address Setting G1~G250
  lcd_send_param(0x00);
  lcd_send_param(0xC7);
  lcd_send_command(0xD0);
  lcd_send_param(0x1F);
  lcd_send_command(0x29);//Display on
  lcd_send_command(0xB9);//enable CLR RAM
  lcd_send_param(0xE3);
  HAL_Delay(100);
  lcd_send_command(0xB9);//enable CLR RAM
  lcd_send_param(0x23);
  lcd_send_command(0x72);
  lcd_send_param(0x00);         //Destress OFF
  lcd_send_command(0x39);//LPM
  lcd_send_command(0x2A);   //Column Address Setting
  lcd_send_command(0x19);
  lcd_send_param(0x23);  //35
  lcd_send_command(0x2B);   //Row Address Setting
  lcd_send_param(0);
  lcd_send_param(0x7C);
  lcd_send_param(0x2C);   //write image data
  HAL_Delay(100);
  lcd_clear();
}

void lcd_update(void){
  lcd_send_command(0x2a);
  lcd_send_param(0x19);
  lcd_send_param(0x19 + 122 / 12);
  lcd_send_command(0x2b);
  lcd_send_param(0x00);
  lcd_send_param(250 / 2 - 1);
  lcd_send_command(0x2c);
  HAL_Delay(1);
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, lcd_buffer, 3813, 100);
  HAL_GPIO_WritePin(LCD_NCS_GPIO_Port, LCD_NCS_Pin, GPIO_PIN_SET);
  
  //SPI1->CR1 &= ~(SPI_CR1_DFF);
  // pin reset
//  delayMicroseconds(50);
//  digitalWrite(_cs_pin, HIGH);
}