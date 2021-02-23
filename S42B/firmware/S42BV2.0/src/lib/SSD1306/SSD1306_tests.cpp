#include <string.h>
#include <stdio.h>
#include "SSD1306.h"
#include "SSD1306_tests.h"

void SSD1306_TestBorder() {
    SSD1306_Fill(BLACK);

    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    uint8_t x = 0;
    uint8_t y = 0;
    do {
        SSD1306_DrawPixel(x, y, BLACK);

        if((y == 0) && (x < 127))
            x++;
        else if((x == 127) && (y < (SSD1306_HEIGHT-1)))
            y++;
        else if((y == (SSD1306_HEIGHT-1)) && (x > 0)) 
            x--;
        else
            y--;

        SSD1306_DrawPixel(x, y, WHITE);
        SSD1306_UpdateScreen();

        HAL_Delay(5);
        end = HAL_GetTick();
    } while((end - start) < 8000);

    HAL_Delay(1000);
}
/*
void SSD1306_TestFonts() {
    SSD1306_Fill(BLACK);
    SSD1306_SetCursor(2, 0);
    SSD1306_WriteString("Font 16x26", Font_16x26, WHITE);
    SSD1306_SetCursor(2, 26);
    SSD1306_WriteString("Font 11x18", Font_11x18, WHITE);
    SSD1306_SetCursor(2, 26+18);
    SSD1306_WriteString("Font 7x10", Font_7x10, WHITE);
    SSD1306_SetCursor(2, 26+18+10);
    SSD1306_WriteString("Font 6x8", Font_6x8, WHITE);
    SSD1306_UpdateScreen();
}
*/
void SSD1306_TestFPS() {
    SSD1306_Fill(WHITE);
   
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    int fps = 0;
    char message[] = "ABCDEFGHIJK";
   
    SSD1306_SetCursor(2,0);
    //SSD1306_WriteString("Testing...", Font_11x18, BLACK);
   
    do {
        SSD1306_SetCursor(2, 18);
        SSD1306_WriteString(message, Font_11x18, BLACK);
        SSD1306_UpdateScreen();
       
        char ch = message[0];
        memmove(message, message+1, sizeof(message)-2);
        message[sizeof(message)-2] = ch;

        fps++;
        end = HAL_GetTick();
    } while((end - start) < 5000);
   
    HAL_Delay(1000);

    char buff[64];
    fps = (float)fps / ((end - start) / 1000.0);
    snprintf(buff, sizeof(buff), "~%d FPS", fps);
   
    SSD1306_Fill(WHITE);
    SSD1306_SetCursor(2, 2);
    SSD1306_WriteString(buff, Font_11x18, BLACK);
    SSD1306_UpdateScreen();
}

void SSD1306_TestLine() {

  SSD1306_Line(1,1,SSD1306_WIDTH - 1,SSD1306_HEIGHT - 1,WHITE);
  SSD1306_Line(SSD1306_WIDTH - 1,1,1,SSD1306_HEIGHT - 1,WHITE);
  SSD1306_UpdateScreen();
  return;
}

void SSD1306_TestRectangle() {
  uint32_t delta;

  for(delta = 0; delta < 5; delta ++) {
    SSD1306_DrawRectangle(1 + (5*delta),1 + (5*delta) ,SSD1306_WIDTH-1 - (5*delta),SSD1306_HEIGHT-1 - (5*delta),WHITE);
  }
  SSD1306_UpdateScreen();
  return;
}

void SSD1306_TestCircle() {
  uint32_t delta;

  for(delta = 0; delta < 5; delta ++) {
    SSD1306_DrawCircle(20* delta+30, 15, 10, WHITE);
  }
  SSD1306_UpdateScreen();
  return;
}

void SSD1306_TestArc() {
  SSD1306_DrawArc(30, 30, 30, 20, 270, WHITE);
  SSD1306_UpdateScreen();
  return;
}

void SSD1306_TestPolyline() {
  SSD1306_VERTEX loc_vertex[] =
  {
      {35,40},
      {40,20},
      {45,28},
      {50,10},
      {45,16},
      {50,10},
      {53,16}
  };

  SSD1306_Polyline(loc_vertex,sizeof(loc_vertex)/sizeof(loc_vertex[0]),WHITE);
  SSD1306_UpdateScreen();
  return;
}

void SSD1306_TestAll() {
    SSD1306_Init(OLED_PANEL_SPI);
    SSD1306_TestFPS();
    HAL_Delay(3000);
    SSD1306_TestBorder();
    SSD1306_TestFonts();
    HAL_Delay(3000);
    SSD1306_Fill(BLACK);
    SSD1306_TestRectangle();
    SSD1306_TestLine();
    HAL_Delay(3000);
    SSD1306_Fill(BLACK);
    SSD1306_TestPolyline();
    HAL_Delay(3000);
    SSD1306_Fill(BLACK);
    SSD1306_TestArc();
    HAL_Delay(3000);
    SSD1306_Fill(BLACK);
    SSD1306_TestCircle();
    HAL_Delay(3000);
}

