#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <ESP32_4848S040.h>
#include <lvgl.h>
#include "touch.h"

#define GFX_BL 38
Arduino_ESP32SPI* bus;
Arduino_RGB_Display* gfx;

int16_t w, h, text_size, banner_height, graph_baseline, graph_height, channel_width, signal_width;
#define BYTE_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565)) /*will be 2 for RGB565 */
static uint8_t buf1[480 * 480 / 10 * BYTE_PER_PIXEL];
lv_display_t *display;


/* lvgl */
#define TFT_HOR_RES   480
#define TFT_VER_RES   480
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];


// Relay config (gpio)
#define GPIO_RELAY1  40
#define GPIO_RELAY2  2
#define GPIO_RELAY3  1


int i = 0;

/* Display flushing */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint8_t * px_map)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  // gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_t * indev, lv_indev_data_t * data )
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PRESSED;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      //Serial.print("Touched : "); Serial.print(touch_last_x); Serial.print(" x ");Serial.println(touch_last_y);
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_RELEASED;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}



static uint32_t my_tick(void)
{
    return millis();
}



void process_relay_action(lv_event_t * e, int relay)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * btn = (lv_obj_t*)lv_event_get_current_target(e);

  if(code == LV_EVENT_CLICKED) { // If it's a simple push button
    LV_LOG_USER("Clicked");
  }
  else if(code == LV_EVENT_VALUE_CHANGED) { // case of toggle button
    Serial.println("Toggled");
    int gpio = 0;
    switch (relay) {
      case 1 : gpio = GPIO_RELAY1; break;
      case 2 : gpio = GPIO_RELAY2; break;
      case 3 : gpio = GPIO_RELAY3; break;                 
    }    
    if (lv_obj_has_state(btn, LV_STATE_CHECKED) == true) {
      Serial.println("Is now checked");    
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, HIGH);
    }
    else {
      Serial.println("Is now released");
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, LOW);
    }
  }

}

static void event_handler_relay1(lv_event_t * e)
{
  process_relay_action(e,1);
}

static void event_handler_relay2(lv_event_t * e)
{
  process_relay_action(e,2);
}

static void event_handler_relay3(lv_event_t * e)
{
  process_relay_action(e,3);

}

void relay_gui(void)
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn1, event_handler_relay1, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -160);
    lv_obj_add_flag(btn1, LV_OBJ_FLAG_CHECKABLE);
    //lv_obj_set_height(btn1, LV_SIZE_CONTENT);
    lv_obj_set_size(btn1, 300, 75);
    label = lv_label_create(btn1);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_label_set_text(label, "Relais 1");
    lv_obj_center(label);

    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn2, event_handler_relay2, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    //lv_obj_set_height(btn2, LV_SIZE_CONTENT);
    lv_obj_set_size(btn2, 300, 75);
    label = lv_label_create(btn2);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_label_set_text(label, "Relais 2");
    lv_obj_center(label);

    lv_obj_t * btn3 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn3, event_handler_relay3, LV_EVENT_ALL, NULL);
    lv_obj_align(btn3, LV_ALIGN_CENTER, 0, 160);
    lv_obj_add_flag(btn3, LV_OBJ_FLAG_CHECKABLE);
    //lv_obj_set_height(btn3, LV_SIZE_CONTENT);
    lv_obj_set_size(btn3, 300, 75);
    label = lv_label_create(btn3);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_label_set_text(label, "Relais 3");
    lv_obj_center(label);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup...");
  touch_init();

  // 9-bit mode SPI
  bus = new Arduino_ESP32SPI(
  GFX_NOT_DEFINED /* DC */, 39 /* CS */, 48 /* SCK */, 47 /* MOSI */, GFX_NOT_DEFINED /* MISO */);

  // panel (Hardware) specific
  Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    18 /* DE */, 17 /* VSYNC */, 16 /* HSYNC */, 21 /* PCLK */,
    4 /* R0 */, 5 /* R1 */, 6 /* R2 */, 7 /* R3 */, 15 /* R4 */,
    8 /* G0 */, 20 /* G1 */, 3 /* G2 */, 46 /* G3 */, 9 /* G4 */, 10 /* G5 */,
    11 /* B0 */, 12 /* B1 */, 13 /* B2 */, 14 /* B3 */, 0 /* B4 */,
    1 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    1 /* vsync_polarity */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);

  // panel parameters & setup
  gfx = new Arduino_RGB_Display(
    480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST */, st7701_4848s040_init_operations, sizeof(st7701_4848s040_init_operations));

  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  gfx->setCursor(100, 200);
  gfx->displayOn();
  gfx->fillScreen(BLACK);
  gfx->setTextColor(BLUE);
  gfx->setTextSize(6 /* x scale */, 6 /* y scale */, 2 /* pixel_margin */);
  gfx->println("Prout !");

  /* Configuration LVGL */
  lv_init();

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(my_tick);
  lv_display_t * disp;
  disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

  // Touch Driver
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);   /*See below.*/
  lv_indev_set_read_cb(indev, my_touchpad_read);

  // build GUI
  relay_gui();
}

void loop()
{  
 #if 0   
    Serial.println("");
    Serial.println("In the loop");
    Serial.print("Total heap: "); Serial.println(ESP.getHeapSize());
    Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());
    Serial.print("Total PSRAM: "); Serial.println(ESP.getPsramSize());
    Serial.print("Free PSRAM: "); Serial.println(ESP.getFreePsram());    
 #endif   
    lv_timer_handler(); /* let the GUI do its work */
    //lv_tick_inc(10);
    delay(5);
}