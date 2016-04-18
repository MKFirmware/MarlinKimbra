#ifndef NEXTIONLCD_H
  #define NEXTIONLCD_H

  #if ENABLED(NEXTION)
    #define LCD_UPDATE_INTERVAL 4000
    #define NEXTION_FIRMWARE_FILE "marlinkimbra.tft"

    void ExitPopCallback(void *ptr);
    void hotPopCallback(void *ptr);
    void sethotPopCallback(void *ptr);
    void settempPopCallback(void *ptr);
    void setfanPopCallback(void *ptr);
    void setmovePopCallback(void *ptr);
    void setgcodePopCallback(void *ptr);
    void lcd_update();
    void lcd_init();
    void lcd_setstatus(const char* message, const bool persist = false);
    void lcd_setstatuspgm(const char* message, const uint8_t level = 0);
    void lcd_setalertstatuspgm(const char* message);
    void lcd_reset_alert_level();

    #if ENABLED(NEXTION_GFX)
      void gfx_clear(float x, float y, float z);
      void gfx_cursor_to(float x, float y, float z);
      void gfx_line_to(float x, float y, float z);
    #endif

    #if ENABLED(SDSUPPORT)
      void setpageSDPopCallback(void *ptr);
      void sdlistPopCallback(void *ptr);
      void sdfilePopCallback(void *ptr);
      void sdfolderPopCallback(void *ptr);
      void sdfolderUpPopCallback(void *ptr);
      void PlayPausePopCallback(void *ptr);
      void StopPopCallback(void *ptr);
      void DownloadNewFirmware();
    #endif

    FORCE_INLINE bool lcd_hasstatus() { return false; }
    FORCE_INLINE void lcd_buttons_update() {}
    FORCE_INLINE bool lcd_detected(void) { return true; }

    #define LCD_MESSAGEPGM(x) lcd_setstatuspgm(PSTR(x))
    #define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatuspgm(PSTR(x))

    char* itostr4sign(const int& x);
    char* ftostr4sign(const float& x);
    char* ftostr32sp(const float& x); // remove zero-padding from ftostr32

  #endif
#endif // NEXTIONLCD_H
