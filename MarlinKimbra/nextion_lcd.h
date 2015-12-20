#ifndef NEXTIONLCD_H
  #define NEXTIONLCD_H

  #if ENABLED(NEXTION)
    #define LCD_UPDATE_INTERVAL 5000

    void ExitPopCallback(void *ptr);
    void setpagePopCallback(void *ptr);
    void hotPopCallback(void *ptr);
    void sethotPopCallback(void *ptr);
    void settempPopCallback(void *ptr);
    void setfanPopCallback(void *ptr);
    void setmovePopCallback(void *ptr);
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
      void sdlistPopCallback(void *ptr);
      void sdfilePopCallback(void *ptr);
      void sdfolderPopCallback(void *ptr);
      void sdfolderUpPopCallback(void *ptr);
      void PlayPausePopCallback(void *ptr);
      void StopPopCallback(void *ptr);
    #endif

    FORCE_INLINE bool lcd_hasstatus() { return false; }
    FORCE_INLINE void lcd_buttons_update() {}
    FORCE_INLINE bool lcd_detected(void) { return true; }

    #define LCD_MESSAGEPGM(x) lcd_setstatuspgm(PSTR(x))
    #define LCD_ALERTMESSAGEPGM(x) lcd_setalertstatuspgm(PSTR(x))
    
    char *itostr2(const uint8_t &x);
    char *itostr31(const int &xx);
    char *itostr3(const int &xx);
    char *itostr3left(const int &xx);
    char *itostr4(const int &xx);

    char *ltostr7(const long &xx);

    char *ftostr3(const float &x);
    char *ftostr30(const float &x);
    char *ftostr31ns(const float &x); // float to string without sign character
    char *ftostr31(const float &x);
    char *ftostr32(const float &x);
    char *ftostr43(const float &x);
    char *ftostr12ns(const float &x); 
    char *ftostr32sp(const float &x); // remove zero-padding from ftostr32
    char *ftostr5(const float &x);
    char *ftostr51(const float &x);
    char *ftostr52(const float &x);
  #endif
#endif // NEXTIONLCD_H
