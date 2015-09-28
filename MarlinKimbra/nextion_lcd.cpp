#include "base.h"

#if ENABLED(NEXTION)
  #include "Marlin_main.h"
  #include "cardreader.h"
  #include "temperature.h"

  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    #include "vector_3.h"
  #endif

  #include "planner.h"
  #include "stepper_indirection.h"
  #include "stepper.h"
  #include "configuration_store.h"
  #include "nextion_lcd.h"
  #include <Nextion.h>

  const float MaxWave   = 0.2;
  bool NextionON        = false;
  bool PageInfo         = false;
  char buffer[100]      = {0};
  uint32_t slidermaxval = 20;
  char lcd_status_message[30] = WELCOME_MSG; // worst case is kana with up to 3*LCD_WIDTH+1
  uint8_t lcd_status_message_level = 0;
  static millis_t next_lcd_update_ms;
  static millis_t return_to_status_ms = 30000;

  // Page
  NexPage Pstart      = NexPage(0, 0, "start");
  NexPage Pinfo       = NexPage(1, 0, "info");
  NexPage Ptemp       = NexPage(2, 0, "temp");
  NexPage Pmenu       = NexPage(3, 0, "menu");
  NexPage Psdcard     = NexPage(4, 0, "sdcard");
  NexPage Psetup      = NexPage(5, 0, "setup");

  // Text
  NexText Hotend0     = NexText(1,  1,  "t0");
  NexText Hotend1     = NexText(1,  4,  "t1");
  NexText Hotend2     = NexText(1,  5,  "t2");
  NexText Hotend21    = NexText(1,  6,  "h2");
  NexText LedStatus   = NexText(1,  7,  "t4");
  NexText LedCoord    = NexText(1,  8,  "t5");
  NexText set0        = NexText(2,  2,  "set0");
  NexText set1        = NexText(2, 15,  "set1");
  NexText sdrow0      = NexText(4,  3,  "t0");
  NexText sdrow1      = NexText(4,  4,  "t1");
  NexText sdrow2      = NexText(4,  5,  "t2");
  NexText sdrow3      = NexText(4,  6,  "t3");
  NexText sdrow4      = NexText(4,  7,  "t4");
  NexText sdrow5      = NexText(4,  8,  "t5");
  NexText sdfolder    = NexText(4, 23,  "sdfolder");

  // Picture
  NexPicture Menu     = NexPicture(1, 10, "p0");
  NexPicture MSD      = NexPicture(1, 11, "p1");
  NexPicture MSetup   = NexPicture(1, 12, "p2");
  NexPicture Hend0    = NexPicture(1, 13, "p3");
  NexPicture Hend1    = NexPicture(1, 14, "p4");
  NexPicture Hend2    = NexPicture(1, 15, "p5");
  NexPicture Fanpic   = NexPicture(1, 19, "p6");
  NexPicture Folder0  = NexPicture(4,  9, "p0");
  NexPicture Folder1  = NexPicture(4, 10, "p1");
  NexPicture Folder2  = NexPicture(4, 11, "p2");
  NexPicture Folder3  = NexPicture(4, 12, "p3");
  NexPicture Folder4  = NexPicture(4, 13, "p4");
  NexPicture Folder5  = NexPicture(4, 14, "p5");
  NexPicture Folderup = NexPicture(4, 15, "p6");
  NexPicture Exit     = NexPicture(4, 16, "p7");

  // Progress Bar

  // Slider
  NexSlider sdlist    = NexSlider(4, 1,   "h0");

  // Wafeform
  NexWaveform Graph0  = NexWaveform(1,  9, "s0");
  NexWaveform Graph1  = NexWaveform(1, 24, "s1");
  NexWaveform Graph2  = NexWaveform(1, 25, "s2");

  // Touch area
  NexHotspot hot0     = NexHotspot(1, 14, "hot0");
  NexHotspot hot1     = NexHotspot(1, 16, "hot1");
  NexHotspot hot2     = NexHotspot(1, 18, "hot2");
  NexHotspot m11      = NexHotspot(2, 14, "m11");
  NexHotspot tup      = NexHotspot(2, 16, "tup");
  NexHotspot tdown    = NexHotspot(2, 17, "tdown");

  // Timer
  NexTimer startimer  = NexTimer(0,  1, "tm0");
  NexTimer fantimer   = NexTimer(1, 23, "tm0");

  // Variable
  NexVar Hotend       = NexVar(1, 20, "he");
  NexVar Bed          = NexVar(1, 21, "bed");
  NexVar filename0    = NexVar(4, 19, "va0");
  NexVar filename1    = NexVar(4, 20, "va1");
  NexVar filename2    = NexVar(4, 21, "va2");
  NexVar filename3    = NexVar(4, 22, "va3");
  NexVar filename4    = NexVar(4, 23, "va4");
  NexVar filename5    = NexVar(4, 24, "va5");

  NexTouch *nex_listen_list[] =
  {
    &Pstart,
    &Menu,
    &MSD,
    &MSetup,
    &Fanpic,
    &hot0,
    &hot1,
    &hot2,
    &m11,
    &tup,
    &tdown,
    &sdlist,
    &sdrow0,
    &sdrow1,
    &sdrow2,
    &sdrow3,
    &sdrow4,
    &sdrow5,
    &Folderup,
    &Exit,
    NULL
  };

  void setpageInfo() {
    Pinfo.show();

    PageInfo = true;

    #if HAS_TEMP_0
      Hotend.setValue(1);
    #endif
    #if HAS_TEMP_1
      Hotend.setValue(2);
    #endif
    #if HAS_TEMP_2
      Hotend.setValue(3);
    #elif HAS_TEMP_BED
      Hotend21.setText("BED");
      Bed.setValue(1);
    #endif

    lcd_setstatus(lcd_status_message);
  }

  #if ENABLED(SDSUPPORT)
    void printrowsd(uint8_t row, const bool folder, const char* filename, char* longFilename) {
      const char* cmd;
      if (longFilename[0])
        cmd = longFilename;
      else
        cmd = filename;

      switch (row) {
        case 0:
        {
          if (folder) {
            Folder0.setPic(18);
            sdrow0.attachPop(sdfolderPopCallback, &sdrow0);
          } else if (cmd == "") {
            Folder0.setPic(17);
            sdrow0.detachPop();
          } else {
            Folder0.setPic(17);
            sdrow0.attachPop(sdfilePopCallback, &sdrow0);
          }
          sdrow0.setText(cmd);
          filename0.setText(filename);
        }
        case 1:
        {
          if (folder) {
            Folder1.setPic(18);
            sdrow1.attachPop(sdfolderPopCallback, &sdrow1);
          } else if (cmd == "") {
            Folder1.setPic(17);
            sdrow1.detachPop();
          } else {
            Folder1.setPic(17);
            sdrow1.attachPop(sdfilePopCallback, &sdrow1);
          }
          sdrow1.setText(cmd);
          filename1.setText(filename);
        }
        case 2:
        {
          if (folder) {
            Folder2.setPic(18);
            sdrow2.attachPop(sdfolderPopCallback, &sdrow2);
          } else if (cmd == "") {
            Folder2.setPic(17);
            sdrow2.detachPop();
          } else {
            Folder2.setPic(17);
            sdrow2.attachPop(sdfilePopCallback, &sdrow2);
          }
          sdrow2.setText(cmd);
          filename2.setText(filename);
        }
        case 3:
        {
          if (folder) {
            Folder3.setPic(18);
            sdrow3.attachPop(sdfolderPopCallback, &sdrow3);
          } else if (cmd == "") {
            Folder3.setPic(17);
            sdrow3.detachPop();
          } else {
            Folder3.setPic(17);
            sdrow3.attachPop(sdfilePopCallback, &sdrow3);
          }
          sdrow3.setText(cmd);
          filename3.setText(filename);
        }
        case 4:
        {
          if (folder) {
            Folder4.setPic(18);
            sdrow4.attachPop(sdfolderPopCallback, &sdrow4);
          } else if (cmd == "") {
            Folder4.setPic(17);
            sdrow4.detachPop();
          } else {
            Folder4.setPic(17);
            sdrow4.attachPop(sdfilePopCallback, &sdrow4);
          }
          sdrow4.setText(cmd);
          filename4.setText(filename);
        }
        case 5:
        {
          if (folder) {
            Folder5.setPic(18);
            sdrow5.attachPop(sdfolderPopCallback, &sdrow5);
          } else if (cmd == "") {
            Folder5.setPic(17);
            sdrow5.detachPop();
          } else {
            Folder5.setPic(17);
            sdrow5.attachPop(sdfilePopCallback, &sdrow5);
          }
          sdrow5.setText(cmd);
          filename5.setText(filename);
        }
      }
    }

    static void setrowsdcard(uint32_t number = 0) {
      uint16_t fileCnt = card.getnrfilenames();
      uint32_t i = 0;
      card.getWorkDirName();

      if (card.filename[0] != '/') {
        Folderup.setPic(20);
        Folderup.attachPop(sdfolderUpPopCallback);
        sdfolder.setText(card.filename);
      } else {
        Folderup.detachPop();
        Folderup.setPic(19);
        sdfolder.setText("");
      }

      for (uint8_t row = 0; row < 6; row++) {
        i = row + number;
        if (i < fileCnt) {
          card.getfilename(i);
          printrowsd(row, card.filenameIsDir, card.filename, card.longFilename);
        } else {
          printrowsd(row, false, "", "");
        }
      }
      sendCommand("ref 0");
    }

    static void setpagesdcard() {
      PageInfo = false;
      Psdcard.show();
      uint16_t fileCnt = card.getnrfilenames();

      if (fileCnt <= 6)
        slidermaxval = 0;
      else
        slidermaxval  = fileCnt - 6;

      uint16_t hig = 210 - slidermaxval * 10;
      if (hig < 10) hig = 10;

      sdlist.setHigValue(hig);
      sdlist.setMaxValue(slidermaxval);
      sdlist.setValue(slidermaxval);
      sendCommand("ref 0");

      setrowsdcard();
    }

    void sdlistPopCallback(void *ptr) {
        uint32_t number = 0;
        sdlist.getValue(&number);
        number = slidermaxval - number;
        setrowsdcard(number);
    }

    static void menu_action_sdfile(const char* filename) {
    char cmd[30];
    char* c;
    sprintf_P(cmd, PSTR("M23 %s"), filename);
    for(c = &cmd[4]; *c; c++) *c = tolower(*c);
    enqueuecommand(cmd);
    enqueuecommands_P(PSTR("M24"));
    setpageInfo();
  }

    static void menu_action_sddirectory(const char* filename) {
      card.chdir(filename);
      setpagesdcard();
    }

    void sdfilePopCallback(void *ptr) {
      memset(buffer, 0, sizeof(buffer));

      if (ptr == &sdrow0)
        filename0.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow1)
        filename1.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow2)
        filename2.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow3)
        filename3.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow4)
        filename4.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow5)
        filename5.getText(buffer, sizeof(buffer));

      menu_action_sdfile(buffer);
    }

    void sdfolderPopCallback(void *ptr) {
      memset(buffer, 0, sizeof(buffer));

      if (ptr == &sdrow0)
        filename0.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow1)
        filename1.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow2)
        filename2.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow3)
        filename3.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow4)
        filename4.getText(buffer, sizeof(buffer));
      else if (ptr == &sdrow5)
        filename5.getText(buffer, sizeof(buffer));

      menu_action_sddirectory(buffer);
    }

    void sdfolderUpPopCallback(void *ptr) {
      card.updir();
      setpagesdcard();
    }

    void ExitPopCallback(void *ptr) {
      setpageInfo();
    }
  #endif

  void PstartPopCallback(void *ptr) {
      setpageInfo();
  }

  void hotPopCallback(void *ptr) {
    Ptemp.show();
    PageInfo = false;
    memset(buffer, 0, sizeof(buffer));
    if (ptr == &hot0) {
      if (degTargetHotend(0) != 0) {
        itoa(degTargetHotend(0), buffer, 10);
      }
      set1.setText("M104 T0 S");
    }
    if (ptr == &hot1) {
      if (degTargetHotend(1) != 0) {
        itoa(degTargetHotend(1), buffer, 10);
      }
      set1.setText("M104 T1 S");
    }

    #if HAS_TEMP_2
      if (ptr == &hot2) {
        if (degTargetHotend(2) != 0) {
          itoa(degTargetHotend(2), buffer, 10);
        }
        set1.setText("M104 T2 S");
      }
    #elif HAS_TEMP_BED
      if (ptr == &hot2) {
        if (degTargetBed() != 0) {
          itoa(degTargetBed(), buffer, 10);
        }
        set1.setText("M140 S");
      }
    #endif

    set0.setText(buffer);
  }

  void settempPopCallback(void *ptr) {
    uint16_t number;

    memset(buffer, 0, sizeof(buffer));
    set0.getText(buffer, sizeof(buffer));

    number = atoi(buffer);

    if (ptr == &tup) number += 1;
    if (ptr == &tdown) number -= 1;

    memset(buffer, 0, sizeof(buffer));
    itoa(number, buffer, 10);

    set0.setText(buffer);
  }

  void sethotPopCallback(void *ptr) {
    memset(buffer, 0, sizeof(buffer));
    set1.getText(buffer, sizeof(buffer));
    enqueuecommands_P(buffer);
    setpageInfo();
  }

  void setpagePopCallback(void *ptr) {
    if (ptr == &Menu)
      Pmenu.show();
    if (ptr == &MSetup)
      Psetup.show();

    #if ENABLED(SDSUPPORT)
      if (ptr == &MSD)
        setpagesdcard();
    #endif
  }

  void setfanPopCallback(void *ptr) {
    if (fanSpeed) fanSpeed = 0;
    else fanSpeed = 255;
  }

  void lcd_init() {
    delay(1000);
    NextionON = nexInit();

    if (!NextionON) {
      ECHO_LM(DB, "Nextion LCD not connected!");
    } else {
      ECHO_LM(DB, "Nextion LCD connected!");

      Pstart.attachPop(PstartPopCallback);

      #if ENABLED(SDSUPPORT)
        MSD.attachPop(setpagePopCallback, &MSD);
        sdlist.attachPop(sdlistPopCallback);
        Exit.attachPop(ExitPopCallback);
      #endif

      #if HAS_TEMP_0
        hot0.attachPop(hotPopCallback,      &hot0);
      #endif
      #if HAS_TEMP_1
        hot1.attachPop(hotPopCallback,      &hot1);
      #endif
      #if HAS_TEMP_2 || HAS_TEMP_BED
        hot2.attachPop(hotPopCallback,      &hot2);
      #endif

      Menu.attachPop(setpagePopCallback,    &Menu);
      MSetup.attachPop(setpagePopCallback,  &Menu);
      Fanpic.attachPop(setfanPopCallback,   &Fanpic);
      m11.attachPop(sethotPopCallback,      &m11);
      tup.attachPop(settempPopCallback,     &tup);
      tdown.attachPop(settempPopCallback,   &tdown);

      startimer.enable();
    }
  }

  static void temptoLCD(int h, int T1, int T2) {
    char valuetemp[25] = {0};
    memset(buffer, 0, sizeof(buffer));
    itoa(T1, valuetemp, 10);
    strcat(buffer, valuetemp);
    strcat(buffer, "/");
    itoa(T2, valuetemp, 10);
    strcat(buffer, valuetemp);
    uint32_t color = 1023;
    uint32_t prc = (T1/(T2 + 0.1)) * 100;
    
    if (prc >= 50 && prc < 75)
      color = 65519;
    else if (prc >= 75 && prc < 95)
      color = 64487;
    else if (prc >= 95 && prc < 100)
      color = 63488;

    switch (h) {
      case 0:
      {
        Hotend0.setText(buffer);
        Hotend0.setColor(color);
        Graph0.addValue(0, (int)(T1 * MaxWave));
        Graph0.addValue(1, (int)(T2 * MaxWave));
        break;
      }
      case 1:
      {
        Hotend1.setText(buffer);
        Hotend1.setColor(color);
        Graph1.addValue(0, (int)(T1 * MaxWave));
        Graph1.addValue(1, (int)(T2 * MaxWave));
        break;
      }
      case 2:
      {
        Hotend2.setText(buffer);
        Hotend2.setColor(color);
        Graph2.addValue(0, (int)(T1 * MaxWave));
        Graph2.addValue(1, (int)(T2 * MaxWave));
        break;
      }
    }
  }

  static void coordtoLCD() {
    char *valuetemp;

    memset(buffer, 0, sizeof(buffer));
    strcat(buffer, "X");
    if (axis_known_position[X_AXIS]) {
      #if MECH(DELTA)
        valuetemp = ftostr30(current_position[X_AXIS]);
      #else
        valuetemp = ftostr3(current_position[X_AXIS]);
      #endif
      strcat(buffer, valuetemp);
    }
    else
      strcat(buffer, "---");

    strcat(buffer, " Y");
    if (axis_known_position[Y_AXIS]) {
      #if MECH(DELTA)
        valuetemp = ftostr30(current_position[Y_AXIS]);
      #else
        valuetemp = ftostr3(current_position[Y_AXIS]);
      #endif
      strcat(buffer, valuetemp);
    }
    else
      strcat(buffer, "---");
    
    strcat(buffer, " Z");
    if (axis_known_position[Z_AXIS]) {
      valuetemp = ftostr32sp(current_position[Z_AXIS] + 0.00001);
      strcat(buffer, valuetemp);
    }
    else
      strcat(buffer, "---");
    
    LedCoord.setText(buffer);
  }

  void lcd_update() {

    if (!NextionON) return;

    nexLoop(nex_listen_list);

    millis_t ms = millis();

    if (ms > next_lcd_update_ms && PageInfo) {

      if (fanSpeed > 0) fantimer.enable();
      else fantimer.disable();

      #if HAS_TEMP_0
        temptoLCD(0, degHotend(0), degTargetHotend(0));
      #endif
      #if HAS_TEMP_1
        temptoLCD(1, degHotend(1), degTargetHotend(1));
      #endif
      #if HAS_TEMP_2
        temptoLCD(2, degHotend(2), degTargetHotend(2));
      #elif HAS_TEMP_BED
        temptoLCD(2, degBed(), degTargetBed());
      #endif

      coordtoLCD();

      #if ENABLED(SDSUPPORT)
        if (card.cardOK)
          MSD.setPic(7);
        else
          MSD.setPic(6);
      #endif

      next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;
      return_to_status_ms = ms + 30000;
    } else if (ms > return_to_status_ms && !PageInfo) {
      setpageInfo();
    }
  }

  void lcd_setstatus(const char* message, bool persist) {
    if (lcd_status_message_level > 0 || !NextionON) return;
    strncpy(lcd_status_message, message, 30);
    LedStatus.setText(lcd_status_message);
  }

  void lcd_setstatuspgm(const char* message, uint8_t level) {
    if (level >= lcd_status_message_level && NextionON) {
      strncpy_P(lcd_status_message, message, 30);
      lcd_status_message_level = level;
      LedStatus.setText(lcd_status_message);
    }
  }

  void lcd_setalertstatuspgm(const char* message) {
    lcd_setstatuspgm(message, 1);
  }

  void lcd_reset_alert_level() { lcd_status_message_level = 0; }

  /*********************************/
  /** Number to string conversion **/
  /*********************************/

  char conv[8];

  // Convert float to string with +123.4 format
  char *ftostr3(const float &x) {
    return itostr3((int)x);
  }

  // Convert int to string with 12 format
  char *itostr2(const uint8_t &x) {
    //sprintf(conv,"%5.1f",x);
    int xx = x;
    conv[0] = (xx / 10) % 10 + '0';
    conv[1] = xx % 10 + '0';
    conv[2] = 0;
    return conv;
  }

  // Convert float to string with +123.4 format
  char *ftostr31(const float &x) {
    int xx = abs(x * 10);
    conv[0] = (x >= 0) ? '+' : '-';
    conv[1] = (xx / 1000) % 10 + '0';
    conv[2] = (xx / 100) % 10 + '0';
    conv[3] = (xx / 10) % 10 + '0';
    conv[4] = '.';
    conv[5] = xx % 10 + '0';
    conv[6] = 0;
    return conv;
  }

  // Convert float to string with 123.4 format, dropping sign
  char *ftostr31ns(const float &x) {
    int xx = abs(x * 10);
    conv[0] = (xx / 1000) % 10 + '0';
    conv[1] = (xx / 100) % 10 + '0';
    conv[2] = (xx / 10) % 10 + '0';
    conv[3] = '.';
    conv[4] = xx % 10 + '0';
    conv[5] = 0;
    return conv;
  }

  // Convert float to string with 123.4 format
  char *ftostr32(const float &x) {
    long xx = abs(x * 100);
    conv[0] = x >= 0 ? (xx / 10000) % 10 + '0' : '-';
    conv[1] = (xx / 1000) % 10 + '0';
    conv[2] = (xx / 100) % 10 + '0';
    conv[3] = '.';
    conv[4] = (xx / 10) % 10 + '0';
    conv[5] = xx % 10 + '0';
    conv[6] = 0;
    return conv;
  }

  // Convert float to string with 1.234 format
  char *ftostr43(const float &x) {
    long xx = x * 1000;
    if (xx >= 0) {
      conv[0] = (xx / 1000) % 10 + '0';
    }
    else {
      conv[0] = '-';
    }
    xx = abs(xx);
    conv[1] = '.';
    conv[2] = (xx / 100) % 10 + '0';
    conv[3] = (xx / 10) % 10 + '0';
    conv[4] = (xx) % 10 + '0';
    conv[5] = 0;
    return conv;
  }

  // Convert float to string with 1.23 format
  char *ftostr12ns(const float &x) {
    long xx=x*100;
    
    xx=abs(xx);
    conv[0]=(xx/100)%10+'0';
    conv[1]='.';
    conv[2]=(xx/10)%10+'0';
    conv[3]=(xx)%10+'0';
    conv[4]=0;
    return conv;
  }

  // Convert float to space-padded string with -_23.4_ format
  char *ftostr32sp(const float &x) {
    long xx = abs(x * 100);
    uint8_t dig;

    if (x < 0) { // negative val = -_0
      conv[0] = '-';
      dig = (xx / 1000) % 10;
      conv[1] = dig ? '0' + dig : ' ';
    }
    else { // positive val = __0
      dig = (xx / 10000) % 10;
      if (dig) {
        conv[0] = '0' + dig;
        conv[1] = '0' + (xx / 1000) % 10;
      }
      else {
        conv[0] = ' ';
        dig = (xx / 1000) % 10;
        conv[1] = dig ? '0' + dig : ' ';
      }
    }

    conv[2] = '0' + (xx / 100) % 10; // lsd always

    dig = xx % 10;
    if (dig) { // 2 decimal places
      conv[5] = '0' + dig;
      conv[4] = '0' + (xx / 10) % 10;
      conv[3] = '.';
    }
    else { // 1 or 0 decimal place
      dig = (xx / 10) % 10;
      if (dig) {
        conv[4] = '0' + dig;
        conv[3] = '.';
      }
      else {
        conv[3] = conv[4] = ' ';
      }
      conv[5] = ' ';
    }
    conv[6] = '\0';
    return conv;
  }

  // Convert int to lj string with +123.0 format
  char *itostr31(const int &x) {
    conv[0] = x >= 0 ? '+' : '-';
    int xx = abs(x);
    conv[1] = (xx / 100) % 10 + '0';
    conv[2] = (xx / 10) % 10 + '0';
    conv[3] = xx % 10 + '0';
    conv[4] = '.';
    conv[5] = '0';
    conv[6] = 0;
    return conv;
  }

  // Convert int to rj string with 123 or -12 format
  char *itostr3(const int &x) {
    int xx = x;
    if (xx < 0) {
       conv[0] = '-';
       xx = -xx;
    }
    else
      conv[0] = xx >= 100 ? (xx / 100) % 10 + '0' : ' ';

    conv[1] = xx >= 10 ? (xx / 10) % 10 + '0' : ' ';
    conv[2] = xx % 10 + '0';
    conv[3] = 0;
    return conv;
  }

  // Convert int to lj string with 123 format
  char *itostr3left(const int &xx) {
    if (xx >= 100) {
      conv[0] = (xx / 100) % 10 + '0';
      conv[1] = (xx / 10) % 10 + '0';
      conv[2] = xx % 10 + '0';
      conv[3] = 0;
    }
    else if (xx >= 10) {
      conv[0] = (xx / 10) % 10 + '0';
      conv[1] = xx % 10 + '0';
      conv[2] = 0;
    }
    else {
      conv[0] = xx % 10 + '0';
      conv[1] = 0;
    }
    return conv;
  }

  // Convert int to rj string with 1234 format
  char *itostr4(const int &xx) {
    conv[0] = xx >= 1000 ? (xx / 1000) % 10 + '0' : ' ';
    conv[1] = xx >= 100 ? (xx / 100) % 10 + '0' : ' ';
    conv[2] = xx >= 10 ? (xx / 10) % 10 + '0' : ' ';
    conv[3] = xx % 10 + '0';
    conv[4] = 0;
    return conv;
  }

  char *ltostr7(const long &xx) {
    if (xx >= 1000000)
      conv[0]=(xx/1000000)%10+'0';
    else
      conv[0]=' ';
    if (xx >= 100000)
      conv[1]=(xx/100000)%10+'0';
    else
      conv[1]=' ';
    if (xx >= 10000)
      conv[2]=(xx/10000)%10+'0';
    else
      conv[2]=' ';
    if (xx >= 1000)
      conv[3]=(xx/1000)%10+'0';
    else
      conv[3]=' ';
    if (xx >= 100)
      conv[4]=(xx/100)%10+'0';
    else
      conv[4]=' ';
    if (xx >= 10)
      conv[5]=(xx/10)%10+'0';
    else
      conv[5]=' ';
    conv[6]=(xx)%10+'0';
    conv[7]=0;
    return conv;
  }

  // convert float to string with +123 format
  char *ftostr30(const float &x) {
    int xx=x;
    conv[0]=(xx>=0)?'+':'-';
    xx=abs(xx);
    conv[1]=(xx/100)%10+'0';
    conv[2]=(xx/10)%10+'0';
    conv[3]=(xx)%10+'0';
    conv[4]=0;
    return conv;
  }

  // Convert float to rj string with 12345 format
  char *ftostr5(const float &x) {
    long xx = abs(x);
    conv[0] = xx >= 10000 ? (xx / 10000) % 10 + '0' : ' ';
    conv[1] = xx >= 1000 ? (xx / 1000) % 10 + '0' : ' ';
    conv[2] = xx >= 100 ? (xx / 100) % 10 + '0' : ' ';
    conv[3] = xx >= 10 ? (xx / 10) % 10 + '0' : ' ';
    conv[4] = xx % 10 + '0';
    conv[5] = 0;
    return conv;
  }

  // Convert float to string with +1234.5 format
  char *ftostr51(const float &x) {
    long xx = abs(x * 10);
    conv[0] = (x >= 0) ? '+' : '-';
    conv[1] = (xx / 10000) % 10 + '0';
    conv[2] = (xx / 1000) % 10 + '0';
    conv[3] = (xx / 100) % 10 + '0';
    conv[4] = (xx / 10) % 10 + '0';
    conv[5] = '.';
    conv[6] = xx % 10 + '0';
    conv[7] = 0;
    return conv;
  }

  // Convert float to string with +123.45 format
  char *ftostr52(const float &x) {
    conv[0] = (x >= 0) ? '+' : '-';
    long xx = abs(x * 100);
    conv[1] = (xx / 10000) % 10 + '0';
    conv[2] = (xx / 1000) % 10 + '0';
    conv[3] = (xx / 100) % 10 + '0';
    conv[4] = '.';
    conv[5] = (xx / 10) % 10 + '0';
    conv[6] = xx % 10 + '0';
    conv[7] = 0;
    return conv;
  }
#endif
