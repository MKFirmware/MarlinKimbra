#ifndef CARDREADER_H
#define CARDREADER_H

#if ENABLED(SDSUPPORT)

#define SD_MAX_FOLDER_DEPTH 10     // Maximum folder depth
#define MAX_VFAT_ENTRIES (2)
#define FILENAME_LENGTH 13
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (FILENAME_LENGTH * MAX_VFAT_ENTRIES + 1)
#define SHORT_FILENAME_LENGTH 14

extern char tempLongFilename[LONG_FILENAME_LENGTH + 1];
extern char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];

enum LsAction { LS_Count, LS_GetFilename };

#include "SdFat.h"

class CardReader {
public:
  SdFat fat;
  SdFile file;
  CardReader();

  void initsd();
  void mount();
  void unmount();
  void ls();
  void getfilename(uint16_t nr, const char* const match = NULL);
  void startPrint();
  void pausePrint();
  void continuePrint(bool intern = false);
  void stopPrint();
  void write_command(char* buf);
  bool selectFile(const char *filename, bool silent = false);
  void printStatus();
  void startWrite(char* filename, bool lcd_status = true);
  void deleteFile(char* filename);
  void finishWrite();
  void makeDirectory(char* filename);
  void closeFile(bool store_location = false);
  char *createFilename(char *buffer, const dir_t &p);
  void printingHasFinished();
  void chdir(const char* relpath);
  void updir();
  void setroot(bool temporary = false);
  void setlast();

  uint16_t getnrfilenames();

  void parseKeyLine(char* key, char* value, int &len_k, int &len_v);
  void unparseKeyLine(const char* key, char* value);

  FORCE_INLINE void setIndex(uint32_t newpos) { sdpos = newpos; file.seekSet(sdpos); }
  FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
  FORCE_INLINE bool eof() { return sdpos >= filesize; }
  FORCE_INLINE int16_t get() { sdpos = file.curPosition(); return (int16_t)file.read(); }
  FORCE_INLINE uint8_t percentDone() { return (isFileOpen() && filesize) ? sdpos / ((filesize + 99) / 100) : 0; }
  FORCE_INLINE char* getWorkDirName() { workDir.getFilename(fullName); return fullName; }

  //files init.g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset
  void checkautostart(bool x);

public:
  bool saving, sdprinting, cardOK, filenameIsDir;
private:
  SdBaseFile root, *curDir, workDir, lastDir, workDirParents[SD_MAX_FOLDER_DEPTH];
  Sd2Card card;
  uint16_t workDirDepth;
  uint32_t filesize;
  millis_t next_autostart_ms;
  uint32_t sdpos;
  uint16_t nrFiles; // counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  LsAction lsAction; //stored for recursion.
  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
  void lsDive(SdBaseFile parent, const char* const match = NULL);
};

extern CardReader card;

#define IS_SD_PRINTING (card.sdprinting)

#if PIN_EXISTS(SD_DETECT)
  #if ENABLED(SD_DETECT_INVERTED)
    #define IS_SD_INSERTED (READ(SD_DETECT_PIN) != 0)
  #else
    #define IS_SD_INSERTED (READ(SD_DETECT_PIN) == 0)
  #endif
#else
  //No card detect line? Assume the card is inserted.
  #define IS_SD_INSERTED true
#endif

#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT

#endif //__CARDREADER_H
