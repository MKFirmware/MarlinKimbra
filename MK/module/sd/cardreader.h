#ifndef CARDREADER_H
#define CARDREADER_H

#if ENABLED(SDSUPPORT)

#define SD_MAX_FOLDER_DEPTH 5     // Maximum folder depth
#define MAX_VFAT_ENTRIES (2)
#define FILENAME_LENGTH 13
/** Total size of the buffer used to store the long filenames */
#define LONG_FILENAME_LENGTH (FILENAME_LENGTH * MAX_VFAT_ENTRIES + 1)
#define SHORT_FILENAME_LENGTH 14

extern char tempLongFilename[LONG_FILENAME_LENGTH + 1];
extern char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];

#include "SdFat.h"

class CardReader {
public:
  SdFat fat;
  //Sd2Card card; // ~14 Byte
  //SdVolume volume;
  //SdFile root;
  //SdFile dir[SD_MAX_FOLDER_DEPTH+1];
  SdFile file;
  CardReader();

  void initsd();
  void mount();
  void unmount();
  void ls();
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
  void updateSDFileCount();
  void chdir(const char* name);

  void parseKeyLine(char* key, char* value, int &len_k, int &len_v);
  void unparseKeyLine(const char* key, char* value);

  FORCE_INLINE void setIndex(uint32_t newpos) { sdpos = newpos; file.seekSet(sdpos); }
  FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
  FORCE_INLINE bool eof() { return sdpos >= filesize; }
  FORCE_INLINE void updir() { chdir(".."); }
  FORCE_INLINE int16_t get() { sdpos = file.curPosition(); return (int16_t)file.read(); }
  FORCE_INLINE uint8_t percentDone() { return (isFileOpen() && filesize) ? sdpos / ((filesize + 99) / 100) : 0; }

  //files init.g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset
  void checkautostart(bool x);

public:
  bool saving, sdprinting, cardOK, filenameIsDir;
  uint16_t nrFiles; // counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  uint8_t folderLevel;
  char cwd[SD_MAX_FOLDER_DEPTH*LONG_FILENAME_LENGTH+2];
private:
  SdFile root, *curDir, workDir;
  Sd2Card card;
  SdVolume volume;
  uint32_t filesize;
  millis_t next_autostart_ms;
  uint32_t sdpos;

  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
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
