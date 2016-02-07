#include "../../base.h"

#if ENABLED(SDSUPPORT)

#include "cardreader.h"

char tempLongFilename[LONG_FILENAME_LENGTH + 1];
char fullName[LONG_FILENAME_LENGTH * SD_MAX_FOLDER_DEPTH + SD_MAX_FOLDER_DEPTH + 1];

CardReader::CardReader() {
  filesize = 0;
  sdpos = 0;
  sdprinting = false;
  cardOK = false;
  saving = false;
  autostart_stilltocheck = true; //the SD start is delayed, because otherwise the serial cannot answer fast enough to make contact with the host software.
  folderLevel = 0;

  //power to SD reader
  #if SDPOWER > -1
    OUT_WRITE(SDPOWER, HIGH);
  #endif // SDPOWER

  next_autostart_ms = millis() + SPLASH_SCREEN_DURATION;
}

void CardReader::initsd() {
  cardOK = false;

  #if ENABLED(SDEXTRASLOW)
    #define SPI_SPEED SPI_QUARTER_SPEED
  #elif ENABLED(SDSLOW)
    #define SPI_SPEED SPI_HALF_SPEED
  #else
    #define SPI_SPEED SPI_FULL_SPEED
  #endif

  if(!fat.begin(SDSS, SPI_SPEED)
    #if defined(LCD_SDSS) && (LCD_SDSS != SDSS)
      && !card.init(SPI_SPEED, LCD_SDSS)
    #endif
  ) {
    ECHO_LM(ER, SERIAL_SD_INIT_FAIL);
  }
  else {
    cardOK = true;
    ECHO_LM(DB, SERIAL_SD_CARD_OK);
  }
  fat.chdir();
  workDir = root;
}

void CardReader::mount() {
  initsd();
}

void CardReader::unmount() {
  cardOK = false;
  sdprinting = false;
  folderLevel = 0;
}

void CardReader::startPrint() {
  if (cardOK) sdprinting = true;
}

void CardReader::pausePrint() {
  if (sdprinting) sdprinting = false;
}

void CardReader::continuePrint(bool intern) {}

void stopPrint() {}

void CardReader::write_command(char* buf) {
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;

  file.writeError = false;
  if ((npos = strchr(buf, 'N')) != NULL) {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  file.write(begin);
  if (file.writeError) {
    ECHO_LM(ER, SERIAL_SD_ERR_WRITE_TO_FILE);
  }
}

bool CardReader::selectFile(const char* filename, bool silent/*=false*/) {
  SdBaseFile parent;
  const char *oldP = filename;

  if(!cardOK) return false;

  file.close();

  parent = *fat.vwd();
  if (file.open(&parent, filename, O_READ)) {
    if ((oldP = strrchr(filename, '/')) != NULL)
      oldP++;
    else
      oldP = filename;
    if(!silent) {
      ECHO_MT(SERIAL_SD_FILE_OPENED, oldP);
      ECHO_EMV(SERIAL_SD_SIZE, file.fileSize());
    }
    sdpos = 0;
    filesize = file.fileSize();
    ECHO_EM(SERIAL_SD_FILE_SELECTED);
    return true;
  }
  else {
    if(!silent) ECHO_EMT(SERIAL_SD_OPEN_FILE_FAIL, oldP);
    return false;
  }
}

char* CardReader::createFilename(char* buffer, const dir_t& p) { //buffer > 12characters
  char* pos = buffer, *src = (char*)p.name;
  for (uint8_t i = 0; i < 11; i++, src++) {
    if (*src == ' ') continue;
    if (i == 8) *pos++ = '.';
    *pos++ = *src;
  }
  *pos = 0;
  return pos;
}

void CardReader::printStatus() {
  if (cardOK) {
    ECHO_MV(SERIAL_SD_PRINTING_BYTE, sdpos);
    ECHO_EMV(SERIAL_SD_SLASH, filesize);
  }
  else
    ECHO_EM(SERIAL_SD_NOT_PRINTING);
}

void CardReader::startWrite(char *filename, bool lcd_status/*=true*/) {
  if(!cardOK) return;
  file.close();
  fat.chdir();

  if(!file.open(filename, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
    ECHO_LMT(ER, SERIAL_SD_OPEN_FILE_FAIL, filename);
  }
  else {
    saving = true;
    ECHO_EMT(SERIAL_SD_WRITE_TO_FILE, filename);
    if (lcd_status) lcd_setstatus(filename);
  }
}

void CardReader::deleteFile(char *filename) {
  if(!cardOK) return;
  sdprinting = false;
  file.close();
  if(fat.remove(filename)) {
    ECHO_EMT(SERIAL_SD_FILE_DELETED, filename);
  }
  else {
    if(fat.rmdir(filename))
      ECHO_EMT(SERIAL_SD_FILE_DELETED, filename);
    else
      ECHO_EMT(SERIAL_SD_FILE_DELETION_ERR, filename);
  }
}

void CardReader::finishWrite() {
    if(!saving) return; // already closed or never opened
    file.sync();
    file.close();
    saving = false;
    ECHO_EM(SERIAL_SD_FILE_SAVED);
}

void CardReader::makeDirectory(char *filename) {
  if(!cardOK) return;
  sdprinting = false;
  file.close();
  if(fat.mkdir(filename)) {
    ECHO_EM(SERIAL_SD_DIRECTORY_CREATED);
  }
  else {
    ECHO_EM(SERIAL_SD_CREATION_FAILED);
  }
}

void CardReader::ls()  {
  SdBaseFile file;
  fat.chdir();
  file.openRoot(fat.vol());
  file.ls(0, 0);
  updateSDFileCount();
}

void CardReader::closeFile(bool store_location) {
  file.sync();
  file.close();
  saving = false;

  if (store_location) {
    //future: store printer state, filename and position for continuing a stopped print
    // so one can unplug the printer and continue printing the next day.
  }
}

void CardReader::checkautostart(bool force) {
  if (!force && (!autostart_stilltocheck || next_autostart_ms >= millis()))
    return;

  autostart_stilltocheck = false;

  if (!cardOK) {
    initsd();
    if (!cardOK) return; // fail
  }

  fat.chdir();
  if(selectFile("init.g", true)) startPrint();
}

void CardReader::printingHasFinished() {
  st_synchronize();
  file.close();
  sdprinting = false;
  if (SD_FINISHED_STEPPERRELEASE) {
    //finishAndDisableSteppers();
    enqueuecommands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
  }
  autotempShutdown();
}

void CardReader::updateSDFileCount() {
  dir_t* p = NULL;
  SdBaseFile *root = fat.vwd();

  root->rewind();
  nrFiles = 0;
  while ((p = root->getLongFilename(p, NULL, 0, NULL))) {
    if (! (DIR_IS_FILE(p) || DIR_IS_SUBDIR(p)))
      continue;
    if (folderLevel >= SD_MAX_FOLDER_DEPTH && DIR_IS_SUBDIR(p) && !(p->name[0]=='.' && p->name[1]=='.'))
      continue;
    nrFiles++;
    if (nrFiles > 5000) // Arbitrary maximum, limited only by how long someone would scroll
      return;
  }
}

void CardReader::chdir(const char* name) {
  bool changedir = false;

  if (name[0]=='.' && name[1]=='.')
    changedir = fat.chdir("/", true);
  else
    changedir = fat.chdir(name, true);

  if (!changedir)
    ECHO_LMT(DB, SERIAL_SD_CANT_ENTER_SUBDIR, name);
  else
    updateSDFileCount();
}

int8_t RFstricmp(const char* s1, const char* s2) {
  while(*s1 && (tolower(*s1) == tolower(*s2)))
    s1++, s2++;
  return (const uint8_t)tolower(*s1) - (const uint8_t)tolower(*s2);
}

int8_t RFstrnicmp(const char* s1, const char* s2, size_t n) {
  while(n--) {
    if(tolower(*s1) != tolower(*s2))
      return (uint8_t)tolower(*s1) - (uint8_t)tolower(*s2);
    s1++;
    s2++;
  }
  return 0;
}

/**
 * File parser for KEY->VALUE format from files
 *
 * Author: Simone Primarosa
 *
 */
void CardReader::parseKeyLine(char* key, char* value, int &len_k, int &len_v) {
  if (!cardOK || !isFileOpen()) {
    key[0] = value[0] = '\0';
    len_k = len_v = 0;
    return;
  }
  int ln_buf = 0;
  char ln_char;
  bool ln_space = false, ln_ignore = false, key_found = false;
  while(!eof()) {   //READ KEY
    ln_char = (char)get();
    if(ln_char == '\n') {
      ln_buf = 0;
      ln_ignore = false;  //We've reached a new line try to find a key again
      continue;
    }
    if(ln_ignore) continue;
    if(ln_char == ' ') {
      ln_space = true;
      continue;
    }
    if(ln_char == '=') {
      key[ln_buf] = '\0';
      len_k = ln_buf;
      key_found = true;
      break; //key finded and buffered
    }
    if(ln_char == ';' || ln_buf+1 >= len_k || ln_space && ln_buf > 0) { //comments on key is not allowd. Also key len can't be longer than len_k or contain spaces. Stop buffering and try the next line
      ln_ignore = true;
      continue;
    }
    ln_space = false;
    key[ln_buf] = ln_char;
    ln_buf++;
  }
  if(!key_found) { //definitly there isn't no more key that can be readed in the file
    key[0] = value[0] = '\0';
    len_k = len_v = 0;
    return;
  }
  ln_buf = 0;
  ln_ignore = false;
  while(!eof()) {   //READ VALUE
    ln_char = (char)get();
    if(ln_char == '\n') {
      value[ln_buf] = '\0';
      len_v = ln_buf;
      break;  //new line reached, we can stop
    }
    if(ln_ignore|| ln_char == ' ' && ln_buf == 0) continue; //ignore also initial spaces of the value
    if(ln_char == ';' || ln_buf+1 >= len_v) { //comments reached or value len longer than len_v. Stop buffering and go to the next line.
      ln_ignore = true;
      continue;
    }
    value[ln_buf] = ln_char;
    ln_buf++;
  }
}

void CardReader::unparseKeyLine(const char* key, char* value) {
  if (!cardOK || !isFileOpen()) return;
  file.writeError = false;
  file.write(key);
  if (file.writeError) {
    ECHO_LM(ER, SERIAL_SD_ERR_WRITE_TO_FILE);
    return;
  }
  
  file.writeError = false;
  file.write("=");
  if (file.writeError) {
    ECHO_LM(ER, SERIAL_SD_ERR_WRITE_TO_FILE);
    return;
  }
  
  file.writeError = false;
  file.write(value);
  if (file.writeError) {
    ECHO_LM(ER, SERIAL_SD_ERR_WRITE_TO_FILE);
    return;
  }

  file.writeError = false;
  file.write("\n");
  if (file.writeError) {
    ECHO_LM(ER, SERIAL_SD_ERR_WRITE_TO_FILE);
    return;
  }
}

#endif //SDSUPPORT
