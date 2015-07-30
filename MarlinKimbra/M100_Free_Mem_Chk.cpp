#define M100_FREE_MEMORY_DUMPER       // Comment out to remove Dump sub-command
#define M100_FREE_MEMORY_CORRUPTOR    // Comment out to remove Corrupt sub-command


// M100 Free Memory Watcher
//
// This code watches the free memory block between the bottom of the heap and the top of the stack.
// This memory block is initialized and watched via the M100 command.
//
// M100 I Initializes the free memory block and prints vitals statistics about the area
// M100 F Identifies how much of the free memory block remains free and unused.  It also
//    detects and reports any corruption within the free memory block that may have
//    happened due to errant firmware.
// M100 D Does a hex display of the free memory block along with a flag for any errant
//    data that does not match the expected value.
// M100 C x Corrupts x locations within the free memory block.   This is useful to check the
//    correctness of the M100 F and M100 D commands.
//
// Initial version by Roxy-3DPrintBoard
//
//


#include "Marlin.h"

#ifdef M100_FREE_MEMORY_WATCHER
  extern void *__brkval;
  extern size_t  __heap_start, __heap_end, __flp;

  //
  // Declare all the functions we need from Marlin_Main.cpp to do the work!
  //
  float code_value();
  long code_value_long();
  bool code_seen(char );


  //
  // Utility functions used by M100 to get its work done.
  //
  unsigned char *top_of_stack();
  void prt_hex_nibble( unsigned int );
  void prt_hex_byte(unsigned int );
  void prt_hex_word(unsigned int );
  int how_many_E5s_are_here( unsigned char *);


  void gcode_M100() {
    static int m100_not_initialized = 1;
    unsigned char *sp, *ptr;
    int i, j, n;

    //
    // M100 D dumps the free memory block from __brkval to the stack pointer.
    // malloc() eats memory from the start of the block and the stack grows
    // up from the bottom of the block.    Solid 0xE5's indicate nothing has
    // used that memory yet.   There should not be anything but 0xE5's within
    // the block of 0xE5's.  If there is, that would indicate memory corruption
    // probably caused by bad pointers.  Any unexpected values will be flagged in
    // the right hand column to help spotting them.
    //
    #ifdef M100_FREE_MEMORY_DUMPER      // Comment out to remove Dump sub-command
      if ( code_seen('D') ) {
        ptr = (unsigned char *) __brkval;

        //
        // We want to start and end the dump on a nice 16 byte boundry even though
        // the values we are using are not 16 byte aligned.
        //
        ECHO_M("\n__brkval : ");
        prt_hex_word( (unsigned int) ptr );
        ptr = (unsigned char *) ((unsigned long) ptr & 0xfff0);

        sp = top_of_stack();
        ECHO_M("\nStack Pointer : ");
        prt_hex_word( (unsigned int) sp );
        ECHO_M("\n");

        sp = (unsigned char *) ((unsigned long) sp | 0x000f);
        n = sp - ptr;

        //
        // This is the main loop of the Dump command.
        //
        while ( ptr < sp ) {
          prt_hex_word( (unsigned int) ptr);  // Print the address
          ECHO_M(":");
          for(i = 0; i < 16; i++) {     // and 16 data bytes
            prt_hex_byte( *(ptr+i));
            ECHO_M(" ");
            delay(2);
          }

          ECHO_M("|");        // now show where non 0xE5's are
          for(i = 0; i < 16; i++) {
            delay(2);
            if ( *(ptr+i)==0xe5)
              ECHO_M(" ");
            else
              ECHO_M("?");
          }
          ECHO_M("\n");

          ptr += 16;
          delay(2);
        }
        ECHO_M("Done.\n");
        return;
      }
    #endif

    //
    // M100 F   requests the code to return the number of free bytes in the memory pool along with
    // other vital statistics that define the memory pool.
    //
    if ( code_seen('F') ) {
      int max_addr = (int) __brkval;
      int max_cnt = 0;
      int block_cnt = 0;
      ptr = (unsigned char *) __brkval;
      sp = top_of_stack();
      n = sp - ptr;

      // Scan through the range looking for the biggest block of 0xE5's we can find

      for(i = 0; i < n; i++) {
        if ( *(ptr+i) == (unsigned char) 0xe5) {
          j = how_many_E5s_are_here( (unsigned char *) ptr+i );
          if ( j > 8) {
            ECHO_MV("Found ", j );
            ECHO_M(" bytes free at 0x");
            prt_hex_word( (int) ptr+i );
            ECHO_M("\n");
            i += j;
            block_cnt++;
          }
          if ( j>max_cnt) {     // We don't do anything with this information yet
            max_cnt  = j;     // but we do know where the biggest free memory block is.
            max_addr = (int) ptr+i;
          }
        }
      }
      if (block_cnt>1)
          ECHO_EM("\nMemory Corruption detected in free memory area.\n");

      ECHO_M("\nDone.\n");
      return;
    }

    //
    // M100 C x  Corrupts x locations in the free memory pool and reports the locations of the corruption.
    // This is useful to check the correctness of the M100 D and the M100 F commands.
    //
    #ifdef M100_FREE_MEMORY_CORRUPTOR
      if ( code_seen('C') ) {
        int x;      // x gets the # of locations to corrupt within the memory pool
        x = code_value();
        ECHO_EM("Corrupting free memory block.\n");
        ptr = (unsigned char *) __brkval;
        ECHO_MV("\n__brkval : ",(long) ptr );
        ptr += 8;

        sp = top_of_stack();
        ECHO_MV("\nStack Pointer : ",(long) sp );
        ECHO_EM("\n");

        n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that
                              // has altered the stack.
        j = n / (x+1);
        for(i = 1; i <= x; i++) {
          *(ptr+(i*j)) = i;
          ECHO_M("\nCorrupting address: 0x");
          prt_hex_word( (unsigned int)  (ptr+(i*j)) );
        }
        ECHO_EM("\n");
        return;
      }
    #endif

    //
    // M100 I    Initializes the free memory pool so it can be watched and prints vital
    // statistics that define the free memory pool.
    //
    if (m100_not_initialized || code_seen('I') ) {        // If no sub-command is specified, the first time
      ECHO_EM("Initializing free memory block.\n");       // this happens, it will Initialize.
      ptr = (unsigned char *) __brkval;         // Repeated M100 with no sub-command will not destroy the
      ECHO_MV("\n__brkval : ",(long) ptr );     // state of the initialized free memory pool.
      ptr += 8;

      sp = top_of_stack();
      ECHO_MV("\nStack Pointer : ",(long) sp );
      ECHO_EM("\n");

      n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that
                            // has altered the stack.

      ECHO_V( n );
      ECHO_EM(" bytes of memory initialized.\n");

      for(i = 0; i < n; i++)
        *(ptr+i) = (unsigned char) 0xe5;

      for(i = 0; i < n; i++) {
        if ( *(ptr+i) != (unsigned char) 0xe5 ) {
          ECHO_MV("? address : ", (unsigned long) ptr+i );
          ECHO_MV("=", *(ptr+i) );
          ECHO_EM("\n");
        }
      }
      m100_not_initialized = 0;
      ECHO_EM("Done.\n");
      return;
    }
    return;
  }

  // top_of_stack() returns the location of a variable on its stack frame.  The value returned is above
  // the stack once the function returns to the caller.

  unsigned char *top_of_stack() {
    unsigned char x;
    return &x + 1; // x is pulled on return;
  }

  //
  // 3 support routines to print hex numbers.  We can print a nibble, byte and word
  //
  void prt_hex_nibble( unsigned int n ) {
    if ( n <= 9 )
      ECHO_V(n);
    else
      ECHO_V( (char) ('A'+n-10) );
    delay(2);
  }

  void prt_hex_byte(unsigned int b) {
    prt_hex_nibble( ( b & 0xf0 ) >> 4 );
    prt_hex_nibble(  b & 0x0f );
  }

  void prt_hex_word(unsigned int w) {
    prt_hex_byte( ( w & 0xff00 ) >> 8 );
    prt_hex_byte(  w & 0x0ff );
  }

  // how_many_E5s_are_here() is a utility function to easily find out how many 0xE5's are
  // at the specified location.  Having this logic as a function simplifies the search code.
  //
  int how_many_E5s_are_here( unsigned char *p) {
    int n;

    for(n = 0; n < 32000; n++) {
      if ( *(p+n) != (unsigned char) 0xe5)
        return n-1;
    }
    return -1;
  }

#endif
