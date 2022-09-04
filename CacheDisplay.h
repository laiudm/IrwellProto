
// ------------ caching display ----------------------------------

// Track the characters written (incl foreground/background colours)
// and only actually write to the screen if it's different from what's already on display
// at those x, y coordinates.
// Ignore the font - require an area to use the same font.

// create a hashmap - the coordinates select the entry.
#define ENTRIES 257

typedef struct entry {
  uint8_t x, y;       // coordinates of character
  char ch;            // the entry is empty if ch == 0;
  //uint8_t fr, fg, fb; // foreground colours. Do I need to track colours?
  //uint8_t br, bg, bb; // background colours
} Entry;

typedef struct hashmap {
  Entry entries[ENTRIES]; // table of entries
  unsigned int len;       // number of entries filled
} Hashmap;

Hashmap hash;

void clearCache() {
  memset(&hash, 0, sizeof(hash));
}

uint16_t hashcode(uint8_t x, uint8_t y) {
  uint16_t result = ((x+1) * (y+1)) % ENTRIES; // add 1 to avoid 0 on x or y axis crashing the hash
  //uint16_t result = (x*256 + y) % ENTRIES;
  //uint16_t result = (x*211 + y*293) % ENTRIES;
  return result;
}

// updates an entry or creates if it can't be found. Returns false on no match or creation; returns true if already exists and matches 
bool matchAndSet(uint8_t x, uint8_t y, char ch) {
  if (hash.len >= ENTRIES-1) {
    traceError(">>>>>>Hashmap set() failed. Hashtable full %i", hash.len);
  }
  for (int index = hashcode(x, y); ;index = (index+1)%ENTRIES) { 
    Entry* current = &hash.entries[index];
    if ((current->x == x) && (current->y == y)) {
      // Found the matching entry; check if there's a character change
      bool result = current->ch == ch;
      traceDisplay("matchAndSet: found match at %i - (%i, %i)'%c', %i", index, x, y, ch, result);
      current->ch = ch;
      return result;
    }
    if ((current->x == 0) && (current->y == 0)) {
      // found an empty slot. So these coords haven't been found; save it here
      hash.len++;
      traceDisplay("matchAndSet: no match; adding at %i - (%i, %i)'%c'", index, x, y, ch);
      current->x = x;
      current->y = y;
      current->ch = ch;
      return false;
    }
  }
}

// Wrap the standard display & intercept the screen writes.
// Only write to the screen if the character has changed.
// This massively improves the screen responsiveness and massively
// simplifies the screen writing code

class Display : public Ucglib_ILI9341_18x240x320_HWSPI {
  typedef Ucglib_ILI9341_18x240x320_HWSPI super;

public:
  ucg_fntpgm_uint8_t *lastFont = NULL;
  ucg_int_t w = 0;
  
  Display( uint8_t cd, uint8_t cs = UCG_PIN_VAL_NONE, uint8_t reset = UCG_PIN_VAL_NONE) :
    super(cd, cs, reset)
    { };

  void clearScreen() {
    clearCache();
    super::clearScreen();
  }
  
  void setFont(const ucg_fntpgm_uint8_t  *font) {
    // Memoising the setFont() and getStrWidth() calls doubles! updateScreen() execution speed.
    // These few lines of code are well worth it
    if (font != lastFont) {
      super::setFont(font);
      w = getStrWidth(" ");
      lastFont = (ucg_fntpgm_uint8_t  *)font;
    }
  }

  size_t write(uint8_t c) {
    ucg_int_t x = get_tx();
    ucg_int_t y = get_ty(); 
     bool matched = matchAndSet(x, y, c);
    traceDisplay("write of '%c' to (%i,%i). Matched: %i", c, get_tx(), get_ty(), matched);
    if (!matched) {
      super::write(c);
    }
    setPrintPos(x+w, y); // monofonts are glitched, hack to fixed width. Just a pixel out
  }
  
};
