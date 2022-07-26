class ButtonEvents {
  private:
    typedef enum {STATE_UP, STATE_DOWN, STATE_LONGPRESSED} transitionStates;  //internal state processing 
          
    struct Button {
      Button* next;
      
      byte pin;
      byte event_button_up;
      byte event_long_press;
      
      // debounce 
      byte stateHistory;          // bit pattern for previous input states
      bool debouncedButtonState;

      // transition handling - button_up, long_press
      transitionStates transitionState;
      int_fast32_t buttonPressedDuration;
    };
    
    Button* next = NULL; // NULL terminated linked list of Buttons
    int noChangeEvent = 0;
    int_fast32_t nextButtonPoll = 0;

    int debounce(Button* b) {
      // debounce algorithm insprired by Listing 2 at https://www.embedded.com/my-favorite-software-debouncers/
      // regard a button's input as valid only when it is stable on 4 consecutive samples
      int buttonPosition = digitalRead(b->pin);
      b->stateHistory = ((b->stateHistory<<1) | buttonPosition) & 0x0f;  
      if (b->stateHistory == 0x0) {b->debouncedButtonState = true;}; 
      if (b->stateHistory == 0xf) {b->debouncedButtonState = false;};
      return b->debouncedButtonState;
    }

    int getButtonEvent(Button* b) {
      // use a Finite State Machine to turn debounced inputs into events
      bool btn = debounce(b);
      int result = noChangeEvent;
      switch (b->transitionState) {
        case STATE_UP:
          if (btn) {
            b->transitionState = STATE_DOWN;
            b->buttonPressedDuration = millis() + 1000;
          }
          break;
        case STATE_DOWN:
          if (!btn) {
            b->transitionState = STATE_UP;
            result = b->event_button_up;
          }
          if (millis() > b->buttonPressedDuration) {
            b->transitionState = STATE_LONGPRESSED;
            result = b->event_long_press;
          }
          break;
        case STATE_LONGPRESSED:
          if (!btn) {
            b->transitionState = STATE_UP;
          }
          break;
      default:
          b->transitionState = STATE_UP;
      }
      return result;
    }
    
  public:
    ButtonEvents(int noChange) {
      noChangeEvent = noChange;
    };
    
    void add(uint8_t pinNo, int evt_button_up, int evt_long_press) {
      pinMode( pinNo, INPUT_PULLUP);
        
      // initialise the button structure
      Button* b = (Button*) malloc(sizeof(struct Button));
      //b = {next, pinNo, evt_button_up, evt_long_press, 0, false, STATE_UP, 0};
      b->next = next;
      b->pin = pinNo;
      b->event_button_up = evt_button_up;
      b->event_long_press = evt_long_press;
      b->stateHistory = 0;
      b->debouncedButtonState = false;
      b->transitionState = STATE_UP;
      b->buttonPressedDuration = 0;

      // add it to the head of the chain
      next = b;
    };


    // call this often in the main loop.
    int getButtonEvent() {
      // poll the button inputs no closer than every 10 ms
      int_fast32_t currentTime = millis();
      if (currentTime >= nextButtonPoll) {
        nextButtonPoll = currentTime + 10;  
        Button* b = next;
        while(b) {  //scan down the linked list
          int event = getButtonEvent(b);
          if (event != noChangeEvent) {
            return event; // return with the first key event found. Ignoring later keys is fine; they'll be picked on the next call
          }
          b = b->next;
        }
      }
      return noChangeEvent;
    }
};
