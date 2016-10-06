

// PINS
const int left  = 52;   // LEFT
const int right = 53;   // RIGHT
const int flag = 13;    // the number of the LED pin

// Variables will change:
int ledState = LOW;
int btnLeft;
int btnRight;
int lastbtnLeft = LOW;
int lastbtnRight = LOW;


unsigned long lastDebounceTime1 = 0;  
unsigned long debounceDelay1 = 50;

void setup() {
  pinMode(left, INPUT);
  pinMode(right, INPUT);
  
  pinMode(flag, OUTPUT);

  // set initial LED state
  digitalWrite(flag, ledState);
}

void loop() {
  int reading1 = digitalRead(left);
  int reading2 = digitalRead(right);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading1 != lastbtnLeft) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
  }

  if ((millis() - lastDebounceTime1) > debounceDelay1) {
    // whatever the reading1 is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading1 != btnLeft) {
      btnLeft = reading1;

      // only toggle the LED if the new button state is HIGH
      if (btnLeft == HIGH) {
        ledState = !ledState;
      }
    }
  }

  // set the LED:
  digitalWrite(flag, ledState);

  // save the reading1.  Next time through the loop,
  // it'll be the lastbtnLeft:
  lastbtnLeft = reading1;
}

