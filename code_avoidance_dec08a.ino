/*
 * ETC-423    Tour Guide Robot
 * 
 * This robot has three programmed paths which can be started once user input has been submitted.
 * Path selection is done with two push buttons.  One to increment a counter for the selected
 * path, the second confirms the path entered and begins the selected path.  There are two sets
 * of LEDs used which are indicators for path selection (displayed in binary) and our playback
 * device (tour guide explanation of area).  After any path is completed, the path selection
 * LEDs blink three times before clearing, indicating the successful completion of a path.  This
 * also indicates that the robot is ready to run another path selected by the user.
 * 
 * The following programmed paths are: 
 *    pathOne    - Hallway run from far bathroom to classroom
 *    pathTwo    - pathOne, in reverse
 *    pathThree  - Room clear
 * 
 * Group members:
 *    Salvatore Carollo
 *    Jessica Catelotti
 *    Antu Das
 *    Rizvo Memisevic
 *    Konrad Mozdzen
 *    Jerry Stuart
 *    Peter Thomsen
 * 
 * Last updated on 12/08/15 @ 11:24a
 */

// Libraries
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// Motor Controller
SoftwareSerial SWSerial(NOT_A_PIN, 13); // RX on no pin (unused), TX on pin 13 (to S1).
Sabertooth ST(128, SWSerial);           // Address 128, and use SWSerial as the serial port.

// Global Constants
const int buttonPath = 2;   // the pin that the pushbutton for path selection is attached to
const int buttonConf = 3;   // The pin that the pushbutton for path conformation is attached to
const int ledZero = 4;      // The pin that LED for 2^0 is attached to
const int ledOne = 5;       // The pin that LED for 2^1 is attached to
const int IR = 8;           // IR sensor pin
const int flashingLED_1 = 9;
const int flashingLED_2 = 10;

// Global Variables
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int IR_cnt = 0;              // counter used in object detection
int endReached = 0;          // trigger used to determine if in a corner


void setup()
{
  SWSerial.begin(9600);   // Serial communicaiton to the motor controller
  delay(200);             // Give the motor controller time to boot 
  ST.autobaud();          // The motor controller configures it's baud rate to 
                          // match the first character sent to it. The autobaud
                          // function sends 0xAA (0b 1010 1010).
  // initializing I/O:
  pinMode(buttonPath, INPUT);
  pinMode(buttonConf, INPUT);
  pinMode(ledZero, OUTPUT);
  pinMode(ledOne, OUTPUT);
  pinMode(flashingLED_1, OUTPUT);
  pinMode(flashingLED_2, OUTPUT);

  Serial.begin(9600);
}

void loop()
/*
 * The main program runs forever calling fuctions pathSelect and pathRun.  pathSelect prompts
 * user for entry while pathRun calls the corresponding value selected in the previous function
 * executing the desired path.
 */
{
  pathSelect();
  pathRun(buttonPushCounter);
}

void pathSelect()
/*
 * Function runs until the path conformation button (buttonConf) is pressed.  Each time the
 * path selection button (buttonPath) is pressed, a path selection counter (buttonPushCounter)
 * is incremented.  There are only three valid path entries therefore if the path selection
 * counter (buttonPushCounter) is out of range, counter is reset to 1.
 */
{
  while(!buttonPushCounter)                     // Path must be selected before exiting this function
  {
    while(!digitalRead(buttonConf))
    {
      buttonState = digitalRead(buttonPath);    // Update pushbutton input
    
      if(buttonState != lastButtonState)        // If pushbutton input has changed from previously stored state
      {
        if(buttonState == HIGH)
        // If pushbutton is currently pressed (Leading edge trigger)
        {
          buttonPushCounter++;                  // Increment path selection counter
        }
        delay(50);                              // Delay for debouncing
      }

      lastButtonState = buttonState;            // Update/store pushbutton state

      if(buttonPushCounter == 4)                // True if path selection is out-of-range
      {
        buttonPushCounter = 1;                  // Reset path selection counter
      }
    
      updateLED(buttonPushCounter);             // Update LEDs with path selection counter value
    } // while(!digitalRead(buttonConf)) END
  }   // while(!buttonPushCounter) END
}


void updateLED(int value)
/*
 * This function displays the given value in binary on LEDs ledZero and ledOne.  Expected
 * passed value range is between 0 and 3.  This is used as a visual reference for path selection.
 */
{
    if(value == 0)                    // Display a binary 0 on LEDs
    {
      digitalWrite(ledZero, LOW);
      digitalWrite(ledOne, LOW);
    }
    else if(value == 1)               // Display a binary 1 on LEDs
    {
      digitalWrite(ledZero, HIGH);
      digitalWrite(ledOne, LOW);
    }
    else if(value == 2)               // Display a binary 2 on LEDs
    {
      digitalWrite(ledZero, LOW);
      digitalWrite(ledOne, HIGH);
    }
    else                              // value == 3, display a binary 3 on LEDs
    {
      digitalWrite(ledZero, HIGH);
      digitalWrite(ledOne, HIGH);
    }
}


void pathRun(int value)
/*
 * Calls path function that corresponds to the value given when called.  After completion,
 * function pathSuccess is called signaling the completion of path function called and all
 * forward progress is halted.
 */
{
  if(value == 1)
  {
    pathOne();        // Call desired path function
    ST.drive(0);      // To insure that all movement is STOPPED after path completion
    ST.turn(0);
    pathSuccess();    // Visual signal showing that the path has completed successfully
  }
  else if(value == 2)
  {
    pathTwo();
    ST.drive(0);
    ST.turn(0);
    pathSuccess();
  }
  else                // value == 3
  {
    pathThree();
    ST.drive(0);
    ST.turn(0);
    pathSuccess();
  }
}

void pathSuccess()
/*
 * This function flashes the LEDs as a visual conformation that a path has ran to completion.
 * The number 3 (in binary) is flashed three times, pushbutton counter reset.
 */
{
  updateLED(0);             // Turn OFF all path selection LEDs
  delay(50);
  updateLED(3);             // Turn ON all path selection LEDs
  delay(250);
  updateLED(0);
  delay(50);
  updateLED(3);
  delay(250);
  updateLED(0);
  delay(50);
  updateLED(3);
  delay(250);
  updateLED(0);
  buttonPushCounter = 0;    // Reset counter/robot for user input of new path selection
}


void tourGuideDesc()
/*
 * This function will be used to indicate that our robot has stopped to provide a "tour guide's"
 * description of the area.  If we have time, the LEDs could be replaced by an audio-playback
 * device.
 */
{
  ST.drive(0);                // Stop forward progess

  int descLength = 0;
  while(descLength < 10)
  /*
   * Signaling delay loop, blinking of "playback" LEDs for 10 seconds.  Changing this value will 
   * increase/decrease function runtime.
   */
  {
    digitalWrite(9, HIGH);    // turn the LED on (HIGH is the voltage level)
    digitalWrite(10, HIGH);
    delay(500);               // wait for a 1/2 second
    digitalWrite(9, LOW);     // turn the LED off by making the voltage LOW
    digitalWrite(10, LOW);
    delay(500);
    descLength++;
  }

  ST.drive(128);                // Start forward progess
  ST.turn(0);
}


void pathOne()
/*
 * This path escorts a group from the lounge area pass the restrooms on the first floor, down
 * the hallway, ending in front of our classroom door (C014).  There are two pre-programmed
 * guide stops in front of the bathrooms/vending machines and nfrastructure TV.  Robot will
 * also stopped when an object is dectected using an IR sensor.  If the object doesn't move
 * for 5 seconds it will be deemed as a wall.  If a wall is detected a 45-degree turn is
 * executed before the robot continues foward progress.  Once the opposing wall is detected,
 * another 45-degree turn is executed.  At this point the robot will be orientated parallel
 * with the wall and foward progress is continued.
 */
{
  IR_cnt = 0;   // Counter used to determine whether object is a wall or not
  int guideStop = 0, turnCount = 0, guideCount = 0;
  delay(1000);  // Short delay before path run begins

  while(turnCount < 2)
  {
    if((guideStop == 2500 && turnCount == 0 && guideCount == 0) || (guideStop == 2800 && turnCount == 1 && guideCount == 1))
    /*
     * Our first stop should be in front of the bathrooms and vending machines (first condition), second stop in front of 
     * nfrastructure TV (second condition).  Counters guideStop and guideCount are updated, insuring that both conditions 
     * are satisfied.
     */
    {
      tourGuideDesc();
      guideStop = 0;
      guideCount++;
    }

    if(digitalRead(IR))   // If IR = 1 (TRUE), no object detected
    {
      ST.drive(128);      // Drive forward at full speed
      ST.turn(0);
    }
    else                  // Else, IR = 0, object detected
    {
      ST.drive(0);        // Stop both motors
      IR_cnt++;
      delay(250);         // wait 250ms
    }

    if(IR_cnt == 20)      // Object still remains after 5sec (20*250ms=5sec)
    // This IF-statement will be used to travel around the walls/bends in the hallway
    {
      // Here we are trying to turn 45 degrees to the left at full speed
      delay(250);
      ST.turn(-128);
      delay(500);         // Modifier for distance travelled         
      ST.drive(0);

      // Now we are ready to drive along the front of the object until adjacent wall detected
      delay(250);
      ST.drive(128);
      ST.turn(0);

      while(digitalRead(IR))
      {
        /*
         * Robot will continue foward until adjacent wall is reached.
         * Once adjacent wall detected (IR = 0), while loop is exited.
         */
      }
      ST.drive(0);
      delay(250);

      /*
       * By this point the object should be cleared, we need to orient ourselves before 
       * driving forward.  The following code should mirror exactly our first turn at the 
       * beginning of IF-statement.
       */
      ST.turn(128);
      delay(500);
      ST.drive(0);
      
      // We are now clear of the wall/bend and can now continue.
      IR_cnt = 0;         // Reset conditional counter
      turnCount++;
      guideStop = 0;
    } // if(IR_cnt == 20) END

    guideStop++;
  }   // while(1) END
  
  ST.drive(128);
  ST.turn(0);
  delay(14000);     // 14sec delay before stopping in front of classroom
}     // pathOne END


void pathTwo()
/*
 * Other than the direction of travel and pre-programmed guide stops, this fuction mirrors
 * pathOne almost exactly.  The robot starts in front of our classroom door (C014) and
 * travels down the hallway, stopping in front of the bathrooms/vending machines.  Our first
 * stop is in front of the Linux Lab (C012) and the second stop is in front of the electricty 
 * closet (C007).
 */
{
  IR_cnt = 0;   // Counter used to determine whether object is a wall or not
  int guideStop = 0, turnCount = 0, guideCount = 0;
  delay(1000);  // Short delay before path run begins
  
  while(turnCount < 2)
  {
    if((guideStop == 1000 && turnCount == 0 && guideCount == 0) || (guideStop == 1200 && turnCount == 1 && guideCount == 1))
    // Our first stop should be in front of room 12 (Linux Lab), second stop in front of electricty closet (rm 7).
    /*
     * Our first stop should be in front of the Linux Lab room C012 (first condition), second stop in front of electricty
     * closet room C007 (second condition).  Counters guideStop and guideCount are updated, insuring that both conditions 
     * are satisfied.
     */
    {
      tourGuideDesc();
      guideStop = 0;
      guideCount++;
    }
    if(digitalRead(IR))   // If IR = 1 (TRUE), no object detected
    {
      ST.drive(128);      // Drive forward at full speed
      ST.turn(0);
    }
    else                  // Else, IR = 0, object detected
    {
      ST.drive(0);        // Stop both motors
      IR_cnt++;
      delay(250);         // wait 250ms
    }

    if(IR_cnt == 20)      // Object still remains after 5sec (20*250ms=5sec)
    // This IF-statement will be used to travel around the walls/bends in the hallway
    {
      // Here we are trying to turn 90 degrees to the left at full speed
      ST.turn(-128);
      delay(1000);        // Modifier for distance travelled         
      ST.drive(0);

      // Now we are ready to drive along the front of the object until adjacent wall detected
      ST.drive(128);
      ST.turn(0);

      while(digitalRead(IR))
      {
        /*
         * Robot will continue foward until adjacent wall is reached.
         * Once adjacent wall detected (IR = 0), while loop is exited.
         */
      }
      ST.drive(0);

      /*
       * By this point the object should be cleared, we need to orient ourselves before 
       * driving forward.  The following code should mirror exactly our first turn at the 
       * beginning of IF-statement.
       */
      ST.turn(128);
      delay(1000); 
      ST.drive(0);
      
      // We are now clear of the wall/bend and can now continue.
      IR_cnt = 0;         // Reset conditional counter
      turnCount++;
      guideStop = 0;
    } // if(IR_cnt == 20) END
    
    guideStop++;
  }   // while(1) END
  
  ST.drive(128);
  ST.turn(0);
  delay(14000);     // 14sec delay before stopping in front of classroom
}     // pathTwo END


void pathThree()
/*
 * This path is our "snake n' clear", the robot snakes back-and-forth along the room in a
 * rectangular zagging pattern.  Once the robot has driven into a corner, we have determined
 * the room has been clear and function ends.
 */
{
  // Initializing function variables used in this path
  IR_cnt = 0;                   // Counter used to determine whether object is a wall or not
  int firstTurn = 0, corner = 0;      // Used for right turn completion and corner detection
  digitalWrite(flashingLED_1, LOW);
  
  delay(1000);                  // Short delay before movement begins
  
  while(!corner)
  // If we determined that a corner has been reached, exit while-loop
  {
    while(firstTurn == 0 && corner == 0)
    {
      if(firstTurn == 0 && corner == 0)
      {
      
        if(digitalRead(IR))     // If IR = 1 (TRUE), no object detected
        /*
        * Robot will continue foward as long as path is clear, once an object is detected,
        * the robot will stop, INC a counter and continue forward once object is cleared OR
        * next IF statement becomes true.
        */
        {
          ST.drive(128);
          ST.turn(0);
        }
        else                    // Else, IR = 0, object detected
        {
          ST.drive(0);          // Stop both motors
          IR_cnt++;
          delay(250);           // wait 250ms
        }

        if(IR_cnt == 10)        // Object still remains after 2.5sec (10*250ms=2.5sec)
        /*
        * Its been determined that a wall has been reached.  Robot will stop foward progress
        * before making two 90-degree turns before continuing to clear the room in the opposite
        *  direction.
        */
        {
          // turning 90 degrees for 500ms
          ST.drive(0);
          ST.turn(128);
          delay(900);
        
          // drive forward a little bit
          ST.turn(0);
          delay(250); // Torque dealy
          if(!digitalRead(IR))
          {
            corner = 1;
          }
          else
          {
            ST.drive(128);
            ST.turn(0);
            delay(500);

            // turning 90 again = 180 degree turn
            ST.drive(0);
            delay(250); // Torque dealy
            ST.turn(128);
            delay(900);
            ST.turn(0);
            IR_cnt = 0;           // counter reset for next wall detection process
            firstTurn++;
            digitalWrite(flashingLED_1, HIGH);
            delay(1000);
          } // else END
          
        }   // if(IR_cnt == 10) END
        
      }     // if(firstTurn == 0 && corner == 0) END
      
    }       // while(firstTurn == 0 && corner == 0) END
    
    /*
     * The previous lines of code (above) will repeat but this time once the opposite  wall
     * is reached, the robot will make two 90-degree turns to the left.
     */
    while(firstTurn == 1 && corner == 0)
    {
      if(firstTurn == 1 && corner == 0)
      {
        if(digitalRead(IR))
        {
          ST.drive(128);
          ST.turn(0);
        }
        else
        {
          ST.drive(0);
          IR_cnt++;
          delay(250);
        }

        delay(500);

        if(IR_cnt == 10)
        {
          // turning 90 degrees for 500ms
          ST.drive(0);
          ST.turn(-128);
          delay(900);

          // drive forward a little bit
          ST.turn(0);
          delay(250); // Torque dealy
          if(!digitalRead(IR))
          {
            corner = 1;           // We are now in a corner, room cleared
          }
          else
          {
            ST.drive(128);
            ST.turn(0);
            delay(500);

            // turning 90 again = 180 degree turn
            ST.drive(0);
            delay(250); // Torque dealy
            ST.turn(-128);
            delay(900);
            ST.turn(0);
            IR_cnt = 0;           // counter reset for next wall detection process
            firstTurn--;
            digitalWrite(flashingLED_1, LOW);
            delay(1000);
          } // else END
          
        }   // if(IR_cnt == 10) END
        
      }     // if(firstTurn == 1 && corner == 0) END
      
    }       // while(firstTurn == 1 && corner == 0) END
    
  }         // while(!corner) END
  
}           // pathThree END

