// TMT Fixed Frame Walker controller
// Version 0.4
// Written by Chris Carter
// Email: chris.carter.iee@protonmail.com
// Date: 29th December 2023
//
// History:
// V0.1: Basic step/direction/speed control via front panel switches.
// V0.2: Added remote control capability over a serial connection to a PC terminal.
// V0.3: Completely reworked the serial command handling using String()
// V0.4: Add Local/Remote switch; fix '3 times too many steps' bug

// Basic serial command set:
//
// A1 n - Number of steps for Actuator 1 (positive or negative or zero)
// A2 n - Number of steps for Actuator 1 (positive or negative or zero)
// A3 n - Number of steps for Actuator 1 (positive or negative or zero)
// ? - Report configuration for pending move
// GO - Send the pending moves to the motors
//
// Remote terminal settings:
// 8 data bits, 1 stop bit, no parity, 115,200 baud.
// Append Carriage Return.
// Local echo.

// Global variable declarations

// String array to contain parsed tokens

String receivedTokens[20];

// String array for recognized command strings

String commandString[6];

// Verbosity of reporting state for debugging purposes
bool verbose = false;

// General stuff

bool debug = false;
int debugLevel = 1;
bool remoteState = false;  // FALSE if we are using front-panel switches; TRUE if we are using serial port

// Stepper-related stuff

int step_speed_pot = A0;
int step_speed = 0;

int localRemote = A1;  // Analogue pin used to read a switch state because I am out of digital I/O...

// Commanded steps are what the user requested over the serial terminal
// Queued steps are steps that are remain to be executed as part of a move
int queuedSteps[3] = { 0, 0, 0 };        // Actuator 1 | Actuator 2 | Actuator 3 queued step counts
int commandedSteps[3] = { 0, 0, 0 };  // Actuator 1 | Actuator 2 | Actuator 3 commanded step counts
int stepsMade[3] = { 0, 0, 0 };          // Actuator 1 | Actuator 2 | Actuator 3 commanded steps made
bool commandRunning = false;             // commandRunning is true when a command has been issued with GO

// Various Boolean arrays for step, direction and front-panel switch status

bool steps[3] = { false, false, false };
bool directions[3] = { false, false, false };
bool actuator_switch_fwd[3] = { false, false, false };
bool actuator_switch_rev[3] = { false, false, false };

// Variables associated with string parsing

char inByte;  // The most recent single character received over the serial connection
int bytesWaiting = 0;
//char rxString[] = "GO GO go 12838 -2032 0 +32.3 ? A1 500";  // Text to tokenise (testing only)
char rxString[64];
char inBuffer[64];
char CR = 13;  // ASCII decimal code for Carriage Return
int readStatus;

char* token;               // Pointer to token
char delim[] = { 32, 0 };  // Delimiter is decimal 32, which is the ASCII code for the 'space' character. MUST BE NULL TERMINATED!
int i = 0;
int validCommand;    // Set to 0-4 if a valid command is received; set to -1 for an unrecognized command
bool match = false;  // TRUE if a received command matches a stored command; otherwise FALSE

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void setup() {

  // Open the serial port connection at 115,200 baud

  Serial.begin(115200, SERIAL_8N1);
  //Serial.print("Initializing FFW controller...");

  // Define commands the controller will accept

  commandString[0] = "A1";
  commandString[1] = "A2";
  commandString[2] = "A3";
  commandString[3] = "GO";
  commandString[4] = "?";

  // Set the ADC analogue reference to the default

  analogReference(DEFAULT);

  // Set Arduino Uno board I/O pin definitions

  // OUTPUTS: Stepper motor driver module Step/Direction lines

  pinMode(2, OUTPUT);  // ACT1 - STEP
  pinMode(3, OUTPUT);  // ACT2 - STEP
  pinMode(4, OUTPUT);  // ACT3 - STEP
  pinMode(5, OUTPUT);  // ACT1 - DIRECTION
  pinMode(6, OUTPUT);  // ACT2 - DIRECTION
  pinMode(7, OUTPUT);  // ACT3 - DIRECTION

  // INPUTS: Panel switches - FWD/REV switches. All have active pull-ups enabled

  pinMode(8, INPUT_PULLUP);   // ACT1 - FWD
  pinMode(9, INPUT_PULLUP);   // ACT2 - FWD
  pinMode(10, INPUT_PULLUP);  // ACT3 - FWD
  pinMode(11, INPUT_PULLUP);  // ACT1 - REV
  pinMode(12, INPUT_PULLUP);  // ACT2 - REV
  pinMode(13, INPUT_PULLUP);  // ACT3 - REV

//  Serial.println(" done.\n");
  Serial.println("Ready.\n");
}

// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------
void parseString() {

  if (debug && debugLevel > 1) {
    Serial.println("\nEntered parseString()...\n");

    Serial.print(" readStatus was: ");
    Serial.println(readStatus);

    Serial.print(" inBuffer contains: ");
    Serial.println(inBuffer);
    Serial.println();
  }

  // Copy the received string, then empty the receive buffer before next time round

  for (i = 0; i < 64; i++) {
    rxString[i] = inBuffer[i];
    inBuffer[i] = 0;
  }
  parseCommand();
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void parseCommand() {
  // This subroutine parses the string received from the serial input and returns a number that
  // represents the command (if it was valid)

  if (debug && debugLevel > 1) {
    Serial.print("\nSplitting rxString (");
    Serial.print(rxString);
    Serial.println(") into tokens:\n");
  }

  // NEED TO CLEAR ARRAY 'receivedTokens[]'!
  for (i = 0; i < 20; i++) {
    receivedTokens[i] = "";
  }

  i = 0;
  token = strtok(rxString, delim);  // Extract the first token
  //Serial.println(token);  // Check
  receivedTokens[i] = token;  // Save it
  while (token != NULL) {
    //Serial.print("Token found: ");
    //Serial.println(token);        // Check
    token = strtok(NULL, delim);  // Get the next token
    i++;
    receivedTokens[i] = token;
  }

  // Show the tokens stored in the String array

  if (debug && debugLevel > 1) {
    for (i = 0; i < 20; i++) {
      Serial.print(" receivedTokens[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.println(receivedTokens[i]);
    }
  }

  // Match the FIRST stored token against the 'acceptable commands' list

  i = 0;
  match = false;

  while (!match && i < 5) {
    if (receivedTokens[0].equalsIgnoreCase(commandString[i])) {

      if (debug && debugLevel > 1) {
        Serial.println("First token matches a command!");
        Serial.print(" receivedTokens[0]: ");
        Serial.print(receivedTokens[0]);
        Serial.println();
        Serial.print(" commandString[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(commandString[i]);
        Serial.println();
      }

      validCommand = i;
      match = true;
    } else {
      validCommand = -1;
      match = false;
    }
    i++;
  }
  if (debug && debugLevel > 1) {
    if (validCommand == -1) {
      Serial.println(" ! Unrecognized command.");
      Serial.print("validCommand: ");
      Serial.print(validCommand);
      Serial.println();
    } else {
      Serial.print("validCommand: ");
      Serial.print(validCommand);
      Serial.println();
    }
  }
  commandHandler();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void commandHandler() {
  // This subroutine works out what to do for each of the various commands received

  switch (validCommand) {

    case -1:  // UNRECOGNIZED COMMAND: Whatever the user entered was not recognized as a valid command.
      Serial.println(" Unrecognized command!");
      break;

    case 0:  // A1: This command has to be followed by an integer number of steps, i.e. 'A1 103', 'A1 -20'
             // This command will look for the required numeric value in the second token that should be available if
             // the user issued the command correctly.

      commandedSteps[0] = receivedTokens[1].toInt();
      break;

    case 1:  // A2: This command has to be followed by an integer number of steps, i.e. 'A2 103', 'A2 -20'
             // This command will look for the required numeric value in the second token that should be available if
             // the user issued the command correctly.

      commandedSteps[1] = receivedTokens[1].toInt();
      break;

    case 2:  // A3: This command has to be followed by an integer number of steps, i.e. 'A3 103', 'A3 -20'
             // This command will look for the required numeric value in the second token that should be available if
             // the user issued the command correctly.

      commandedSteps[2] = receivedTokens[1].toInt();
      break;

    case 3:  // GO: This command sends the commanded steps to the stepper motors
             // This command puts the commanded stepper motor step requests into the actual
             // step count queue, and sets a flag that notifies the system not to look for
             // further commands while the moves are happening.
             // Note that the sign of the step commands are used to determine how to set the
             // direction states (TRUE or FALSE).

      for (i = 0; i < 3; i++) {
        queuedSteps[i] = abs(commandedSteps[i]);
        (commandedSteps[i] >= 0) ? directions[i] = true : directions[i] = false;
      }
      commandRunning = true;
      break;

    case 4:  // ?: This command requests the contents of the Step Commands buffer be reported.
      // Serial.println(" > Command ?");
      Serial.println();
      Serial.println(" Step commands to be executed:");

      Serial.print("  Actuator A1: ");
      Serial.print(commandedSteps[0]);
      Serial.println(" steps");

      Serial.print("  Actuator A2: ");
      Serial.print(commandedSteps[1]);
      Serial.println(" steps");

      Serial.print("  Actuator A3: ");
      Serial.print(commandedSteps[2]);
      Serial.println(" steps");

      Serial.println();
      break;

    default:
      Serial.println(" Unrecognized command!");
  }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void stepMotors() {
  // Steps all the stepper motors one step, if their steps[] bit is set.

  // Write out Directions

  for (int i = 0; i <= 2; i++) {
    if (directions[i] == true) {
      digitalWrite(i + 5, HIGH);
    } else if (directions[i] == false) {
      digitalWrite(i + 5, LOW);
    }
  }

  // Write out Steps

  for (int i = 0; i <= 2; i++) {
    if (steps[i] == true) {
      digitalWrite(i + 2, HIGH);
      delay(1);
      digitalWrite(i + 2, LOW);
    } else {
      digitalWrite(i + 2, LOW);
      delay(1);
    }
  }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void checkControlMode() {
  // Checks the state of the Local/Remote switch connected to A1
  (analogRead(localRemote) > 511) ? remoteState = true : remoteState = false;
  //Serial.print("remoteState: ");
  //Serial.println(remoteState);
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void loop() {

  // Read the speed potentiometer
  step_speed = analogRead(step_speed_pot);

  checkControlMode();

  if (remoteState) {
    // We are in Local mode
    // Serial.println("LOCAL mode");

    // Read the front-panel switch states

    for (int i = 0; i <= 2; i++) {
      actuator_switch_fwd[i] = digitalRead(8 + i);
      actuator_switch_rev[i] = digitalRead(11 + i);
    }

    // Evaluate the front-panel switch states

    if (verbose) {
      Serial.print("FWD[0]:");
      Serial.print(actuator_switch_fwd[0]);

      Serial.print(", REV[0]:");
      Serial.println(actuator_switch_rev[0]);
    }

    for (int i = 0; i <= 2; i++) {
      if (!actuator_switch_fwd[i] && actuator_switch_rev[i]) {
        directions[i] = true;
        steps[i] = true;
      } else if (actuator_switch_fwd[i] && !actuator_switch_rev[i]) {
        directions[i] = false;
        steps[i] = true;
      } else {
        steps[i] = false;
      }
    }

    if (verbose) {
      Serial.print("directions[0]:");
      Serial.println(directions[0]);
      Serial.println();
    }

    delayMicroseconds(10 * step_speed);

    // Set new step/direction states

    // Write out Directions

    for (int i = 0; i <= 2; i++) {
      if (directions[i] == true) {
        digitalWrite(i + 5, HIGH);
      } else if (directions[i] == false) {
        digitalWrite(i + 5, LOW);
      }
    }

    // Write out Steps

    for (int i = 0; i <= 2; i++) {
      if (steps[i] == true) {
        digitalWrite(i + 2, HIGH);
        delay(1);
        digitalWrite(i + 2, LOW);
      } else {
        digitalWrite(i + 2, LOW);
      }
    }

  } else {
    // We are in Remote mode
    //Serial.println("REMOTE mode");

    // Read serial port. Only call this if there are bytes waiting to be read and an existing command
    // is not already running.

    if (Serial.available() && !commandRunning) {
      readStatus = Serial.readBytesUntil(CR, inBuffer, 64);
      parseString();
    }

    if (commandRunning) {
      // A command is currently running. As long as this is true, we will not check for serial port
      // updates.
      //
      // When we enter this block, we will have the step counts that need to be taken for the command
      // to be considered 'complete' in the queuedSteps[] array. We should do this many steps.
      // The directions to use will be in the directions[] array.
      //
      // queuedSteps[]: Steps that remain to be taken (integer)
      // directions[]: Motor directions (bool)
      // steps[]: Step the motor (bool)

      for (i = 0; i < 3; i++) {
        // For all motors
        if (queuedSteps[i] > 0) {
          // Step this motor
          steps[i] = true;
          if (queuedSteps[i] > 0) {  // Keep stepping this motor as long as queuedSteps[] for it isn't zero
            queuedSteps[i] = queuedSteps[i] - 1;
          }
        } else {
          // Do not step this motor
          steps[i] = false;
        }
      }

      stepMotors();  // Update all three motors
      delayMicroseconds(step_speed);

      // If queuedSteps[] is all zeroes, we are done.

      if ((queuedSteps[0] + queuedSteps[1] + queuedSteps[2]) == 0) {
        for (i = 0; i < 3; i++) {
          commandedSteps[i] = queuedSteps[i];
        }
        Serial.println(" All motor moves have completed.");
        commandRunning = false;
      }
    }
  }
}