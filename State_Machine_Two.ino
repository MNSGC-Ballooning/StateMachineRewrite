//////////////////////////////DEFINITIONS AND PREREQUISITES////////////////////////////////////
#define ALTITUDE_FLOOR 5000
#define ALTITUDE_CEILING 100000
#define SA_FLOOR 50000
#define SLOW_DESCENT_FLOOR 80000
////change boundaries before every flight!!!////
#define EASTERN_BOUNDARY 90
#define WESTERN_BOUNDARY 90
#define SOUTHERN_BOUNDARY 90
#define NORTHERN_BOUNDARY 90
///////////////////////////////////////////////
#define MASTER_TIMER 180*M2MS
#define ASCENT_TIMER 150*M2MS
#define SA_TIMER 30*M2MS
#define FLOAT_TIMER 30*M2MS
#define SLOW_DESCENT_TIMER 40*M2MS
#define INITIALIZATION TIME 25*M2MS


#define INITIALIZATION 0x00
#define ASCENT 0x01
#define SLOW_ASCENT 0x02
#define FLOAT 0x03
#define SLOW_DESCENT 0x04
#define DESCENT 0x05
#define TEMP_FAILURE 0x06
#define BATTERY_FAILURE 0x07
#define OUT_OF_BOUNDS 0x08
#define PAST_TIMER 0x09

// have compareGPS output the following GPS average of all systems, saved as struct
struct GPSData{ // proposed struct filled out by compareGPS
  float alt;
  float latitude;
  float longitude;
  float AR;
} GPSdata;

struct DetData{ // proposed struct filled out by Determination
  // lat, long = 0 if GPS data is bad (might want to use something other than 0)
  // pressure = 0 if GPS data is good
  
  float alt;
  float latitude;
  float longitude;
  float AR;
  float pressure;
  float Time; 
  uint8_t Usage; // 00 means using timer (error on all others), 01 means using 1 GPS, 02 using 2 GPS, 03 all 3 GPS, 04 using pressure, 05 using linear progression
} detData;

uint8_t ascentCounter = 0, SAcounter = 0, floatCounter = 0, descentCounter = 0;
uint8_t tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0;
unsigned long ascentStamp = 0, SAstamp = 0, floatStamp = 0, defaultStamp = 0, defaultStamp2, defaultStampCutA = 0;

uint8_t stateSuggest; // state recommended by control
uint8_t currentState = INITIALIZATION; // state we are in, starts as initialization
///////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////DETERMINATION////////////////////////////////////////////////////
void Determination(){
  // takes in GPS data, pressure data, and current time
  // if GPS is valid, uses GPS to determine lat, long, alt, and AR
  // if GPS is not valid but pressure is, calculates altitude and AR (below 80000ft)
  // if neither are valid, and above floor with at least 10 good previous GPS hits, use linear progression
  // if below floor with bad GPS, throws error
  // uses hex indication as to whether it is using GPS, pressure, or linear progression

  if (compareGPS()){ // have compareGPS return a bool - true if GPS data is good, false if not (all 3 GPS fail) SET DETDATA.USAGE TO 1, 2, or 3 in COMPAREGPS FUNCTION
    
    detData.alt = GPSdata.alt;
    detData.latitude = GPSdata.latitude;
    detData.longitude = GPSdata.longitude;
    detData.AR = GPSdata.AR;
    
  }
  
  else if (pressureValid()){ // need a way to determine if the pressure is valid

    Serial.println(F("GPS NOT WORKING");
    detData.pressure = pressure;
    detData.alt = GetAltFromPressure();
    detData.latitude = 0; // or other alternative - 0 may be unreliable
    detData.longitude = 0;
    detData.AR = GetARFromPressure();
    detData.Usage = 0x04; // using pressure for determination
      
  }

  else if (tenGoodHits && (LastKnownAltitude >= ALTITUDE_FLOOR)){

    Serial.println(F("GPS NOT WORKING");
    linearProgression();
    detData.Usage = 0x05; // using linear progression
    
  }

  else{
    Serial.println(F("GPS NOT WORKING"); // outputs a warning if GPS not working while on the ground
    detData.Usage = 0x00; // indicates error
    // have LED indication
  }
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////CONTROL////////////////////////////////////////////////////////////////////////////
void Control(){
  // uses detData to determine what state it thinks we should be in
  // worst case states take priority, but every possible state is outputted as hex

  if (detData.AR>=375){
    stateSuggest = ASCENT;
  }
  if (detData.AR<375 && detData.AR>=100){
    stateSuggest = SLOW_ASCENT;
  }
  if (detData.AR<100 && detData.AR>-100){
    stateSuggest = FLOAT;
  }
  if (detData.AR<=-100 && detData.AR>-375){
    stateSuggest = SLOW_DESCENT;
  }
  if (detData.AR<=375){
    stateSuggest = DESCENT;
  }

  /*if (tempfailure){
    stateSuggest = TEMP_FAILURE;
  }

  if (batteryFailure){
    stateSuggest = BATTERY_FAILURE;
  }*/

  if ((detData.longitude > EASTERN_BOUNDARY) || (detData.longitude < WESTERN_BOUNDARY) || (detData.latitude >NORTHERN_BOUNDARY) || (detData.latitude < SOUTHERN_BOUNDARY){
    stateSuggest = OUT_OF_BOUNDS;  
  }

  if (millis() > MASTER_TIMER){
    stateSuggest = PAST_TIMER;
  }

  if (detData.Usage == 0x00){ // error state
    stateSuggest = ERROR_STATE; // doesn't exist - State will go to default
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////STATE/////////////////////////////////////////////////////////////////
void State(){
  // take in stateSuggest from control and switches into that state after a predetermined number of hits
  switch (stateSuggest){
    ////ASCENT////
    case ASCENT:
    
      if (currentState!=ASCENT){ // criteria for entering Ascent functionality
        ascentCounter += 1; // increment ascent counter
        SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (ascentCounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = ASCENT;
          ascentStamp = millis();
        }
      }

      if (currentState==ASCENT){ // operations while in ascent
        
        if (detData.alt > ALTITUDE_CEILING || millis()-ascentStamp >= ASCENT_TIMER){ // if ceiling or timer is reached
          cutResistorOnA(); // only cuts A (large balloon) so slow descent can still be acheived
        }
      }

    ////SLOW ASCENT////
    case SLOW_ASCENT:
    
      if (currentState!=SLOW_ASCENT){ // criteria for entering Slow Ascent functionality
          SAcounter += 1; // increment slow ascent counter
          ascentCounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
          tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
          
          if (SAcounter >= 60 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 60 consecutive state suggestions
            currentState = SLOW_ASCENT;
            SAstamp = millis();
          }
        }
  
        if (currentState==SLOW_ASCENT){ // operations while in slow ascent
          
          if (detData.alt > ALTITUDE_CEILING || millis()-SAstamp >= SA_TIMER){ // if ceiling or timer is reached
            cutResistorOnA(); // cut only A to enter slow descent
          }

          if (detData.alt < SA_FLOOR){ // cuts immediately under threshold
            cutResistorOnA(); // only cuts A (large balloon) so some slow descent can still be acheived
          }
        }

    ////FLOAT////
    case FLOAT:

      if (currentState!=FLOAT){ // criteria for entering Float functionality
        floatCounter += 1; // increment float counter
        ascentCounter =0, SAcounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (ascentCounter >= 180 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 180 consecutive state suggestions
          currentState = FLOAT;
          floatStamp = millis();
        }
      }

      if (currentState==FLOAT){ // operations while in float
        
        if (detData.alt > ALTITUDE_CEILING){ // if ceiling is reached
          cutResistorOnA(); // cut only A to attempt to enter slow descent
        }

        if (millis()-floatStamp >= FLOAT_TIMER){ // if timer reached
          cutResistorOnA();
          cutResistorOnB(); // cut both balloons
        }
      }

    ////SLOW DESCENT////
    case SLOW_DESCENT:

      if (currentState!=SLOW_DESCENT){ // criteria for entering Slow Descent functionality
        SDcounter += 1; // increment slow descent counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (SDcounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = SLOW_DESCENT;
          SDstamp = millis();
        }
      }

      if (currentState==SLOW_DESCENT){ // operations while in slow descent
        
        if (detData.alt < SLOW_DESCENT_FLOOR || millis()-SDstamp >= SLOW_DESCENT_TIMER){ // if floor or timer is reached
          cutResistorOnA(); 
          cutResistorOnB(); // cut both balloons (A is likely cut already, but just in case)
        }
      }


    ////DESCENT////
    case DESCENT:
    
      if (currentState!=DESCENT){ // criteria for entering Descent functionality
        descentCounter += 1; // increment descent counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0;
        tempCounter = 0, battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (descentCounter >= 30 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 30 consecutive state suggestions
          currentState = DESCENT;
          descentStamp = millis();
        }
      }

      if (currentState==DESCENT){ // operations while in ascent
        
        if (millis()-ascentStamp >= SLOW_DESCENT_TIMER){ // reuses SD timer as a backup
          cutResistorOnA();
          cutResistorOnB(); // cut both balloons
        }
      }

    ////TEMPERATURE FAILURE//// To be added later... REVIEW AND ADJUST BEFORE USING
    /* case TEMP_FAILURE:
    
      if (currentState!=TEMP_FAILURE){ // criteria for entering Temperature Failure functionality
        tempCounter += 1; // increment temperature failure counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        battCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
      }

      if (currentState==TEMP_FAILURE){ // operations while in temperature failure
        cutResistorOnA();
        cutResistorOnB();
      }*/

    ////BATTERY FAILURE////To be added later... REVIEW AND ADJUST BEFORE USING
    /*case BATTERY_FAILURE:
      if (currentState!=BATTERY_FAILURE){ // criteria for entering Battery Failure functionality
        battCounter += 1; // increment battery failure counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, boundCounter = 0, timerCounter = 0; // reset all other state counters
      }

      if (currentState==BATTERY_FAILURE){ // operations while in battery failure
        cutResistorOnA();
        cutResistorOnB();
      }*/
        
    ////OUT OF BOUNDARY////
    case OUT_OF_BOUNDS:
    
      if (currentState!=OUT_OF_BOUNDS){ // criteria for entering Out of Boundary functionality
        boundCounter += 1; // increment out of boundary counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, timerCounter = 0; // reset all other state counters
        
        if (boundCounter >= 180 && detData.alt > ALTITUDE_FLOOR){ // doesn't activate below floor or before 180 consecutive state suggestions
          currentState = OUT_OF_BOUNDS;
        }
      }

      if (currentState==OUT_OF_BOUNDS){ // operations while out of boundary
        cutResistorOnA();
        cutResistorOnB(); // cut both balloons
      }

    ////MASTER TIMER REACHED////
    case PAST_TIMER:

      if (currentState!=PAST_TIMER){ // criteria for entering Master Timer Reached functionality
        timerCounter += 1; // increment ascent counter
        ascentCounter = 0, SAcounter = 0, floatCounter = 0, SDcounter = 0, descentCounter = 0; 
        tempCounter = 0, battCounter = 0, boundCounter = 0; // reset all other state counters
        
        if (timerCounter >= 10){ // activates after 10 consecutive state suggestions, regardless of altitude
          currentState = PAST_TIMER;
        }
      }

      if (currentState==PAST_TIMER){ // operations after Master Timer reached
        
        cutResistorOnA();
        timerStampCutA = millis();
         if (millis() - timerStampCutA >= SLOW_DESCENT_TIMER){ // wait to cut B (hopefully to get slow descent data)
          cutResistorOnB();
         }
      }

      break;

    ////DEFAULT////
    default: 

    // move outside of switch-case
    // has to get through initialization to trigger states
    // if initialization never triggers, moves to the next of the function where it cuts A then B

      if (currentState==INITIALIZATION){ // currentState is initialized as INITIALIZATION, no other states have been activated

        defaultStamp = millis();
        if ( (millis()-defaultStamp) >= (INITIALIZATION_TIME + ASCENT_TIMER) ){ // gives an extra time buffer to leave initialization

         cutResistorOnA();
         defaultStampCutA = millis();
         if (millis() - defaultStampCutA >= SLOW_DESCENT_TIMER){ // wait to cut B (hopefully to get slow descent data)
          cutResistorOnB();
         }
        }
          
        }

        else {
          // if it's not in initialization, means it entered another state at some point, then everything stopped working and stateSuggest = ERROR_STATE now
          // should wait a while to see if it will enter a state again, then do the same cut strategy, hoping for some slow descent
          defaultStamp2 = millis();
          if ( (millis()-defaultStamp2) >= (DEFAULT_TIME + ASCENT_TIMER) ){ // gives an extra time buffer to leave default

            cutResistorOnA();
            defaultStampCutA = millis();
            if (millis() - defaultStampCutA >= SLOW_DESCENT_TIMER){ // wait to cut B (hopefully to get slow descent data)
              cutResistorOnB();
            }
          }
        }
        
  }
}
