//////////////////////////////DEFINITIONS AND PREREQUISITES////////////////////////////////////
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
} detData;

uint8_t detHex; // 00 means using GPS, 01 using pressure, 02 using linear progression
///////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////DETERMINATION////////////////////////////////////////////////////
void Determination(){
  // takes in GPS data, pressure data, and current time
  // if GPS is valid, uses GPS to determine lat, long, alt, and AR
  // if GPS is not valid but pressure is, calculates altitude and AR (below 80000ft)
  // if neither are valid, and above floor with at least 10 good previous GPS hits, use linear progression
  // if below floor with bad GPS, throws error
  // uses hex indication as to whether it is using GPS, pressure, or linear progression

  if (compareGPS()){ // have compareGPS return a bool - true if GPS data is good, false if not (all 3 GPS fail)
    
    DetData.alt = GPSdata.alt;
    DetData.latitude = GPSdata.latitude;
    DetData.longitude = GPSdata.longitude;
    DetData.AR = GPSdata.AR;

    detHex = 0x00;
    
  }
  
  if (pressureValid()){ // need a way to determine if the pressure is valid
    
    DetData.pressure = pressure;
    
    if (!compareGPS()){

      DetData.alt = GetAltFromPressure();
      DetData.latitude = 0; // or other alternative - 0 may be unreliable
      DetData.longitude = 0;
      DetData.AR = GetARFromPressure();

      detHex = 0x01;
      
    } 
  }

  if (!compareGPS() && !pressureValid() && tenGoodHits() && (LastKnownAltitude >= AltFloor)){

    linearProgression();
    detHex = 0x02;    
    
  }

  if (!compareGPS() && (LastKnownAltitude < AltFloor){
    Serial.println(F("GPS NOT WORKING");
  }
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////CONTROL////////////////////////////////////////////////////////////////////////////
void Control(){
  // uses DetData to determine what state it thinks we should be in
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

  if (tempfailure){
    stateSuggest = TEMP_FAILURE;
  }

  if (batteryFailure){
    stateSuggest = BATTERY_FAILURE;
  }

  if ((detData.longitude > EASTERN_BOUNDARY) || (detData.longitude < WESTERN_BOUNDARY) || (detData.latitude >NORTHERN_BOUNDARY) || (detData.latitude < SOUTHERN_BOUNDARY){
    stateSuggest = OUT_OF_BOUNDS;  
  }

  if (millis() > MASTER_TIMER){
    stateSuggest = PAST_TIMER;
  }
}
