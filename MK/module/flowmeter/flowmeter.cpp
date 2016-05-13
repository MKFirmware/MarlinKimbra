volatile byte flowrate_pulsecount;  
float flowrate;
unsigned int flowml;
static millis_t flowmeter_timer = 0;


void flow_init() {

   flowrate = 0;
   pulsecount = 0;
   flowml = 0;
   pinMode(FLOWMETER_PIN, INPUT);
   
   attachInterrupt(FLOWMETER_INTERRUPT, flowrate_pulsecounter, FALLING);
}

void flowrate_manage() {
   millis_t = now;
   now = millis()
   if(ELAPSED(now, flowmeter_timer) {
      detachInterrupt(FLOWMETER_INTERRUPT);
      flowrate  = ((1000.0 / (now - flowmeter_timer)) * flowrate_pulsecount) / FLOWMETER_CALIBRATION;
      flowmeter_timer = now + 1000UL;
      flowml = (flowrate / 60) * 1000;

      pulseCount = 0;
      attachInterrupt(FLOWMETER_INTERRUPT, flowrate_pulsecounter,, FALLING);
   }

}

void flowrate_pulsecounter()
{
  // Increment the pulse counter
  flowrate_pulsecount++;
}



