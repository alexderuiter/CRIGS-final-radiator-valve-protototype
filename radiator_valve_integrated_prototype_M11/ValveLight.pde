class LightController {
  
  //int amountRadiatorsOn;
  
  //float roomTemperature[]; 
  //float desiredTemperature;
  //float radiatorTemperature;
  
  //float currentBrightness;
  //float maxBrightness;
  //float maxCurrentBrightness;
  
  //int periodTime;
  //int intensityStepSize;


  ValveLight() {
  }


  // methods here
  void determineIntensity() {
     //get amount of radiators turned on

     //map the pulsing frequency to the amount of radiators that are turned on in the house (more rediators = higher frequency)

     //get temperature of radiator itself

     //get room temperature

     //map brightness of light to the difference between room temperatre and radiator temperature (higher difference = brighter max light)
     
     //if turning bck show color of system in ful brightness. 
  }

  void determineColor() {
    //get desired temerature

    //map desired temperature to a collor from yellow (cold, 15 degrees) to orange/red (hot, 25 degrees)
    
    //if turning back, show color of system (green for sustainability) or color of person (any other color that is not used yet)
  }
}