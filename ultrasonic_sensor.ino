void calcule_distance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343  
  if(cm< 6 || cm>2000){
    cm=250;//max
  }
  pourcentage = cm*1.0 / base_dist*1.0 *100;
  /*Serial.print(cm);
  Serial.print(F("cm"));
  Serial.println(); */

  }
