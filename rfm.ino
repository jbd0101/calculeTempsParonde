boolean sendPassage(){
        char radiopacket[20] = "PASSAGE";
        itoa(packetnum++, radiopacket+13, 10);
        Serial.print(F("Sending ")); Serial.println(radiopacket);
        
        // Send a message!
        rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
        rf69.waitPacketSent();
      
        // Now wait for a reply
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        
       //on veut une reponse par politesse
        if (rf69.waitAvailableTimeout(500))  { 
          // Should be a reply message for us now   
          if (rf69.recv(buf, &len)) {
            Serial.print(F("Got a reply: "));
            Serial.println((char*)buf);
            //printscreen(F("\n OK"),F("Passage"));
            delay(2000);
            return(true);
            
          } else {
            return(false);
            Serial.println(F("Receive failed"));
          }
        } else {
            //printscreen(F("\n NO RESP"),F("Passage"));
            return(false);
        }
      
 }
