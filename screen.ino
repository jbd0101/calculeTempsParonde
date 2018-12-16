void printscreen(String TXT,String credit){
   if(SCREEN == true){
    display.clearDisplay();
      // text display tests
    display.setTextSize(0.9);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println(credit);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.println(TXT);
    display.display();
    }
}

