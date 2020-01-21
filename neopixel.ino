void lightbar(float distanceFromLine ){
  int level = constrain (distanceFromLine * 0.1 / cmPerLightbarPixel , -centerpixcel ,centerpixcel);
  byte n = level + centerpixcel;
  for (int i = 0 ;i < NUMPIXELS; i++){
    if (i==centerpixcel && distanceFromLine != 32020){
      pixels.setPixelColor(centerpixcel,10,10,0);//Yellow center
    }else if (i == n && distanceFromLine != 32020){
      pixels.setPixelColor(n, pixels.Color(levelcolor[n][0],levelcolor[n][1],levelcolor[n][2])); //Light Bar
    }else{
      pixels.setPixelColor(i,0,0,0);//Clear
    }
  }
  pixels.show();
}
