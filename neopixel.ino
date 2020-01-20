void lightbar(float distanceFromLine ){
  int level = constrain (distanceFromLine * 0.1 / cmPerLightbarPixel , -centerpixcel ,centerpixcel);
  byte n = level + centerpixcel;
  for (int i = 0 ;i < NUMPIXELS; i++){ //Clear All pixel
    pixels.setPixelColor(i,0,0,0);
  }
  if (distanceFromLine != 32020){ //autosteer ON
    pixels.setPixelColor(n, pixels.Color(levelcolor[n][0],levelcolor[n][1],levelcolor[n][2])); //Light Bar
    pixels.setPixelColor(centerpixcel,10,10,0);//Yellow center
  }
  pixels.show();
}
