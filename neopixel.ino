void lightdot(float distanceFromLine ){
  int level = constrain (distanceFromLine * 0.1 / cmPerLightbarPixel , -centerpixcel ,centerpixcel);
  byte n = level + centerpixcel;
  for (int i = 0 ;i < NUMPIXELS; i++){
    if (i==centerpixcel && distanceFromLine != 32020){
      pixels.setPixelColor(i,10,10,0);//Yellow center
    }else if (i == n && distanceFromLine != 32020){
      pixels.setPixelColor(i, pixels.Color(levelcolor[i][0],levelcolor[i][1],levelcolor[i][2])); //Light Dot
    }else{
      pixels.setPixelColor(i,0,0,0);//Clear
    }
  }
  pixels.show();
}

void lightbar(float distanceFromLine ){
  int level = constrain (distanceFromLine * 0.1 / cmPerLightbarPixel , -centerpixcel ,centerpixcel);
  byte n = level + centerpixcel;
    for (int i = 0 ;i < NUMPIXELS; i++){
      if ( (i == centerpixcel && i == n)|| //Center
           (level < 0 && i >= n && i < centerpixcel && distanceFromLine != 32020)|| //Right Bar
           (level > 0 && i <= n && i > centerpixcel && distanceFromLine != 32020) ) //Left Bar
      {
        pixels.setPixelColor(i,levelcolor[i][0],levelcolor[i][1],levelcolor[i][2]);
      }else{
        pixels.setPixelColor(i,0,0,0);//Clear
      }
    }
  pixels.show();
}
