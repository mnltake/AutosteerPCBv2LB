void lightbar(float distanceFromLine ,byte cmPerLightbarPixcel ){
  byte centerpixcel = (NUMPIXELS-1) /2;
  int level = constrain (distanceFromLine * 0.1 / cmPerLightbarPixcel , -centerpixcel ,centerpixcel);
  byte levelcolor[NUMPIXELS][3];
  for (int i =0 ;i < centerpixcel;i++){ //Right
    levelcolor[i][0]=0; levelcolor[i][1]=255; levelcolor[i][2]=0;//Green
  }
  for (int i = centerpixcel;i < NUMPIXELS;i++){ //Left
    levelcolor[i][0]=255; levelcolor[i][1]=0; levelcolor[i][2]=0;  //Red
  }
  int n = level + centerpixcel;
  for (int i = 0 ;i < NUMPIXELS; i++){ //Clear All pixel
    pixels.setPixelColor(i,0,0,0);
  }
  if (distanceFromLine < 32020){ //autosteer ON
    pixels.setPixelColor(n, pixels.Color(levelcolor[n][0],levelcolor[n][1],levelcolor[n][2])); //Light Bar
    pixels.setPixelColor(centerpixcel,10,10,0);//Yellow center
  }
  pixels.show();
}
