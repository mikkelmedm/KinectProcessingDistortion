// Mikkel Loose

// Build on top of:
// Depth thresholding example by Daniel Shiffman

// https://github.com/shiffman/OpenKinect-for-Processing
// http://shiffman.net/p5/kinect/

// Original example by Elie Zananiri
// http://www.silentlycrashing.net

int tal = 0;

import gab.opencv.*;
import org.openkinect.freenect.*;
import org.openkinect.processing.*;

Kinect kinect;
OpenCV opencv;
ArrayList<Contour> contours;

// Depth image
PImage depthImg, bg, img2, cs;

// Måske kan denne fin-justeres lidt:
int blurFactor = 7;

// TODO - Fjernsyn - Fix depth og threshold
//                 - Tjek opacity ift lys, om man kan se tingene ordenligt osv.

// Which pixels do we care about?
int minDepth =  60;
int maxDepth = 850;

float blue1 = 10;
float blue2 = 10;
float blue3 = 10;

void setup() {
  
 
  fullScreen();
  //size(1280, 720);
  background(0);

  // Fjernsyn er 1280x720

  bg = loadImage("8611.jpg");
  cs = loadImage("colorstatic.jpg");
  img2 = createImage(1280, 800, RGB);

  kinect = new Kinect(this);
  kinect.initDepth();  
  kinect.enableMirror(true);

  depthImg = new PImage(kinect.width, kinect.height);
  opencv = new OpenCV(this, img2);

  contours = opencv.findContours();
}

void draw() {
  // Skal måske udkommenteres:
  //surface.setTitle(int(frameRate) + " fps");

  // Threshold the depth image
  int[] rawDepth = kinect.getRawDepth();
  for (int i=0; i < rawDepth.length; i++) {
    if (rawDepth[i] >= minDepth && rawDepth[i] <= maxDepth) {
      depthImg.pixels[i] = color(255);//img.pixels[i];
    } else {
      depthImg.pixels[i] = color(0);
    }
  }

  fastblur(depthImg, blurFactor);
  depthImg.updatePixels();

  img2.copy(depthImg, 0, 0, depthImg.width, depthImg.height, 0, 0, img2.width, img2.height);

  opencv.loadImage(img2);

  strokeWeight(1);

  if (tal<100) {
    if (tal % 10 == 0) {
      drawContours(false, false, false, true, false, false, false);
    }
    tal++;
  } else if (tal==100) {
    drawContours(false, false, false, false, false, false, true);
    tal++;
  } else if (tal>100 && tal<200) {
    drawContours(true, false, false, false, false, false, false);
    tal++;
  } else if (tal>=200 && tal<300) {
    drawContours(false, true, false, false, false, false, false);
    tal++;
  } else if (tal>=300 && tal<400) {
    drawContours(false, false, true, false, false, false, false);
    tal++;
  } else if (tal>=400 && tal<=410) {
    drawContours(false, false, false, false, false, false, true);
    tal++;
  } else if (tal>410 && tal<500) {
    drawContours(false, false, false, false, true, false, false);
    tal++;
  } else if (tal>=500 && tal<510) {
    drawContours(false, false, false, false, false, true, false);
    tal++;
  } else if (tal==510) {
    drawContours(false, false, false, false, false, false, true);
    tal++;
  } else if (tal>=411) { 
    tal=0;
  }
  
  //if (hasFinished()) {
  //  println(WAIT_TIME/1000
  //    + " seconds have past! At "
  //    + millis()
  //    + " millis.");
  //  startTime = millis();
  //  //setup();
    
  //}
}

//boolean hasFinished() {
//  return millis() - startTime >= WAIT_TIME;
//}

void reset() {
  kinect = new Kinect(this);
  kinect.initDepth();  
  kinect.enableMirror(true);
}

void fastblur(PImage img, int radius)
{
  if (radius<1) {
    return;
  }
  int w=img.width;
  int h=img.height;
  int wm=w-1;
  int hm=h-1;
  int wh=w*h;
  int div=radius+radius+1;
  int r[]=new int[wh];
  int g[]=new int[wh];
  int b[]=new int[wh];
  int rsum, gsum, bsum, x, y, i, p, p1, p2, yp, yi, yw;
  int vmin[] = new int[max(w, h)];
  int vmax[] = new int[max(w, h)];
  int[] pix=img.pixels;
  int dv[]=new int[256*div];
  for (i=0; i<256*div; i++) {
    dv[i]=(i/div);
  }

  yw=yi=0;

  for (y=0; y<h; y++) {
    rsum=gsum=bsum=0;
    for (i=-radius; i<=radius; i++) {
      p=pix[yi+min(wm, max(i, 0))];
      rsum+=(p & 0xff0000)>>16;
      gsum+=(p & 0x00ff00)>>8;
      bsum+= p & 0x0000ff;
    }
    for (x=0; x<w; x++) {

      r[yi]=dv[rsum];
      g[yi]=dv[gsum];
      b[yi]=dv[bsum];

      if (y==0) {
        vmin[x]=min(x+radius+1, wm);
        vmax[x]=max(x-radius, 0);
      }
      p1=pix[yw+vmin[x]];
      p2=pix[yw+vmax[x]];

      rsum+=((p1 & 0xff0000)-(p2 & 0xff0000))>>16;
      gsum+=((p1 & 0x00ff00)-(p2 & 0x00ff00))>>8;
      bsum+= (p1 & 0x0000ff)-(p2 & 0x0000ff);
      yi++;
    }
    yw+=w;
  }

  for (x=0; x<w; x++) {
    rsum=gsum=bsum=0;
    yp=-radius*w;
    for (i=-radius; i<=radius; i++) {
      yi=max(0, yp)+x;
      rsum+=r[yi];
      gsum+=g[yi];
      bsum+=b[yi];
      yp+=w;
    }
    yi=x;
    for (y=0; y<h; y++) {
      pix[yi]=0xff000000 | (dv[rsum]<<16) | (dv[gsum]<<8) | dv[bsum];
      if (x==0) {
        vmin[y]=min(y+radius+1, hm)*w;
        vmax[y]=max(y-radius, 0)*w;
      }
      p1=x+vmin[y];
      p2=x+vmax[y];

      rsum+=r[p1]-r[p2];
      gsum+=g[p1]-g[p2];
      bsum+=b[p1]-b[p2];

      yi+=w;
    }
  }
}

void drawContours(boolean edge, boolean edge2, boolean edge3, boolean fill, boolean psych, boolean colorstat, boolean black) {

  ArrayList<Contour> contours = opencv.findContours();

  strokeWeight(2);
  blue2=random(255);
  blue3=random(255);
  blue1=random(255);

  for (int i=0; i<contours.size(); i++) {
    Contour contour = contours.get(i);

    //if (contour.area() > 50000) {

    //background(0);
    if (contour!= null) {
      // Edges
      if (edge) { // Forsøg evt at skifte mellem at én af værdierne er fast
        noFill();
        stroke(255, blue2, blue2, 99);
        contour.draw();

        fill(255, blue2, blue2);
        textSize(20);
        text("@mikkelloose", 1052, 671);
      }
      if (edge2) { // Forsøg evt at skifte mellem at én af værdierne er fast
        noFill();
        stroke(blue2, blue2, 255, 99);
        contour.draw();

        fill(blue2, blue2, 255);
        textSize(20);
        text("@mikkelloose", 1052, 671);
      }
      if (edge3) { // Forsøg evt at skifte mellem at én af værdierne er fast
        noFill();
        stroke(blue2, 255, blue2, 99);
        contour.draw();

        fill(blue2, 255, blue2);
        textSize(20);
        text("@mikkelloose", 1052, 671);
      }
      // Fill
      if (fill) {

        fill(blue1, blue2, blue3);
        textSize(20);
        text("@mikkelloose", 1052, 671);

        noStroke(); // Måske der kan leges med opacity her:
        fill(blue1, blue2, blue3);
        //contour.draw();
        beginShape();
        for (PVector point : contour.getPolygonApproximation().getPoints()) {
          vertex(point.x-random(100), point.y-random(100));
        }
        endShape();
      }
      // psych
      if (psych) {

        fill(blue1, blue2, blue3);
        textSize(20);
        text("@mikkelloose", 1052, 671);

        beginShape();
        for (PVector p : contour.getPoints()) {
          noStroke();
          fill(blue3, blue1, blue2);
          vertex(p.x-random(110), p.y-random(110));
        }
        endShape();
      }
      if (colorstat) {
        background(cs);
      }
      if (black) {
        background(0);
      }
    } //else if (contour.area() < 10000) { // Re-evaluate this solution:
    //  background(0);
    //}
  }
}
