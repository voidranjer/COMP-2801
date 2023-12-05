import java.awt.*;
import java.io.*;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class LidarDisplayApp {
  private static final int   ROBOT_RADIUS = 14; //cm
  private static final int   ROBOT_COLOR = 0xC8C8FF;
  
  static final int Magnification = 1; 
  static int       WindowWidth;
  static int       WindowHeight;
  
  // The lidar model
  private LidarProcessor  aLidar;
  
  private Display       display;             // inner display window of Webots that will display the map
  private int           DISPLAY_COUNTER = 0; // This is used to limit the number of times to display
  private int[]         gridImage;           // For display purposes only
    
  public LidarDisplayApp(int widthInCentimeters, int heighInCentimeters, Display d) {
    display = d;
    WindowWidth = widthInCentimeters;
    WindowHeight = heighInCentimeters;
    
    aLidar = new LidarProcessor(90);
    gridImage = new int[WindowWidth*WindowHeight];
  }
  
  // Save a set of lidar readings
  void applyLidarReading(float[] newRanges, int x, int y, int angle) { 
    aLidar.applyReadings(newRanges, angle);
    drawLidarDataFrom(x, y);
  }
  
  // Display way point
  void setWayPoint(int centerX, int centerY, int x, int y) { 
    aLidar.setCenterPoint(centerX, centerY);
    drawLidarDataFrom(x, y);
  }
  
  void setDoorwayPoints(int leftX, int leftY, int rightX, int rightY, int x, int y) { 
    aLidar.setLeftWayPoint(leftX, leftY);
    aLidar.setRightWayPoint(rightX, rightY);
    drawLidarDataFrom(x, y);
  }
  
    // This is the method that is responsible for displaying the map
  public void drawLidarDataFrom(int rx, int ry) {
    // Only draw once in a while (to speed things up)
    if (DISPLAY_COUNTER++ %5 != 0)
        return;
        
    float[]  data = aLidar.getRangeData();
    int         x = 0; 
    int         y = 0;
    float     inc = (float)Math.PI/180.0f;
    
    // Erase the background
    for (int i=0; i<WindowWidth*WindowHeight; i++)
      gridImage[i] = 0x00000000;

    // Draw the range data
    try{
    for (int a=0; a<360; a++) {
      double angle = Math.toRadians(a);
      x = (rx + (int)(data[a] * Math.cos(angle)))/2;
      y = WindowHeight - (ry + (int)(data[a] * Math.sin(angle)))/2;
      if ((x>5) && (y>5) && (x<WindowWidth-5) && (y<WindowHeight-5)) {
        for (float an=0; an<Math.PI*2; an+=inc) {
          for (float d=0; d<2; d+=0.5f) {
            int px = (int)(x+d*Math.cos(an));
            int py = (int)(y+d*Math.sin(an));
            if ((py*WindowWidth + px) < (WindowWidth*WindowHeight))
              gridImage[py*WindowWidth + px] = 0xFFFF0000; // red
          }
        }
      }
    }
    } catch(Exception e) {
      System.out.println("("+x+","+y+")");
    }

    // Draw the robot as a circular ring
    for (float an=0; an<Math.PI*2; an+=inc) {
      int px = (int)(rx+ROBOT_RADIUS*Math.cos(an))/2;
      int py = WindowHeight - (int)(ry + ROBOT_RADIUS*Math.sin(an))/2;
      gridImage[py*WindowWidth + px] = 0xFFC8C8FF; //light blue
    }
    
    // Draw the left and right way points if they are available and close enough
    Point pl  = aLidar.getLeftWayPoint();
    Point pr  = aLidar.getRightWayPoint();
    if (((pl.x != 0) && (pl.y != 0)) && ((pr.x != 0) && (pr.y != 0))) {
      x = (rx + pl.x)/2;
      y = WindowHeight - (ry + pl.y)/2;  
      if ((x>0) && (y>0)) {
        for (float a=0; a<Math.PI*2; a+=inc) {
          for (float d=0; d<3; d+=0.5f) {
            int px = (int)(x+d*Math.cos(a));
            int py = (int)(y+d*Math.sin(a));
            if ((py*WindowWidth + px) < (WindowWidth*WindowHeight))
              gridImage[py*WindowWidth + px] = 0xFF00FF00; // green
          }
        }
      }
      x = (rx + pr.x)/2;
      y = WindowHeight - (ry + pr.y)/2; 
      if ((x>0) && (y > 0)) {
        for (float a=0; a<Math.PI*2; a+=inc) {
          for (float d=0; d<3; d+=0.5f) {
            int px = (int)(x+d*Math.cos(a));
            int py = (int)(y+d*Math.sin(a));
            if ((py*WindowWidth + px) < (WindowWidth*WindowHeight))
              gridImage[py*WindowWidth + px] = 0xFF6666FF; // blue
          }
        }
      }
    }
    
    // Draw the center way point
    x = (aLidar.getCenterPoint().x)/2;
    y = WindowHeight - (aLidar.getCenterPoint().y)/2; 
    if ((x>5) && (y>5) && (x<WindowWidth-5) && (y<WindowHeight-5)) { 
      for (float a=0; a<Math.PI*2; a+=inc) {
        for (float d=0; d<3; d+=0.5f) {
          int px = (int)(x+d*Math.cos(a));
          int py = (int)(y+d*Math.sin(a));
          if ((py*WindowWidth + px) < (WindowWidth*WindowHeight))
            gridImage[py*WindowWidth + px] = 0xFFFFFFFF; // White
        }
      }
    }
  
    // Overlay the image onto the environment
    ImageRef imageOfGrid = display.imageNew(WindowWidth, WindowHeight, gridImage, Display.ARGB);
    display.imagePaste(imageOfGrid, 0, 0, false);
  }
}
