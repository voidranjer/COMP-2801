import java.util.*;
import java.io.*;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class Map {
  private byte[][]   occupancyGrid;
  private int[]      gridImage; // For display purposes only
  private int        width;
  private int        height;
  private double     resolution; 

  private int        DISPLAY_COUNTER = 0; // This is used to limit the number of times to display

  public Map(int w, int h, double res) {
    //System.out.println("MAP WITH RESOLUTION ("+w+" x "+h+")");
    width = w;
    height = h;
    resolution = res;
    occupancyGrid = new byte[width][height];
    gridImage = new int[width*height];
  }

  public void setObjectPoint(float x, float y) {
   // System.out.println("Adding ("+x+","+y+")");
    if (((x/resolution+width/2) >= 0) && ((y/resolution+height/2) >= 0) &&
        ((x/resolution+width/2) < width) && ((y/resolution+height/2) < height))
      occupancyGrid[(int)(x/resolution+width/2)][(int)(y/resolution+height/2)] = 1;
  }
  
  public void resetObjectPoint(int x, int y) {
    if (((x/resolution+width/2) >= 0) && ((y/resolution+height/2) >= 0) &&
        ((x/resolution+width/2) < width) && ((y/resolution+height/2) < height))
      occupancyGrid[(int)(x/resolution+width/2)][(int)(y/resolution+height/2)] = 0;
  }

  // Draw the map onto the webots display
  public void draw(Display aPen, int magnification) {
    if (DISPLAY_COUNTER++ %250 != 0)
      return;
    aPen.setColor(0x000000); //  black
    try {
      for (int y=0; y<height; y++) {
        for (int x=0; x<width; x++) {
          if (occupancyGrid[x][height - 1 - y] != 0)
            gridImage[y*width + x] = 0xFFFFFFFF;
        }
      }
      ImageRef imageOfGrid = aPen.imageNew(width, height, gridImage, Display.ARGB);
      aPen.imagePaste(imageOfGrid, 0, 0, false);
      
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }

}
