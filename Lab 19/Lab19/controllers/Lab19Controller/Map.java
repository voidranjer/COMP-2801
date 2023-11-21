import java.io.*;
import java.util.*;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class Map {
  private VectorMap  vectorMap;     // a vector version of the map
  private int        width;
  private int        height;
  private double     resolution;

  private int        DISPLAY_COUNTER = 0; // This is used to limit the number of times to display

  // Create a map with the given with, height,  resolution
  public Map(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    vectorMap = new VectorMap(w, h, res);
  }
  public VectorMap getVectorMap() { return vectorMap; }
  
  public void draw(Display aPen, int magnification) {
 /*   if (DISPLAY_COUNTER++ %250 != 0)
      return;
    aPen.setColor(0x000000); //  black
    try {
    
      //vectorMap.draw(aPen, magnification);
      
      
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
    }*/
  }
}
