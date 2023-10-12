import java.io.*;
import java.awt.Dimension;
import com.cyberbotics.webots.controller.Display;

public class MapperApp {
  
  static final float Resolution = 0.5f; // 0.5 cm per grid cell
  static final int Magnification = 2;  // 2x magnification
  static int WindowWidth;
  static int WindowHeight;
  
  private Map      aMap;     // The model (i.e. the occupancyGrid)
  private Display  display;  // The inner display window of Webots that will display the map

  public MapperApp(int widthInCentimeters, int heighInCentimeters, Display d) {
    display = d;
    
    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification);
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification);
    
    aMap = new Map((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution);
  }
  
  void addObjectPoint(float x, float y) { 
    aMap.setObjectPoint(x, y);
    aMap.draw(display, Magnification);
  }
}
