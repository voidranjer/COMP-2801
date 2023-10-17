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

  public MapperApp(int widthInCentimeters, int heighInCentimeters, boolean binary, Display d) {
    display = d;
    
    WindowWidth = (int)(widthInCentimeters/Resolution*Magnification);
    WindowHeight = (int)(heighInCentimeters/Resolution*Magnification);

    // Draw a white background on the display:
    display.setColor(0xFFFFFF);
    display.fillRectangle(0, 0, WindowWidth, WindowHeight);

    aMap = new Map((int)(widthInCentimeters/Resolution), (int)(heighInCentimeters/Resolution), Resolution, binary);
  }
  
  // Add a single object point to the map
  void addObjectPoint(float x, float y) { 
    aMap.setObjectPoint(x, y);
    aMap.draw(display, Magnification);
  }
  
  // Apply a sensor model reading to the map
  void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance, double beamWidthInDegrees, double distanceErrorAsPercent) { 
    aMap.applySensorModelReading(sensorX, sensorY, sensorAngle,
                                 distance, beamWidthInDegrees, distanceErrorAsPercent);
    aMap.draw(display, Magnification);
  }
}
