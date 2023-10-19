import java.io.*;
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
  
  // Apply a sensor model reading to the map
  void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance, double beamWidthInDegrees, double distanceErrorAsPercent) { 
    aMap.applySensorModelReading(sensorX, sensorY, sensorAngle,
                                  distance, beamWidthInDegrees, distanceErrorAsPercent);
    aMap.draw(display, Magnification);
  }
  
  // Update the grid by applying the temp grid changes to the occupancy grid
  void updateGrid(double x, double y) { 
    aMap.updateGrid(x, y);
    aMap.draw(display, Magnification);
  }
}
