import java.util.*;
import java.io.*;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class Map {

  private static final float ROBOT_RADIUS = 5.75f; // cm

  static final float[] SIGMA_PROB = { 0.0214f, 0.1359f, 0.3413f, 0.3413f, 0.1359f, 0.0214f };

  private float[][] tempGrid; // stores temporary sensor readings
  private float[][] occupancyGrid; // stores "occupied" ceryainty at each grid cell
  private int[] gridImage; // For display purposes only
  private int width;
  private int height;
  private double resolution;
  private float maxValue = 0.0f;
  private float minValue = 0.5f;
  private static int[] greyScalePalette = new int[256];

  private int DISPLAY_COUNTER = 0; // This is used to limit the number of times to display

  // Create a mao with the given with, height, resolution and whether or not it is
  // binary (otherwise gray scale)
  public Map(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    tempGrid = new float[width][height];
    occupancyGrid = new float[width][height];
    gridImage = new int[width * height];

    // Create Grayscale Palette
    for (int i = 0; i <= 255; i++)
      greyScalePalette[i] = (255 - i) * (0x10000 + 0x100 + 1);
  }

  // Returns true if the given point is within the occupancy grid boundaries and
  // false otherwise.
  boolean isInsideGrid(double x, double y) {
    return (((int) (x / resolution + width / 2) >= 0) && ((int) (y / resolution + height / 2) >= 0) &&
        ((int) (x / resolution + width / 2) < width) && ((int) (y / resolution + height / 2) < height));
  }

  // Set the value of the temp grid atthe given location to be the given amount
  void setTempGridValue(double x, double y, float amount) {
    tempGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)] = amount;
  }

  // Clear the area under the robot
  private void clearAroundRobot(double x, double y) {
    float inc = (float) Math.PI / 180.0f;
    for (float a = 0; a < Math.PI * 2; a += inc) {
      for (float d = 0; d < ROBOT_RADIUS; d += 0.5f) {
        float px = (float) (x + d * Math.cos(a));
        float py = (float) (y + d * Math.sin(a));
        occupancyGrid[(int) (px / resolution + width / 2)][(int) (py / resolution + height / 2)] = 0;
      }
    }
  }

  // Transfer all the tempGrid changes to the occupancyGrid
  void updateGrid(double rx, double ry) {
    for (int x = 0; x < width; x++)
      for (int y = 0; y < height; y++) {
        occupancyGrid[x][y] += tempGrid[x][y];
        if (occupancyGrid[x][y] < 0)
          occupancyGrid[x][y] = 0;
        if (occupancyGrid[x][y] > 255)
          occupancyGrid[x][y] = 255;
        tempGrid[x][y] = 0;
        if (occupancyGrid[x][y] > maxValue)
          maxValue = occupancyGrid[x][y];
        if (occupancyGrid[x][y] < minValue)
          minValue = occupancyGrid[x][y];
      }
    clearAroundRobot(rx, ry);
  }

  // Apply the sensor model
  public void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance,
      double beamWidthInDegrees, double distanceErrorAsPercent) {
    for (double r = 0; r < distance * (1 + distanceErrorAsPercent); r += 0.25) {
      double xa = sensorX + r * Math.cos(Math.toRadians(sensorAngle + beamWidthInDegrees / 2));
      double ya = sensorY + r * Math.sin(Math.toRadians(sensorAngle + beamWidthInDegrees / 2));
      double xb = sensorX + r * Math.cos(Math.toRadians(sensorAngle - beamWidthInDegrees / 2));
      double yb = sensorY + r * Math.sin(Math.toRadians(sensorAngle - beamWidthInDegrees / 2));

      double w = beamWidthInDegrees / Math.sqrt((xa - xb) * (xa - xb) + (ya - yb) * (ya - yb)) / 2;

      for (double a = -beamWidthInDegrees / 2; a <= beamWidthInDegrees / 2; a += w) {
        double x = sensorX + (r * Math.cos(Math.toRadians(sensorAngle + a)));
        double y = sensorY + (r * Math.sin(Math.toRadians(sensorAngle + a)));

        int angleIndex = (int) (((a + beamWidthInDegrees / 2) / beamWidthInDegrees) * 5.99);
        int distIndex = (int) (((r - distance * (1 - distanceErrorAsPercent)) / (2 * distance * distanceErrorAsPercent))
            * 5.99);

        if (isInsideGrid(x, y)) {
          float probability;
          if (r < distance * (1 - distanceErrorAsPercent)) {
            probability = -SIGMA_PROB[angleIndex];
          } else {
            probability = SIGMA_PROB[angleIndex] * SIGMA_PROB[distIndex];
          }
          setTempGridValue(x, y, probability);
        }

      }
    }
  }

  public void draw(Display aPen, int magnification) {
    if (DISPLAY_COUNTER++ % 250 != 0)
      return;
    try {
      // copy grid into image
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          if (occupancyGrid[x][height - 1 - y] != 0) {
            // double darkenFactor = 255;
            double darkenFactor = 512;
            if (maxValue != minValue)
              darkenFactor /= (maxValue - minValue);
            short shade = (short) (255 - occupancyGrid[x][height - 1 - y] * darkenFactor);
            if (shade < 0)
              shade = 0;
            gridImage[y * width + x] = 0xFF000000 + shade * 0x010000 + shade * 0x0100 + shade;
          } else
            gridImage[y * width + x] = 0xFFFFFFFF;
        }
      }
      ImageRef imageOfGrid = aPen.imageNew(width, height, gridImage, Display.ARGB);
      aPen.imagePaste(imageOfGrid, 0, 0, true);
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}
