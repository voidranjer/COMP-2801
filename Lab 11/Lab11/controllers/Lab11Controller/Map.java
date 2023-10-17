import java.util.*;
import java.io.*;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class Map {

  private short[][] occupancyGrid;
  private int[] gridImage; // For display purposes only
  private int width;
  private int height;
  private double resolution;
  private boolean isBinary;
  private short maxValue = 0;
  private short minValue = 0;

  private int DISPLAY_COUNTER = 0; // This is used to limit the number of times to display

  // Create a mao with the given with, height, resolution and whether or not it is
  // binary (otherwise gray scale)
  public Map(int w, int h, double res, boolean binary) {
    width = w;
    height = h;
    resolution = res;
    occupancyGrid = new short[width][height];
    gridImage = new int[width * height];
    isBinary = binary;
  }

  // Returns true if the given point is within the occupancy grid boundaries and
  // false otherwise.
  boolean isInsideGrid(double x, double y) {
    return (((int) (x / resolution + width / 2) >= 0) && ((int) (y / resolution + height / 2) >= 0) &&
        ((int) (x / resolution + width / 2) < width) && ((int) (y / resolution + height / 2) < height));
  }

  // Increase the value at the grid by one
  void increaseGridValue(double x, double y) {
    short value = occupancyGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)];
    occupancyGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)] = ++value;
    if ((value > maxValue) && (value <= 255))
      maxValue = value;
    if (value < minValue)
      minValue = value;
  }

  // Decrease the value at the grid by one
  void decreaseGridValue(double x, double y) {
    short value = occupancyGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)];
    occupancyGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)] = --value;
    if ((value > maxValue) && (value <= 255))
      maxValue = value;
    if (value < minValue)
      minValue = value;
  }

  // Set the value at the grid to the given value
  void setGridValue(double x, double y, short value) {
    occupancyGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)] = value;
  }

  /**************************************************************************************************************/
  /* Apply the sensor model */
  /**************************************************************************************************************/
  public double cos(double angle) {
    return Math.cos(Math.toRadians(angle));
  }

  public double sin(double angle) {
    return Math.sin(Math.toRadians(angle));
  }

  public void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance,
      double beamWidthInDegrees, double distanceErrorAsPercent) {
    double xs = sensorX;
    double ys = sensorY;
    double d = distance;
    double phi = sensorAngle;
    double sigma = beamWidthInDegrees;

    // Slide 23
    double xa = xs + d * cos(phi + sigma / 2);
    double ya = ys + d * sin(phi + sigma / 2);
    double xb = xs + d * cos(phi - sigma / 2);
    double yb = ys + d * sin(phi - sigma / 2);

    // Slide 24
    double firstTemp = Math.pow(xa - xb, 2);
    double secondTemp = Math.pow(ya - yb, 2);
    double omega = sigma / Math.sqrt(firstTemp + secondTemp);

    for (double a = -sigma / 2; a <= sigma / 2; a += omega) {
      double objX = xs + (d * cos(phi + a));
      double objY = ys + (d * sin(phi + a));

      if (isInsideGrid(objX, objY)) {
        if (isBinary)
          setGridValue(objX, objY, (short) 1);
        else
          increaseGridValue(objX, objY);
      }
    }

    // Slide 26
    double epsilon = distanceErrorAsPercent;
    final double INC = 0.25;
    for (double r = d * (1 - epsilon); r <= d * (1 + epsilon); r += INC) {
      xa = xs + (r * cos(phi + sigma / 2));
      ya = ys + (r * sin(phi + sigma / 2));
      xb = xs + (r * cos(phi - sigma / 2));
      yb = ys + (r * sin(phi - sigma / 2));

      firstTemp = Math.pow(xa - xb, 2);
      secondTemp = Math.pow(ya - yb, 2);
      omega = (sigma / Math.sqrt(firstTemp + secondTemp)) / 2;

      for (double a = -sigma / 2; a <= sigma / 2; a += omega) {
        double objX = xs + (r * cos(phi + a));
        double objY = ys + (r * sin(phi + a));

        if (isInsideGrid(objX, objY)) {
          if (isBinary)
            setGridValue(objX, objY, (short) 1);
          else
            increaseGridValue(objX, objY);
        }
      }
    }
  }

  // Set one point in the grid to 1 or increase by 1
  public void setObjectPoint(float x, float y) {
    if (isInsideGrid(x, y)) {
      if (isBinary)
        setGridValue(x, y, (short) 1);
      else {
        increaseGridValue(x, y);
      }
    }
  }

  // Set one point in the grid to 0 or decrease by 1
  public void resetObjectPoint(int x, int y) {
    if (isInsideGrid(x, y)) {
      if (isBinary)
        setGridValue(x, y, (short) 0);
      else {
        decreaseGridValue(x, y);
      }
    }
  }

  // Draw the Occupancy Grid. Do Not Change This Code.
  public void draw(Display aPen, int magnification) {
    if (DISPLAY_COUNTER++ % 250 != 0)
      return;
    if (isBinary)
      aPen.setColor(0x000000); // black
    try {
      // copy grid into image
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          if (occupancyGrid[x][height - 1 - y] != 0) {
            if (!isBinary) {
              double darkenFactor = 255;
              if (maxValue != minValue)
                darkenFactor /= (maxValue - minValue);
              short shade = (short) (255 - occupancyGrid[x][height - 1 - y] * darkenFactor);
              if (shade < 0)
                shade = 0;
              gridImage[y * width + x] = 0xFF000000 + shade * 0x010000 + shade * 0x0100 + shade;
            } else
              gridImage[y * width + x] = 0xFF000000;
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
