package frc.robot.util;

import java.util.concurrent.Callable;

public class CoordinateSource {
  Callable<Double> xSource;
  Callable<Double> ySource;

  public CoordinateSource(Callable<Double> xSource, Callable<Double> ySource) {
    this.xSource = xSource;
    this.ySource = ySource;
  }

  public double getX() {
    try {
      return xSource.call().doubleValue();
    } catch (Exception e) {

      e.printStackTrace();
      return 0;
    }
  }

  public double getY() {
    try {
      return ySource.call().doubleValue();
    } catch (Exception e) {

      e.printStackTrace();
      return 0;
    }
  }
}
