// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class AprilTagSim {
  public static Pose2d apriltag = LimelightAiming.AprilTagPose;
  /**
   * @return pose of april tag for easy use!
   */
  public static double getPose() {
    return apriltag.getRotation().getDegrees();
  }
  /**
   * @param currentAngle
   * @return angle between [-180,180)
   * @see degrees make sure that angle is in degrees
   */
  public static double getRestrictedAngle(double currentAngle) {
    if (currentAngle <= 180 || currentAngle >= -179) {
      return currentAngle;
    }
    currentAngle %= 360;
    currentAngle = (currentAngle + 360) % 360;
    if (currentAngle > 180) {
      currentAngle -= 360;
    }
    return currentAngle;
  }
  /**
   * @param currentHeading
   * @return calculated heading based on april tag rotation
   * @see degrees use degrees when passing in current heading
   */
  public static double calculateNewHeading(double currentHeading, Turret turret) {

    // get current heading
    double atRotation = getPose();
    double angleOffset =
        Units.degreesToRadians(
            -(Drive.odometry.getPoseMeters().getRotation().getDegrees() - atRotation));
    // get haeding difference
    currentHeading = getRestrictedAngle(currentHeading);
    double headingDiff = atRotation - currentHeading;
    // get heading that we need to head to
    if (Math.abs(headingDiff) < 30) {
      return Units.degreesToRadians(currentHeading);
    } else {
      if (headingDiff > 0) {
        return Units.degreesToRadians(currentHeading + headingDiff) + angleOffset;

      } else if (headingDiff < 0) {
        return Units.degreesToRadians(currentHeading + -headingDiff) - angleOffset;
      } else {
        return Units.degreesToRadians(currentHeading);
      }
    }
  }
}
