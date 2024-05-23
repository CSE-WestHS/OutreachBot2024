// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
    //checks if angle is within bounds
    if (currentAngle <= 180 || currentAngle >= -179) { 
      return currentAngle;
    }
    //if not: get it there
    currentAngle %= 360;
    currentAngle = (currentAngle + 360) % 360;
    if (currentAngle > 180) {
      currentAngle -= 360;
    }
    //return current angle
    return currentAngle;
  }
  /**
   * @param currentHeading
   * @return calculated heading based on april tag rotation
   * @see degrees use degrees when passing in current heading
   */
  public static double calculateNewHeading(double currentHeading) {
    //angle tolerance for turret accuracy
    double tolerance = 15;
    // get current heading
    double atRotation = getPose();
    //sets offset to the difference between the current drive heading and the april tag
    //heading. Then makes the magnitude opposite.
    //This is because the drive odometry is related to the turret odometry.
    double angleOffset =
        Units.degreesToRadians(
            -(Drive.odometry.getPoseMeters().getRotation().getDegrees() - atRotation));
    // get haeding difference
    currentHeading = getRestrictedAngle(currentHeading);
    //calculates heading difference
    double headingDiff = atRotation - currentHeading;
    // get heading that we need to head to

    //checks if heading difference is within tolerance
    if ((headingDiff) < tolerance) {
      return Units.degreesToRadians(currentHeading); //returns current heading
    } else {
      if (headingDiff > 0) {//checks if angle is above target
        return Units.degreesToRadians(currentHeading + headingDiff) + angleOffset; //changes heading by the difference and offset
      } else if (headingDiff < 0) { //checks if magnitude is negative -- meaning it is below target angle
        return Units.degreesToRadians(currentHeading + -headingDiff) - angleOffset; //changes heading by the difference and offset
      } else { //if it is equal return current heading
        return Units.degreesToRadians(currentHeading);
      }
    }
  }
}
