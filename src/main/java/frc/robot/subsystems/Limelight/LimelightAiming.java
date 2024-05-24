// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightAiming {
  public static final Pose2d AprilTagPose = new Pose2d().rotateBy(new Rotation2d().fromDegrees(90));
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("");
  /**
   * @return heading of april tag
   */
  public double getRealheading() { // this is not tested
    double tx = table.getEntry("tx").getDouble(0);
    return 90 - tx;
  }
  /**
   * @return distance to target in inches
   */
  public double getDistanceToTarget() {
    double ty = table.getEntry("ty").getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; // TODO: add in value/constant for our robot

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; // TODO: add in value/constant for our robot

    // distance from the target to the floor
    double goalHeightInches = 60.0; // TODO: add in constant/value

    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches =
        (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }
}
