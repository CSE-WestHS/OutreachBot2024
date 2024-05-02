// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class LimelightAiming {
  public static final Pose2d AprilTagPose = new Pose2d();
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("");
  /**
   * @return april tag heading
   */
  public static double getAprilTagHeading() {
    double xOffset = table.getValue("tx").getDouble();
    return 90 - xOffset;
  }

  public static double getAprilTagHeading(Turret turret) {
    double turretangle = turret.getPosition();
    if (Units.radiansToDegrees(turret.getPosition()) > 360) {
      turretangle = Units.degreesToRadians((360 * (turret.getPosition() % 360)));
    }
    if (Math.abs(Drive.odometry.getPoseMeters().getRotation().getDegrees()
            - AprilTagPose.getRotation().getDegrees())
        < 15) {
          return turretangle;
    }
    else {
      if (Math.abs(Drive.odometry.getPoseMeters().getRotation().getDegrees() - AprilTagPose.getRotation().getDegrees()) >= 180){
        return AprilTagPose.getRotation().getRadians() - Units.degreesToRadians(180);
      }
      else if (/*fix */true) {

      }
    }
  }

  /**
   * @return distance to target
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
