// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class LimelightAiming {
  public static final Pose2d AprilTagPose = new Pose2d().rotateBy(new Rotation2d().fromDegrees(90));
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("");
  private static double lastTurret = 0;
  private static PIDController pidController = new PIDController(0.05, 0, 0);
  /**
   * @return april tag heading
   */
  public static double getAprilTagHeading() {
    double xOffset = table.getValue("tx").getDouble();
    if (xOffset == -1) { return 90; }
    else { return 90 - xOffset; }
  }
  /**
   * @param turret object
   * @see Notes april tag pose > 0 is recommended
   * @return heading for simulated april tag
   */
  public static double getAprilTagHeading(Turret turret) {
    double turretangle = turret.getPosition();
    // if above 360 in degrees the turret is set as 0 for below data to work
    if (Units.radiansToDegrees(turret.getPosition()) >= 360) {
      turret.setTargetPosition(0);
      turretangle = turret.getPosition();
    }
    // if the heading is within 30 degrees away from the opposite of the heading of apriltagpose it
    // sets turret position to point toward april tag
    if (Math.abs(
                -Drive.odometry.getPoseMeters().getRotation().getDegrees()
                    - AprilTagPose.getRotation().getDegrees())
            < 30
        && Drive.odometry.getPoseMeters().getY() > AprilTagPose.getX()) {
      if (lastTurret == turretangle) {
        return -(turretangle - (1.571 / 400));
      } else {
        return turretangle;
      }
      // if facing the same heading as the april tag: turn 180
    } else if (Math.abs(
                Drive.odometry.getPoseMeters().getRotation().getDegrees()
                    - AprilTagPose.getRotation().getDegrees())
            <= 30
        && Drive.odometry.getPoseMeters().getY() > AprilTagPose.getX() // rotated so different
        && Math.abs(turretangle - -Drive.odometry.getPoseMeters().getRotation().getRadians())
            <= 30) {
      if (lastTurret == turretangle) {
        lastTurret = turretangle;
        return turretangle - Units.degreesToRadians(180);
      } else {
        lastTurret = turretangle;
        return (turretangle);
      }
    }
    if (Math.abs(
                -Drive.odometry.getPoseMeters().getRotation().getDegrees()
                    - AprilTagPose.getRotation().getDegrees())
            >= 180
        && Drive.odometry.getPoseMeters().getY() < AprilTagPose.getX() // rotated so different
    ) {
      lastTurret = turretangle;
      return turretangle - Units.degreesToRadians(180);
    } else if (Drive.odometry.getPoseMeters().getX() > AprilTagPose.getY()
        && Math.abs(
                Drive.odometry.getPoseMeters().getRotation().getDegrees()
                    - AprilTagPose.getRotation().getDegrees())
            < 30) {
      lastTurret = turretangle;
      return turretangle - Units.degreesToRadians(90);
    } else {
      return turretangle + Units.degreesToRadians(-90);
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
