// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.*;
import frc.robot.subsystems.drive.*;

/** Add your docs here. */
public class LimelightLocalization {
  private static boolean doRejectUpdate = false;
  private static SimpleMatrix storage = new SimpleMatrix(1, 3);
  private static PoseEstimator poseEst =
      new PoseEstimator(Drive.kinematics, Drive.odometry, new Matrix<>(storage), new Matrix<>(storage));

  public static PoseEstimator getEstimatedPose() {
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    LimelightHelpers.SetRobotOrientation(
        "limelight", poseEst.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    // if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees
    // per second, ignore vision updates
    // {
    //   doRejectUpdate = true;
    // }
    if (!doRejectUpdate) {
      poseEst.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      poseEst.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
    return poseEst;
  }
}
