// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.ShooterCommands.*;
import frc.robot.Commands.TurretCommands.GotoPosition;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;
import frc.robot.subsystems.Limelight.AprilTagSim;
import frc.robot.subsystems.Limelight.LimelightAiming;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOSparkMax;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.Turret.TurretIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.util.CoordinateSource;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel flywheel;
  private final Turret turret;

  private final Intake intake;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOSparkMax());
        flywheel = new Flywheel(new FlywheelIOSparkMax());
        turret = new Turret(new TurretIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
        // drive = new Drive(new DriveIOTalonFX());
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        turret = new Turret(new TurretIOSim());

        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        turret = new Turret(new TurretIO() {});
        intake = new Intake(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)", flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    Logger.recordOutput("apriltagPose", LimelightAiming.AprilTagPose);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    drive.setDefaultCommand(
        // Runs the left trigger of the drive controls
        Commands.run(
            () ->
                drive.CurvatureDrive(
                    ((controller.getRightTriggerAxis())),
                    applyDeadband(-controller.getLeftX() / 2)),
            drive));
    turret.setDefaultCommand(Commands.run(() -> turret.setTargetPosition(0), turret));
    // turret.setDefaultCommand( // closest implementation
    //     new GotoPosition(
    //         turret,
    //         new CoordinateSource(
    //             () -> LimelightAiming.getX(turret), () -> LimelightAiming.getY(turret))));
    controller
        .y()
        .whileTrue(
            Commands.run(
                () ->
                    turret.setTargetPosition(
                        AprilTagSim.calculateNewHeading(turret.getPosition())),
                turret));
    controller
        .rightStick()
        .whileTrue(
            new GotoPosition(
                turret, /*Math.atan2(controller.getLeftY(), controller.getLeftX())*/
                new CoordinateSource(controller::getRightX, controller::getRightY)));
    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    controller
        .b()
        .whileTrue(Commands.startEnd(() -> shooter.runVelocity(2500), shooter::stop, shooter));
    controller
        .x()
        .whileTrue(Commands.startEnd(() -> intake.runVelocity(2500), intake::stop, intake));
    controller
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () ->
                    drive.CurvatureDrive(
                        -controller.getLeftTriggerAxis(),
                        applyDeadband(controller.getLeftX() / 2))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * @param triggerValue ex: controller.getRightTriggerAxis()
   * @see deadband deadband is set at 0.15
   * @return returns 0 if below deadband and the trigger axis value if not
   */
  public static double applyDeadband(double triggerValue) {
    if (Math.abs(triggerValue) < .15) {
      return 0;
    } else {
      return triggerValue;
    }
  }
}
