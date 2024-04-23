// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeStart extends Command {
  //class variables
  private Intake intake;
  private double startTime = 0.0;
  public double currentTime = 0.0;
  /** Creates a new IntakeStart. */
  public IntakeStart(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

    //instance variables
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    intake.runVelocity(2500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    intake.runVelocity(25000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentTime-startTime > 5) { return true; }
    return false;
  }
}
