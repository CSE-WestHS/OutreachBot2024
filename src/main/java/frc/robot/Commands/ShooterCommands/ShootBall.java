// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
// command to shoot without april tag aiming

public class ShootBall extends Command {
  private Shooter shooter;
  double currentTime = 0;
  double startTime = 0;
  /** Creates a new ShootBall. */
  public ShootBall(Shooter newShooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = newShooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    shooter.runVelocity(2500);
    Timer.delay(0.5); // not sure if this is bad or not
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if run too long it will stop
    if (currentTime - startTime > 5) {
      return true;
    }
    return false;
  }
}
