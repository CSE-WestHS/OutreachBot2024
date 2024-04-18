package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class IncrementPositionLeft extends Command {
  Turret turret;
  private final double TURRETINCREMENT = 0.0174533;

  public IncrementPositionLeft(Turret turret) {
    addRequirements(turret);
    this.turret = turret;
  }

  public boolean isFinished() {
    return Math.abs(turret.getPosition()) >= 1;
    // return false;
  }

  public void execute() {
    if (Math.abs(turret.getPosition()) >= 1) {
      isFinished();
    }
    turret.setTargetPosition(turret.getPosition() + TURRETINCREMENT);
  }

  public void end() {}
}
