package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;

public class IncrementPosition extends Command {
  Turret turret;
  double position;

  public IncrementPosition(Turret turret) {
    addRequirements(turret);
    this.turret = turret;
  }

  public boolean isFinished() {
    return Math.abs(turret.getPosition() - position) < Constants.TURRET_ROTATION_TOLERANCE;
  }

  public void execute() {
    turret.setTargetPosition(turret.getPosition() + 0.0174533);
  }

  public void end() {}
}
