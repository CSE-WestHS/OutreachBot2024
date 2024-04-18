package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;

public class GotoPosition extends Command {
  Turret turret;
  double position;

  public GotoPosition(Turret turret, double position) {
    addRequirements(turret);
    this.turret = turret;
    this.position = position;
  }

  public boolean isFinished() {
    return Math.abs(turret.getPosition() - position) < Constants.TURRET_ROTATION_TOLERANCE;
  }

  public void execute() {
    turret.setTargetPosition(position);
  }
}
