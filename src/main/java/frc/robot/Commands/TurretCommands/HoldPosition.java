package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.PositionMode;

public class HoldPosition extends Command {
  Turret turret;
  PositionMode mode;
  double position;
  /** Only robot relative for now, sorry. */
  public HoldPosition(PositionMode mode, Turret turret) {
    this.mode = mode;
    addRequirements(turret);
    this.turret = turret;
  }

  public void initialize() {
    position = turret.getPosition();
  }

  public void execute() {
    turret.setTargetPosition(position);
  }
}
