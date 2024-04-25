package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.CoordinateSource;

public class GotoPosition extends Command {
  Turret turret;
  double position;
  CoordinateSource controller;

  public GotoPosition(Turret turret, CoordinateSource controller) {
    addRequirements(turret);
    this.turret = turret;
    this.controller = controller;
  }
  // New Default, Never Ends^tm

  @Override
  public void execute() {
    boolean passesDeadband =
        Math.abs(controller.getY()) > Constants.TURRET_DEADBAND
            || Math.abs(controller.getX()) > Constants.TURRET_DEADBAND;
    if (passesDeadband)
      position =
          Math.atan2(controller.getY() * -1, controller.getX()) + Constants.TURRET_ZERO_OFFSET;
    turret.setTargetPosition(position);
  }
}
