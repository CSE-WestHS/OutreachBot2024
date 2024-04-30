package frc.robot.Commands.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.CoordinateSource;
import frc.robot.util.PositionMode;
import java.util.concurrent.Callable;

public class GotoPosition extends Command {
  Turret turret;
  double position;
  CoordinateSource controller;
  PositionMode mode;
  Callable<Double> robotRotation;

  public GotoPosition(
      Turret turret,
      CoordinateSource controller,
      PositionMode mode,
      Callable<Double> robotRotation) {
    addRequirements(turret);
    this.turret = turret;
    this.controller = controller;
    this.mode = mode;
    if (!(robotRotation == null)) {
      this.robotRotation = robotRotation;
    } else {
      robotRotation =
          () -> {
            return 0.0;
          };
    }
  }
  // New Default, Never Ends^tm
  // public boolean isFinished() {
  //  return Math.abs(turret.getPosition() - position) < Constants.TURRET_ROTATION_TOLERANCE;
  // }

  public void execute() {
    boolean passesDeadband =
        Math.abs(controller.getY()) > Constants.TURRET_DEADBAND
            || Math.abs(controller.getX()) > Constants.TURRET_DEADBAND;
    if (passesDeadband) {
      switch (mode) {
        case ROBOT_RELATIVE:
          position =
              Math.atan2(controller.getY() * -1, controller.getX()) + Constants.TURRET_ZERO_OFFSET;
          break;
        case FIELD_RELATIVE:
          try {
            position =
                Math.atan2(controller.getY() * -1, controller.getX())
                    + Constants.TURRET_ZERO_OFFSET
                    - robotRotation.call();
          } catch (Exception e) {
            e.printStackTrace();
          }
          break;
      }
    }

    turret.setTargetPosition(position);
  }
}
