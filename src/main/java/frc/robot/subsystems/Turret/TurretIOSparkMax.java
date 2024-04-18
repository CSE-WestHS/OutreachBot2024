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

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class TurretIOSparkMax implements TurretIO {
  private static final double GEAR_RATIO = Constants.TURRET_GEAR_RATIO;

  private final CANSparkMax turret = new CANSparkMax(Constants.TURRET_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = turret.getEncoder();
  private final SparkPIDController pid = turret.getPIDController();

  public TurretIOSparkMax() {
    turret.restoreFactoryDefaults();

    turret.setCANTimeout(250);

    turret.setInverted(false);

    turret.enableVoltageCompensation(12.0);
    turret.setSmartCurrentLimit(30);

    turret.burnFlash();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.positionRad = turret.getEncoder().getPosition() / GEAR_RATIO * Math.PI * 2;
    inputs.appliedVolts = turret.getAppliedOutput() * turret.getBusVoltage();
    inputs.currentAmps = new double[] {turret.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    turret.setVoltage(volts);
  }

  // @Override
  public void setPosition(float posRads) {
    pid.setReference(posRads / Math.PI * 2, ControlType.kPosition, 0);
  }

  @Override
  public void stop() {
    turret.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
