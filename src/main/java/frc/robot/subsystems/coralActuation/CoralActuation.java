// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralActuation;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class CoralActuation extends SubsystemBase {
  private CoralActuationIO coralActuationIO;
  private CoralActuationIOInputsAutoLogged inputs = new CoralActuationIOInputsAutoLogged();

  private LoggedTunableNumber toleranceTime = new LoggedTunableNumber("CoralActuation/toleranceTime", 0.1);
  private LoggedTunableNumber tolerance = new LoggedTunableNumber("CoralActuation/tolerance", 1);

  private Timer toleranceTimer = new Timer();

  private Timer brakeTimer = new Timer();
  private boolean brakeMode = true;

  private enum mode {
    POSITION,
    VOLTAGE,
    STOPPED;
  }

  private mode currentMode = mode.POSITION;

  private double value = 0;

  /** Creates a new CoralActuation. */
  public CoralActuation(CoralActuationIO coralActuationIO) {
    this.coralActuationIO = coralActuationIO;

    toleranceTimer.start();
    brakeTimer.start();
  }

  @Override
  public void periodic() {
    coralActuationIO.updateInputs(inputs);
    Logger.processInputs("CoralActuation", inputs);

    Logger.recordOutput("CoralActuation/mode", currentMode);
    Logger.recordOutput("CoralActuation/value", value);

    if (DriverStation.isEnabled()) {
      brakeTimer.reset();
      if (!brakeMode) {
        coralActuationIO.setBrakeMode(true);
        brakeMode = true;
      }
    } else if (brakeTimer.hasElapsed(5) && brakeMode) {
      coralActuationIO.setBrakeMode(false);
      brakeMode = false;
    }

    switch (currentMode) {
      case POSITION:
        if (!MathUtil.isNear(value, inputs.relativeEncoderPos.getDegrees(), tolerance.get())) {
          toleranceTimer.reset();
        }
        coralActuationIO.moveToPos(value);
        break;
      case VOLTAGE:
        coralActuationIO.runVoltage(value);
        break;
      case STOPPED:
        coralActuationIO.stopMotor();
        break;
    }
  }

  public void setPos(double pos) {
    currentMode = mode.POSITION;
    value = pos;
    coralActuationIO.seedMotor(inputs.relativeEncoderPos);
  }

  public void stop() {
    currentMode = mode.STOPPED;
    value = 0;
  }

  public void runVoltage(double volts) {
    currentMode = mode.VOLTAGE;
    value = volts;
  }
  
  public Command runVoltageCommand(double volts) {
    return Commands.startEnd(() -> runVoltage(volts), () -> stop(), this);
  }

  public Command runVoltageCommand(DoubleSupplier volts) {
    return Commands.startEnd(() -> runVoltage(volts.getAsDouble()), () -> stop(), this);
  }

  public boolean isAtAngle() {
        return toleranceTimer.hasElapsed(toleranceTime.get());
    }
}
