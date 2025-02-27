// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralActuation;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralActuation extends SubsystemBase {
  private CoralActuationIO coralActuationIO;
  private CoralActuationIOInputsAutoLogged inputs = new CoralActuationIOInputsAutoLogged();

  private enum mode {
    POSITION,
    VOLTAGE,
    STOPPED;
  }

  private mode currentMode = mode.STOPPED;

  private double value = 0;

  /** Creates a new CoralActuation. */
  public CoralActuation(CoralActuationIO coralActuationIO) {
    this.coralActuationIO = coralActuationIO;
  }

  @Override
  public void periodic() {
    coralActuationIO.updateInputs(inputs);
    Logger.processInputs("CoralActuation", inputs);

    Logger.recordOutput("CoralActuation/mode", currentMode);
    Logger.recordOutput("CoralActuation/value", value);

    switch (currentMode) {
      case POSITION:
        coralActuationIO.seedMotor();
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
}
