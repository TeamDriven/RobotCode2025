// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeActuation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeActuation extends SubsystemBase {
  private final AlgaeActuationIO actuationIO;
  private final AlgaeActuationIOInputsAutoLogged activationInputs = new AlgaeActuationIOInputsAutoLogged();

  private enum mode {
    POSITION,
    VOLTAGE,
    STOPPED;
  }

  private mode currentMode = mode.STOPPED;

  private double value = 0;

  public AlgaeActuation(AlgaeActuationIO actuationIO) {
    this.actuationIO = actuationIO;
  }

  @Override
  public void periodic() {
    actuationIO.updateInputs(activationInputs);
    Logger.processInputs("algaeActuation", activationInputs);

    switch (currentMode) {
      case POSITION:
        actuationIO.moveToPos(value);
      case VOLTAGE:
        actuationIO.runVoltage(value);
      case STOPPED:
        actuationIO.stopMotors();
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
}
