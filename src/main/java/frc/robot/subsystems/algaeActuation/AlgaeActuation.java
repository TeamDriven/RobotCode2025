// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeActuation;

import static frc.robot.subsystems.algaeActuation.AlgaeActuationConstants.tuckPos;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeActuation extends SubsystemBase {
  private final AlgaeActuationIO activationIO;
  private final AlgaeActuationIOInputsAutoLogged activationInputs = new AlgaeActuationIOInputsAutoLogged();

  private double position = tuckPos;
  private boolean stopped = false;
  private double voltage = 0;
  private boolean isPositionControl = true;

  public AlgaeActuation(AlgaeActuationIO activationIO) {
    this.activationIO = activationIO;
  }

  @Override
  public void periodic() {
    activationIO.updateInputs(activationInputs);
    Logger.processInputs("algaeActuation", activationInputs);

    if (stopped == true) {
      activationIO.stopMotors();
      return;
    }

    if (isPositionControl == true) {
      activationIO.moveToPos(position);
    } else {
      activationIO.runVoltage(voltage);
    }
  }

  public void setPos(double pos) {
    stopped = false;
    isPositionControl = true;
    position = pos;
  }

  public void stop() {
    stopped = true;
  }

  public void runVoltage(double volt) {
    stopped = false;
    isPositionControl = false;
    voltage = volt;
  }
}
