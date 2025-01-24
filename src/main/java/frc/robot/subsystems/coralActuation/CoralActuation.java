// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralActuation;

import static frc.robot.subsystems.coralActuation.CoralActuationConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralActuation extends SubsystemBase {
  private CoralActuationIO coralActuationIO;
  private CoralActuationIOInputsAutoLogged inputs = new CoralActuationIOInputsAutoLogged();

  private double position = startPos;
  private boolean stopped = false;
  private double voltage = 0;
  private boolean isPositionControl = true;

  /** Creates a new CoralActuation. */
  public CoralActuation(CoralActuationIO coralActuationIO) {
    this.coralActuationIO = coralActuationIO;
  }

  @Override
  public void periodic() {
    coralActuationIO.updateInputs(inputs);
    Logger.processInputs("coralActuation", inputs);

    if (stopped == true) {
      coralActuationIO.stopMotor();
      return;
    }

    if (isPositionControl == true) {
      coralActuationIO.moveToPos(position);
    } else {
      coralActuationIO.runVoltage(voltage);
    }
  }

  public void setPos(double pos) {
    isPositionControl = true;
    position = pos;
  }

  public void stop() {
    stopped = true;
  }

  public void runVoltage(double volts) {
    isPositionControl = false;
    voltage = volts;
  }
}
