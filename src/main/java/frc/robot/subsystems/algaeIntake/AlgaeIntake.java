// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algaeIntake;

import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.inSpeed;
import static frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants.outSpeed;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private final AlgaeIntakeIO intakeIO;
  private final AlgaeIntakeIOInputsAutoLogged intakeInputs = new AlgaeIntakeIOInputsAutoLogged();

  private double velocity = 0;

  public AlgaeIntake(AlgaeIntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("algaeIntake", intakeInputs);

    if (velocity == 0) {
      intakeIO.stopMotors();
    } else {
      intakeIO.runVelocity(velocity);
    }
  }

  public void runVelocity(double vel) {
    velocity = vel;
  }
}
