package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil.MotorFactory;

public class CoralIntakeIOKraken implements CoralIntakeIO {
    private MotorFactory motorFactory;
    private final TalonFX intakeMotor; 

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withAcceleration(motorAcceleration);
    private final NeutralOut stopMode = new NeutralOut();

    public CoralIntakeIOKraken(int motorID) {
        motorFactory = new MotorFactory("coralIntake", motorID);

        motorFactory.setBrakeMode(true);
        motorFactory.setInverted(false);
        motorFactory.setCurrentLimits(40);
        motorFactory.setVoltageLimits(8);
        motorFactory.setSlot0(0.01, 0, 0);

        motorFactory.configureMotors();
        intakeMotor = motorFactory.getMotors()[0];
    }

    @Override
    public void updateInputs(CoralIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorAccel = intakeMotor.getAcceleration().getValueAsDouble();

        // motorFactory.checkForUpdates();
    }

    @Override
    public void runMotor(double velocity) {
        intakeMotor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void stopMotor() {
        intakeMotor.setControl(stopMode);
    }
}
