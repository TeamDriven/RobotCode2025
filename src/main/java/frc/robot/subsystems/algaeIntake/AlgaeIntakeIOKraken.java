package frc.robot.subsystems.algaeIntake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil.MotorFactory;

public class AlgaeIntakeIOKraken implements AlgaeIntakeIO{
    private MotorFactory motorFactory;
    private TalonFX intakeMotor;

    private VelocityVoltage velocityControl;
    private NeutralOut StopMode;

    public AlgaeIntakeIOKraken(int motorID) {
        motorFactory = new MotorFactory("algaeIntake", motorID);

        motorFactory.setBrakeMode(true);
        motorFactory.setInverted(true);
        motorFactory.setCurrentLimits(50);
        motorFactory.setVoltageLimits(8);
        motorFactory.setSlot0(0.025, 0, 0);

        motorFactory.configureMotors();
        intakeMotor = motorFactory.getMotors()[0];

        velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
        StopMode = new NeutralOut();
    }

    @Override
    public void updateInputs(AlgaeIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorAccel = intakeMotor.getAcceleration().getValueAsDouble();

        // motorFactory.checkForUpdates();
    }

    @Override
    public void runVelocity(double velocity) {
        intakeMotor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void stopMotors() {
        intakeMotor.setControl(StopMode);
    }
}
