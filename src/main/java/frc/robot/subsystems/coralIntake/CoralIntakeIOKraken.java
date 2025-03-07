package frc.robot.subsystems.coralIntake;

import static frc.robot.subsystems.coralIntake.CoralIntakeConstants.*;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.TalonFXUtil.MotorFactory;

public class CoralIntakeIOKraken implements CoralIntakeIO {
    private MotorFactory motorFactory;
    private final TalonFX intakeMotor; 
    private final DigitalInput gamePieceSensor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true).withSlot(0).withAcceleration(motorAcceleration);
    private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
    private final NeutralOut stopMode = new NeutralOut();

    public CoralIntakeIOKraken(int motorID, int sensorChannel) {
        motorFactory = new MotorFactory("CoralIntake", motorID);

        motorFactory.setBrakeMode(true);
        motorFactory.setInverted(true);
        motorFactory.setCurrentLimits(40);
        motorFactory.setVoltageLimits(8);
        motorFactory.setSlot0(0.15, 0, 0.0025);
        motorFactory.setSlot0(0.5);

        motorFactory.configureMotors();
        intakeMotor = motorFactory.getMotors()[0];

        gamePieceSensor = new DigitalInput(sensorChannel);
    }

    @Override
    public void updateInputs(CoralIntakeIOInputs inputs) {
        inputs.motorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = intakeMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorAccel = intakeMotor.getAcceleration().getValueAsDouble();

        inputs.gamePieceSensor = !gamePieceSensor.get();

        // motorFactory.checkForUpdates();
    }

    @Override
    public void runMotor(double velocity) {
        intakeMotor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void runVoltage(double volts) {
        intakeMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stopMotor() {
        intakeMotor.setControl(stopMode);
    }
}
