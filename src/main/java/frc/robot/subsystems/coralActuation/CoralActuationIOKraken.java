package frc.robot.subsystems.coralActuation;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil.MotorFactory;

public class CoralActuationIOKraken implements CoralActuationIO {
    private MotorFactory motorFactory;
    private TalonFX coralActuationMotor;

    private MotionMagicVoltage motionMagicControl;
    private VoltageOut voltageControl;
    private NeutralOut stopMode;

    public CoralActuationIOKraken(int motorID) {
        motorFactory = new MotorFactory("coralActuation", motorID);

        motorFactory.setBrakeMode(true);
        motorFactory.setInverted(false);
        motorFactory.setCurrentLimits(40);
        motorFactory.setVoltageLimits(8);

        motorFactory.setMotionMagic(10, 25, 35);
        motorFactory.setSlot0(0.01, 0, 0);

        motorFactory.configureMotors();

        coralActuationMotor = motorFactory.getMotors()[0];

        motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        voltageControl = new VoltageOut(0).withEnableFOC(true);
        stopMode = new NeutralOut();
    }

    @Override
    public void updateInputs(CoralActuationIOInputs inputs) {
        inputs.motorPos = coralActuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = coralActuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = coralActuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = coralActuationMotor.getMotorVoltage().getValueAsDouble();

        // motorFactory.checkForUpdates();
    }

    @Override
    public void moveToPos(double pos) {
        coralActuationMotor.setControl(motionMagicControl.withPosition(pos));
    }

    @Override
    public void runVoltage(double voltage) {
        coralActuationMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void stopMotor() {
        coralActuationMotor.setControl(stopMode);
    }
}

