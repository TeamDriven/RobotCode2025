package frc.robot.subsystems.algaeActuation;

import static frc.robot.subsystems.algaeActuation.AlgaeActuationConstants.rotationsToDegrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.TalonFXUtil;
import frc.robot.util.TalonFXUtil.ConfigFactory;
import frc.robot.util.TalonFXUtil.MotorFactory;

public class AlgaeActuationIOKraken implements AlgaeActuationIO {
    private MotorFactory motorFactory;
    private TalonFX actuationMotor;

    private MotionMagicVoltage motionMagicControl;
    private VoltageOut voltageControl;
    private NeutralOut StopMode;

    public AlgaeActuationIOKraken(int motorID) {
        motorFactory = new MotorFactory("algaeActuation", motorID);

        motorFactory.setBrakeMode(true);
        motorFactory.setInverted(true);
        motorFactory.setCurrentLimits(60);
        motorFactory.setVoltageLimits(8);

        motorFactory.setSlot0(0.01, 0, 0);
        motorFactory.setMotionMagic(10, 25, 35);

        motorFactory.configureMotors();

        actuationMotor = motorFactory.getMotors()[0];

        motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        voltageControl = new VoltageOut(0).withEnableFOC(true);
        StopMode = new NeutralOut();
    }

    @Override
    public void updateInputs(AlgaeActuationIOInputs inputs) {
        inputs.motorPos = actuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = actuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = actuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = actuationMotor.getMotorVoltage().getValueAsDouble();

        motorFactory.checkForUpdates();
    }

    @Override
    public void moveToPos(double pos) {
        actuationMotor.setControl(motionMagicControl.withPosition(pos / rotationsToDegrees));
    }

    @Override
    public void runVoltage(double voltage) {
        actuationMotor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void stopMotors() {
        actuationMotor.setControl(StopMode);
    }
}
