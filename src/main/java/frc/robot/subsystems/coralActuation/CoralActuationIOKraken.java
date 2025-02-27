package frc.robot.subsystems.coralActuation;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.TalonFXUtil.MotorFactory;

public class CoralActuationIOKraken implements CoralActuationIO {
    private MotorFactory motorFactory;
    private TalonFX coralActuationMotor;

    private MotionMagicVoltage motionMagicControl;
    private VoltageOut voltageOut;
    private NeutralOut stopMode;
    private DutyCycleEncoder encoder;

    public CoralActuationIOKraken(int motorID, int encoderChannel) {
        motorFactory = new MotorFactory("CoralActuation", motorID);

        motorFactory.setInverted(false);
        motorFactory.setBrakeMode(true);
        motorFactory.setVoltageLimits(8);
        motorFactory.setCurrentLimits(40);

        motorFactory.setSlot0(3, 0, 0, 0);
        motorFactory.setMotionMagic(1000, 1000, 10000);

        motorFactory.setSensorToOutputRatio(CoralActuationConstants.gearRatio / 360);

        motorFactory.configureMotors();
        var motors = motorFactory.getMotors();
        coralActuationMotor = motors[0];

        encoder = new DutyCycleEncoder(encoderChannel);
        encoder.setInverted(false);

        seedMotor();

        motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(true).withSlot(0);
        voltageOut = new VoltageOut(0).withEnableFOC(true);
        stopMode = new NeutralOut();
    }

    @Override
    public void updateInputs(CoralActuationIOInputs inputs) {
        inputs.motorPos = coralActuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = coralActuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = coralActuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = coralActuationMotor.getMotorVoltage().getValueAsDouble();

        inputs.absoluteEncoderPos = encoder.get() * 360;
        inputs.relativeEncoderPos = encoder.get() * 360 - CoralActuationConstants.offset;

        motorFactory.checkForUpdates();
    }

    @Override
    public void moveToPos(double pos) {
        coralActuationMotor.setControl(motionMagicControl.withPosition(pos));
    }

    @Override
    public void runVoltage(double voltage) {
        coralActuationMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void stopMotor() {
        coralActuationMotor.setControl(stopMode);
    }

    @Override
    public void seedMotor() {
        coralActuationMotor.setPosition(encoder.get() * 360 - CoralActuationConstants.offset);
    }
}
