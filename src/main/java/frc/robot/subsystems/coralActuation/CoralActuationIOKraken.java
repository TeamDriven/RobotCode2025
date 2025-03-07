package frc.robot.subsystems.coralActuation;

import static frc.robot.subsystems.coralActuation.CoralActuationConstants.offset;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotState;
import frc.robot.util.TalonFXUtil.MotorFactory;

public class CoralActuationIOKraken implements CoralActuationIO {
    private MotorFactory motorFactory;
    private TalonFX coralActuationMotor;

    private PositionVoltage positionControl;
    private VoltageOut voltageOut;
    private NeutralOut stopMode;
    private DutyCycleEncoder encoder;

    public CoralActuationIOKraken(int motorID, int encoderChannel) {
        motorFactory = new MotorFactory("CoralActuation", motorID);

        motorFactory.setInverted(false);
        motorFactory.setBrakeMode(true);
        motorFactory.setVoltageLimits(8);
        motorFactory.setCurrentLimits(40);
        
        motorFactory.setSlot0(45, 0.5, 1.5);
        motorFactory.setSlot0(0, 0.22, GravityTypeValue.Arm_Cosine);

        motorFactory.setSlot1(50, 0, 7.5);
        motorFactory.setSlot1(3, 0.26, GravityTypeValue.Arm_Cosine);

        motorFactory.setSensorToOutputRatio(CoralActuationConstants.gearRatio);

        motorFactory.configureMotors();
        var motors = motorFactory.getMotors();
        coralActuationMotor = motors[0];

        encoder = new DutyCycleEncoder(encoderChannel);
        encoder.setInverted(false);

        seedMotor(Rotation2d.fromRotations(encoder.get()).minus(new Rotation2d(offset)));

        positionControl = new PositionVoltage(0).withEnableFOC(true);
        voltageOut = new VoltageOut(0).withEnableFOC(true);
        stopMode = new NeutralOut();
    }

    @Override
    public void updateInputs(CoralActuationIOInputs inputs) {
        inputs.motorPos = coralActuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = coralActuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = coralActuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = coralActuationMotor.getMotorVoltage().getValueAsDouble();

        inputs.absoluteEncoderPos = Rotation2d.fromRotations(encoder.get());
        inputs.relativeEncoderPos = inputs.absoluteEncoderPos.minus(new Rotation2d(offset));

        // motorFactory.checkForUpdates();
    }

    @Override
    public void moveToPos(double pos) {
        pos = Units.degreesToRotations(pos);
        coralActuationMotor.setControl(positionControl.withPosition(pos).withSlot(RobotState.getInstance().hasCoral() ? 1 : 0));
    }

    @Override
    public void runVoltage(double voltage) {
        coralActuationMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        motorFactory.setBrakeMode(brakeMode);
        motorFactory.configureMotors();
    }

    @Override
    public void stopMotor() {
        coralActuationMotor.setControl(stopMode);
    }

    @Override
    public void seedMotor(Rotation2d currentRot) {
        coralActuationMotor.setPosition(currentRot.getRotations());
    }
}
