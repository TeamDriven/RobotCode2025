package frc.robot.subsystems.actuation;

import static frc.robot.subsystems.actuation.ActuationConstants.offset;

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

public class ActuationIOKraken implements ActuationIO {
    private MotorFactory motorFactory;
    private TalonFX actuationMotor;

    private PositionVoltage positionControl;
    private VoltageOut voltageOut;
    private NeutralOut stopMode;
    private DutyCycleEncoder encoder;

    public ActuationIOKraken(int motorID, int encoderChannel) {
        motorFactory = new MotorFactory("Actuation", motorID);

        motorFactory.setInverted(true);
        motorFactory.setBrakeMode(true);
        motorFactory.setVoltageLimits(8);
        motorFactory.setCurrentLimits(40);
        
        motorFactory.setSlot0(50, 5, 8);
        motorFactory.setSlot0(7, 0, GravityTypeValue.Arm_Cosine);

        motorFactory.setSlot1(65, 8, 8);
        motorFactory.setSlot1(5, 0.3, GravityTypeValue.Arm_Cosine);

        motorFactory.setSensorToOutputRatio(ActuationConstants.gearRatio);

        motorFactory.configureMotors();
        var motors = motorFactory.getMotors();
        actuationMotor = motors[0];

        encoder = new DutyCycleEncoder(encoderChannel);
        encoder.setInverted(false);

        seedMotor(Rotation2d.fromRotations(encoder.get()).minus(new Rotation2d(offset)).unaryMinus());

        positionControl = new PositionVoltage(0).withEnableFOC(true);
        voltageOut = new VoltageOut(0).withEnableFOC(true);
        stopMode = new NeutralOut();
    }

    @Override
    public void updateInputs(ActuationIOInputs inputs) {
        inputs.motorPos = actuationMotor.getPosition().getValueAsDouble();
        inputs.motorCurrent = actuationMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorVel = actuationMotor.getVelocity().getValueAsDouble();
        inputs.motorVoltage = actuationMotor.getMotorVoltage().getValueAsDouble();

        inputs.encoderConnected = encoder.isConnected();

        inputs.absoluteEncoderPos = Rotation2d.fromRotations(encoder.get());
        inputs.relativeEncoderPos = inputs.absoluteEncoderPos.minus(new Rotation2d(offset)).unaryMinus();

        // motorFactory.checkForUpdates();
    }

    @Override
    public void moveToPos(double pos) {
        pos = Units.degreesToRotations(pos);
        // actuationMotor.setControl(positionControl.withPosition(pos).withSlot(1));
        actuationMotor.setControl(positionControl.withPosition(pos).withSlot(RobotState.getInstance().hasAlgae() ? 1 : 0));
    }

    @Override
    public void runVoltage(double voltage) {
        actuationMotor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setBrakeMode(boolean brakeMode) {
        motorFactory.setBrakeMode(brakeMode);
        motorFactory.configureMotors();
    }

    @Override
    public void stopMotor() {
        actuationMotor.setControl(stopMode);
    }

    @Override
    public void seedMotor(Rotation2d currentRot) {
        actuationMotor.setPosition(currentRot.getRotations());
    }
}
