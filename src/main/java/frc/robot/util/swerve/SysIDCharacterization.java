package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drive.Drive;

public class SysIDCharacterization {
    private final SysIdRoutine m_driveSysIdRoutine;

    private final SysIdRoutine m_steerSysIdRoutine;

    public SysIDCharacterization(Drive drive) {
        this.m_driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1.5).per(Second), null, null, (state) -> Logger.recordOutput("SysID/state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> drive.runCharacterization(volts.in(Volts)),
                null,
                drive));

        this.m_steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("SysID/state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> drive.runCharacterization(volts.in(Volts), Drive.getCircleOrientations()),
                null,
                drive));
    }

    public Command runDriveQuasiTest(Direction direction)
    {
        return m_driveSysIdRoutine.quasistatic(direction);
    }

    public Command runDriveDynamTest(Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction);
    }

    public Command runSteerQuasiTest(Direction direction)
    {
        return m_steerSysIdRoutine.quasistatic(direction);
    }

    public Command runSteerDynamTest(Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction);
    }
}
