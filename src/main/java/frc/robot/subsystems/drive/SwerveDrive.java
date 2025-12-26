package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {

    private boolean isLocked = false;
    public boolean isLocked() {
        return isLocked;
    }

    private Rotation2d skewCompensationThetaShift = new Rotation2d();

    private final SwerveModule[] modules;

    private final Pigeon2 pigeon;

    private final StructArrayPublisher<SwerveModuleState> publisher;

    private final SysIdRoutine sysIdRoutine;

    public SwerveDrive() {
        modules = new SwerveModule[] {
            new REVSwerveModule(
                CANDevices.flModuleCANCoderID,
                CANDevices.flModuleDriveMtrID,
                CANDevices.flModuleSteerMtrID,
                SwerveDriveConstants.flModuleOffset),
            new REVSwerveModule(
                CANDevices.frModuleCANCoderID,
                CANDevices.frModuleDriveMtrID,
                CANDevices.frModuleSteerMtrID,
                SwerveDriveConstants.frModuleOffset),
            new REVSwerveModule(
                CANDevices.blModuleCANCoderID,
                CANDevices.blModuleDriveMtrID,
                CANDevices.blModuleSteerMtrID,
                SwerveDriveConstants.blModuleOffset),
            new REVSwerveModule(
                CANDevices.brModuleCANCoderID,
                CANDevices.brModuleDriveMtrID,
                CANDevices.brModuleSteerMtrID,
                SwerveDriveConstants.brModuleOffset)
        };

        pigeon = new Pigeon2(CANDevices.pigeonID);

        // TODO Modify this to reflect how the Pigeon is mounted
        // Pigeon is mounted upside down
        pigeon.getConfigurator().apply(new MountPoseConfigs().withMountPoseRoll(180.0));

        // start publishing an array of module states to NetworkTables with the "/SwerveStates" key
        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

        // initialzing sysID routines
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // default ramp rate
                null, // default step voltage
                null // default timeout)
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    for (int i = 0; i < modules.length; i++) {
                        double volts = voltage.in(Volts);
                        modules[i].applyCharacterizationVoltage(volts);
                    }
                },
                (log) -> {
                    for (int i = 0; i < modules.length; i++) {
                        Distance distance = Meters.of(modules[i].getDriveDistanceMeters());
                        LinearVelocity velocity = MetersPerSecond.of(modules[i].getDriveSpeedMetersPerSec());
                        log.motor("drive-" + i)
                            .linearPosition(distance)
                            .linearVelocity(velocity);
                    }
                },
                this
            )
        );

    }

    // sysID command factories
    public Command sysIdQuasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdQuasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command sysIdDynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command sysIdDynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    @Override
    public void periodic() {
        // periodically updates the set of swerve module states published to NetworkTables
        publisher.set(getModuleStates());
    }

    public Rotation2d getHeading() {
        // reading is negated so that CCW is positive
        return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    public double getAngularVelocityDegPerSec() {
        return pigeon.getAngularVelocityZDevice().getValueAsDouble();
    }

    public void lock() {
        isLocked = true;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            // undo skew compensation for correct pose estimation
            SwerveModulePosition positionSkewed = modules[i].getModulePosition();
            positions[i] = new SwerveModulePosition(
                positionSkewed.distanceMeters, 
                positionSkewed.angle.plus(skewCompensationThetaShift));
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            // undo skew compensation for correct pose estimation
            SwerveModuleState stateSkewed = modules[i].getModuleState();
            states[i] = new SwerveModuleState(
                stateSkewed.speedMetersPerSecond,
                stateSkewed.angle.plus(skewCompensationThetaShift));
        }
        return states;
    }

    public Rotation2d[] getCanCoderAngles() {
        Rotation2d[] canCoderAngles = new Rotation2d[4];
        for(int i = 0; i < 4; i++) {
            canCoderAngles[i] = modules[i].getCanCoderAngle();
        }
        return canCoderAngles;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveDriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeSpeeds(Rotation2d robotHeading) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), robotHeading);
    }

    public void driveFieldRelative(double xMetersPerSec, double yMetersPerSec, double omegaRadPerSec, Rotation2d heading) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMetersPerSec, yMetersPerSec, omegaRadPerSec, heading);
        driveRobotRelative(chassisSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        // only lock wheels if all inputs are zero
        if(chassisSpeeds.vxMetersPerSecond != 0.0 || chassisSpeeds.vyMetersPerSecond != 0.0 || chassisSpeeds.omegaRadiansPerSecond != 0.0) {
            isLocked = false;
        }
        
        SwerveModuleState[] desiredModuleStates;

        if(isLocked) {
            desiredModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            };
        }
        else {
            chassisSpeeds = compensateForSkew(chassisSpeeds);

            desiredModuleStates = SwerveDriveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredModuleStates,
                chassisSpeeds,
                SwerveModuleConstants.driveFreeSpeedMetersPerSec,
                SwerveDriveConstants.maxAttainableSpeedMetersPerSec,
                SwerveDriveConstants.maxAttainableRotationRadPerSec);
        }

        for(int i = 0; i < 4; i++) {
            modules[i].setState(desiredModuleStates[i]);
        }
    }

    /**
     * Counters the skew in lateral motion caused by rotation of the chassis by shifting the direction of
     * the lateral motion by a factor of the angular velocity opposite the skew. The shift is stored as a
     * local variable and should be used to unshift the module angles when applying them to odometry.
     * 
     * @param chassisSpeeds The ChassisSpeeds to modify.
     * @return The skew-compensated ChassisSpeeds.
     */
    private ChassisSpeeds compensateForSkew(ChassisSpeeds chassisSpeeds) {
        double omegaRadPerSec = chassisSpeeds.omegaRadiansPerSecond;
        double thetaRad = Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);
        double velMetersPerSec = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        
        // TODO: made sure this functions as intended
        if (DriverStation.isAutonomous()) {
            skewCompensationThetaShift = Rotation2d.fromRadians(omegaRadPerSec * 0.0);
        } else {
            skewCompensationThetaShift = Rotation2d.fromRadians(omegaRadPerSec * SwerveDriveConstants.skewCompensationRatioOmegaPerTheta);
        }

        thetaRad -= skewCompensationThetaShift.getRadians();
            return new ChassisSpeeds(
                velMetersPerSec * Math.cos(thetaRad),
                velMetersPerSec * Math.sin(thetaRad),
                omegaRadPerSec);
    }

    // for sysID use only
    public void applyCharacterizationVoltage(double volts) {
        for(SwerveModule module : modules) {
            module.applyCharacterizationVoltage(volts);
        }
    }
}
