// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;

// FIXME: uncomment once pathplannerlib is released
// import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class RobotConstants {
        // TODO: Set to the current the weight of the robot, including the battery and
        // bumpers.
        public static final double massKg = 52.41;

        // TODO: Set the frame dimensions of the robot.
        public static final double robotWidthMeters = Units.inchesToMeters(25.0);
        public static final double robotLengthMeters = Units.inchesToMeters(25.0);

        // Moment of inertia of a uniform-mass slab with the axis of rotation centered
        // and perpendicular to the slab
        // This should be a reasonable approximation of the robot's MOI
        public static final double momentOfInertiaKgMetersSq = massKg
                * (Math.pow(robotWidthMeters, 2) + Math.pow(robotLengthMeters, 2)) / 12;
    }

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        // TODO: change deadband based on controller drift
        public static final double joystickDeadband = 0.08;
        public static final double tiggerPressedThreshold = 0.35;
    }

    public static class CANDevices {
        public static final int pigeonID = 14;

        public static final int frModuleCANCoderID = 2;
        public static final int frModuleDriveMtrID = 6;
        public static final int frModuleSteerMtrID = 10;

        public static final int brModuleCANCoderID = 3;
        public static final int brModuleDriveMtrID = 7;
        public static final int brModuleSteerMtrID = 11;

        public static final int flModuleCANCoderID = 4;
        public static final int flModuleDriveMtrID = 8;
        public static final int flModuleSteerMtrID = 12;

        public static final int blModuleCANCoderID = 5;
        public static final int blModuleDriveMtrID = 9;
        public static final int blModuleSteerMtrID = 13;

        public static final int RollerMtrID = 15;
        public static final int LeftActuatorMtrID = 16;
        public static final int RightActuatorMtrID = 17;

        public static final int spindexerMtrID = 20;
        public static final int towerMtrID = 21;

        public static final int azimuthMtrID = 22;
        public static final int leftFlyWheelMtrID = 23;
        public static final int rightFlyWheelMtrID = 24;

        public static final int HookMtrID = 25;
        public static final int LeftElevatorID = 26;
        public static final int RightElevatorID = 27;

    }

    public static class SwerveModuleConstants {
        // TODO: Tune the below PID and FF values using the SysID routines.
        public static final double driveKp = 0.12;
        public static final double driveKd = 0.0;

        public static final double steerKp = 0.45;
        public static final double steerKd = 0.25;

        public static final double driveKsVolts = 0.667;
        public static final double driveKvVoltSecsPerMeter = 2.44;
        public static final double driveKaVoldSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKsVolts,
                driveKvVoltSecsPerMeter, driveKaVoldSecsPerMeterSq);

        // TODO: Change this value depending on your breakers and the current usage of
        // the rest of your robot.
        public static final int driveMtrCurrentLimitAmps = 40;
        public static final int steerMtrCurrentLimitAmps = 40;

        // TODO: Change this number based on actual wheel diamter.
        public static final double wheelRadiusMeters = Units.inchesToMeters(1.9);

        // TODO: Set this value to the coefficient of friction of your wheels.
        // FIXME: Do we need this value?
        public static final double wheelCoefficientOfFriction = 1.5;

        public static final double driveGearReduction = (16.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0);

        public static final double driveMetersPerEncRev = driveGearReduction * 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerSecPerEncRPM = driveMetersPerEncRev / 60.0;

        public static final double steerGearReduction = 1.0 / 26.0;

        public static final double steerRadiansPerEncRev = steerGearReduction * 2.0 * Math.PI;

        public static final double steerRadiansPerSecPerEncRPM = steerRadiansPerEncRev / 60.0;

        public static final double driveFreeSpeedMetersPerSec = Units.feetToMeters(22.5);

        public static final double driveFreeSpeedRadPerSec = driveFreeSpeedMetersPerSec / wheelRadiusMeters;

        public static final double driveNominalOperatingVoltage = 12.4;
        public static final double driveStallTorqueNewtonMeters = 3.6 / driveGearReduction; // Motor's stall torque
                                                                                            // times gear ratio
        public static final double driveStallCurrentAmps = 211.0;
        public static final double driveFreeCurrentAmps = 3.6;

        // FIXME: uncomment once pathplannerlib is released
        public static final ModuleConfig moduleConfig = new ModuleConfig(
            wheelRadiusMeters, SwerveDriveConstants.maxAttainableSpeedMetersPerSec, wheelCoefficientOfFriction,
            new DCMotor(driveNominalOperatingVoltage, driveStallTorqueNewtonMeters, driveStallCurrentAmps,
                    driveFreeCurrentAmps, driveFreeSpeedRadPerSec, 1),
            driveMtrCurrentLimitAmps, 1);
    }

    public static class SwerveDriveConstants {
        // TODO: set these offsets based on module's zero position
        public static final Rotation2d flModuleOffset = Rotation2d.fromDegrees(-109.5); // 162.5
        public static final Rotation2d frModuleOffset = Rotation2d.fromDegrees(57.4); // 122.0
        public static final Rotation2d blModuleOffset = Rotation2d.fromDegrees(106.1); // -151.6
        public static final Rotation2d brModuleOffset = Rotation2d.fromDegrees(-30.4); // -102.8

        // Set these dimensions for the distance between the center of each wheel.
        // NOTE: these values are different from the robot's overall dimenstions.
        public static final double chassisTrackLengthMeters = Units.inchesToMeters(21.75); // 27 inch frame
        public static final double chassisTrackWidthMeters = Units.inchesToMeters(21.75); // 27 inch frame

        public static final double chassisRadiusMeters = Math.hypot(chassisTrackLengthMeters, chassisTrackWidthMeters);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(chassisTrackWidthMeters / 2.0, chassisTrackLengthMeters / 2.0), // front left
                new Translation2d(chassisTrackWidthMeters / 2.0, -chassisTrackLengthMeters / 2.0), // front right
                new Translation2d(-chassisTrackWidthMeters / 2.0, chassisTrackLengthMeters / 2.0), // back left
                new Translation2d(-chassisTrackWidthMeters / 2.0, -chassisTrackLengthMeters / 2.0) // back right
        );

        // TODO: Tune these values based on actual robot performaance.
        public static final double maxAttainableSpeedMetersPerSec = Units.feetToMeters(20.1 * 0.9);
        public static final double maxAttainableRotationRadPerSec = 13.4;

        public static final double skewCompensationRatioOmegaPerTheta = 0.1;

        // TODO: Tune the below PID values using the SysID routines.
        public static final double autoTranslationKp = 6.0;
        public static final double autoTranslationKd = 0.0;

        public static final double autoRotationKp = 8.0;
        public static final double autoRotationKd = 0.0;
    }

    public class VisionConstants {
        public static final String limelightOneName = "limelight-front";

        public static final String limelightTwoName = "limelight-back";

        public static final double odomTranslationStdDevMeters = 0.05;
        public static final double odomRotationStdDevRad = Units.degreesToRadians(0.25);

        public static final double visionTranslationStdDevMeters = 0.35;
        public static final double visionRotationStdDevRad = Units.degreesToRadians(30.0);

        public static final double angularVelocityDegPerSecThreshold = 360.0;
    }

    public class IntakeConstants {
        public static final int maxRollerCurrentAmps = 45;
        public static final int maxActuatorCurrentAmps = 20;

        public static final double ActuatorPulleyToothCount = 18.0;
        public static final double actuatorPositionConversionFactor = 0.95; // Units.metersToInches(ActuatorPulleyToothCount
                                                                            // * 5.0) / 5.0; // convertInches(tooth *
                                                                            // pitch) / ratio
        public static final double actuatorVelocityConversionFactor = 0.95 / 60;// Units.metersToInches(ActuatorPulleyToothCount
                                                                                // * 5.0) / 5.0 /60.0;

        public static final double rollerPositionConversionFactor = 1.0 / 3.0;
        public static final double rollerVelocityConversionFactor = 1.0 / 3.0;

        public static final double actuatorOutPositionInches = 12;
        public static final double actuatorInPositionInches = 0.0;
        public static final double actuatorSafePositionInches = 5.0;

        public static final double actuatorMinPositionInches = 0.0;
        public static final double actuatorMaxPositionInches = 12.5;

        public static final double RollerP = 0.0010;
        public static final double RollerD = 0.0002;

        public static final double actuatorP = 1;
        public static final double actuatorD = 0.05;
        public static final double intakingRollerRPM = 3999.0;
        public static final double agitatingRollerRPM = 500.0;

        // public static final double manualActuatorAdjustmentSpeed = 0.4;
    }

    public class TurretConstants {
        public static final int maxAzimuthCurrentAmps = 20;
        public static final int maxFlyWheelCurrentAmps = 40;

        public static final double azimuthP = 0.0;
        public static final double azimuthD = 0.0;
        public static final double azimuthkS = 0.0;
        public static final double azimuthkV = 0.0;
        public static final double azimuthkA = 0.0;

        public static final double flywheelkP = 0.00012;
        public static final double flywheelkD = 0.00;
        public static final double flywheelkS = 0.05; // increment voltage setpoint until the flywheel moves to find this value
        public static final double flywheelkV = 0.067; // calculated from ReCalc
        public static final double flywheelkA = 0.06; // calculated from ReCalc

        public static final double azimuthMaxVelocityRadPerSec = 180.0;
        public static final double azimuthMaxAccelerationRadPerSecSq = 360.0;
        public static final double maximumAizmuthAngleDeg = 90.0;
        public static final double minimumAizmuthAngleDeg = -90.0;
        public static final double azimuthErrorTolerance = 5;
        public static final double azimuthDefaultSetpointDeg = 0.0;

        public static final double azimuthPositionConversionFactorRadPerRot = 20.0 / 173.0 * 2.0 * Math.PI;
        public static final double azimuthVelocityConversionFactorRadPerRotPerSec = 20.0 / 173.0 * 2.0 * Math.PI / 60.0;

        public static final double flyWheelPositionConversionFactorRot = 15.0 / 18.0;
        public static final double flyWheelVelocityConversionFactorRPM = 15.0 / 18.0;

        public static final double zerothDegreeFitConstant = 0.0;
        public static final double firstDegreeFitConstant = 0.0;
        public static final double secondDegreeFitConstant = 0.0;

        public static final double flywheelOffsetRPMIncrement = 200.0;

        public static final Pose2d targetPoseBlue = new Pose2d(4.62, 4.04, null);
        public static final Pose2d targetPoseRed = new Pose2d(11.915, 4.035, null);

        public static final double turretOffsetX = 0.0;
        public static final double turretOffsetY = 0.0;
        public static final Transform2d robotToTurret = new Transform2d(new Translation2d(turretOffsetX, turretOffsetY),
                new Rotation2d());
    }

    public class ClimberConstants {
        public static final int maxHookCurrentAmps = 20;
        public static final int maxElevatorCurrentAmps = 40;
        public static final double ElevatorP = 0.0;
        public static final double ElevatorD = 0.0;

        public static final double HookP = 0.0;
        public static final double HookD = 0.0;

        public static final double elevatorPositionConversionFactor = 1.0;
        public static final double elevatorVelocityConversionFactor = 1.0;

        public static final double hookPositionConversionFactor = 1.0;
        public static final double hookVelocityConversionFactor = 1.0;

        public static final double ElevatorMinInches = 0.0;
        public static final double ElevatorMaxInches = 0.0;

        public static final double hookMinDeg = 0.0;
        public static final double hookMaxDeg = 0.0;

        public static final double ElevatorMinSafeInches = 0.0;
        public static final double ElevatorMaxSafeInches = 0.0;
        public static final double hookMinPositionInches = 0.0;
        public static final double hookMaxPositionInches = 0.0;

        public static final double elevatorClimbPositionInches = 0.0;
        public static final double elevatorSafePositionInches = 0.0;
        public static final double elevatorStartPositionInches = 0.0;

        public static final double hookOutPositionRotations = 0.25;

        public static final double ClimberL1Position = 0;

    }

    public class IndexerConstants {
        public static final int maxTowerCurrentAmps = 50;
        public static final int maxSpindexerCurrentAmps = 50;

        public static final double towerP = 0.00015;
        public static final double towerD = 0.00;

        public static final double spindexerP = 0.005;
        public static final double spindexerD = 0.0005;

        public static final double towerPositionConversionFactor = 1.0;
        public static final double towerVelocityConversionFactor = 1.0 / 60.0;

        public static final double spindexerPositionConversionFactor = 1.0 / 15.0;
        public static final double spindexerVelocityConversionFactor = 1.0 / 15.0;

        public static final double spindexerShootingRPM = 5000.0;
        public static final double spindexerAgitatingRPM = 5000.0;

        public static final double towerShootingRPM = 0.0;
        public static final double towerIntakingRPM = 0.0;
    }
}