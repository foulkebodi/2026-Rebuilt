package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveModuleConstants;

public class REVSwerveModule extends SwerveModule {

    private final SparkFlex driveMtr;
    private final SparkFlex steerMtr;

    private final RelativeEncoder driveEnc;
    private final RelativeEncoder steerEnc;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController steerPID;

    private final CANcoder canCoder;

    private final Rotation2d moduleOffset;

    public REVSwerveModule(int canCoderID, int driveMtrID, int steerMtrID, Rotation2d moduleOffset) {
        this.moduleOffset = moduleOffset;

        driveMtr = new SparkFlex(driveMtrID, MotorType.kBrushless);
        steerMtr = new SparkFlex(steerMtrID, MotorType.kBrushless);

        SparkFlexConfig driveConfig = new SparkFlexConfig();
        SparkFlexConfig steerConfig = new SparkFlexConfig();

        driveConfig.idleMode(IdleMode.kBrake);
        steerConfig.idleMode(IdleMode.kBrake);

        driveConfig.smartCurrentLimit(SwerveModuleConstants.driveMtrCurrentLimitAmps);
        steerConfig.smartCurrentLimit(SwerveModuleConstants.steerMtrCurrentLimitAmps);

        steerConfig.inverted(false);
        driveConfig.inverted(false);

        driveConfig.encoder.positionConversionFactor(SwerveModuleConstants.driveMetersPerEncRev);
        steerConfig.encoder.positionConversionFactor(SwerveModuleConstants.steerRadiansPerEncRev);

        driveConfig.encoder.velocityConversionFactor(SwerveModuleConstants.driveMetersPerSecPerEncRPM);
        steerConfig.encoder.velocityConversionFactor(SwerveModuleConstants.steerRadiansPerSecPerEncRPM);

        driveConfig.closedLoop.p(SwerveModuleConstants.driveKp);
        steerConfig.closedLoop.p(SwerveModuleConstants.steerKp);

        driveConfig.closedLoop.d(SwerveModuleConstants.driveKd);
        steerConfig.closedLoop.d(SwerveModuleConstants.steerKd);

        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.closedLoop.positionWrappingInputRange(0.0, 2.0 * Math.PI);

        driveMtr.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steerMtr.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEnc = driveMtr.getEncoder();
        steerEnc = steerMtr.getEncoder();

        drivePID = driveMtr.getClosedLoopController();
        steerPID = steerMtr.getClosedLoopController();

        canCoder = new CANcoder(canCoderID);

        canCoder.getConfigurator().apply(new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));

        resetSteerEncAngle();
    }

    public double getDriveDistanceMeters() {
        return driveEnc.getPosition();
    }

    public double getDriveSpeedMetersPerSec() {
        return driveEnc.getVelocity();
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(steerEnc.getPosition());
    }

    public double getSteerSpeedRadPerSec() {
        return steerEnc.getVelocity();
    }

    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getSteerAngle());
    }  
    
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveSpeedMetersPerSec(), getSteerAngle());
    }

    public void resetSteerEncAngle() {
        steerEnc.setPosition(getCanCoderAngle().getRadians() - moduleOffset.getRadians());
    }

    public void setState(SwerveModuleState desiredState) {
        // optimize speed and angle to minimize change in heading
        // (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
        desiredState.optimize(getSteerAngle());

        // scale velocity based on turn error to help prevent skew
        double angleErrorRad = desiredState.angle.getRadians() - getSteerAngle().getRadians();
        desiredState.speedMetersPerSecond *= Math.cos(angleErrorRad);

        drivePID.setSetpoint(desiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, SwerveModuleConstants.driveFF.calculate(desiredState.speedMetersPerSecond));

        steerPID.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // for use in sysID characterization routines
    public void applyCharacterizationVoltage(double volts) {
        driveMtr.setVoltage(volts);
        steerPID.setSetpoint(0.0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getCharacterizationVoltage(double volts) {
        return driveMtr.getAppliedOutput() * RobotController.getBatteryVoltage();
    }
}