// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.PoseEstimator;

public class TurretSys extends SubsystemBase {
  private final SparkFlex leftFlyWheelMtr;
  private final SparkFlex rightFlyWheelMtr;
  private final SparkFlex azimuthMtr;

  private final RelativeEncoder azimuthEnc;
  private final RelativeEncoder leftFlyWheelEnc;
  private final RelativeEncoder rightFlyWheelEnc;

  private final SparkClosedLoopController leftFlyWheelPID;
  private final SparkClosedLoopController rightFlyWheelPID;
  private final ProfiledPIDController azimuthPID;
  private final SimpleMotorFeedforward azimuthFeedforward;
  private final PoseEstimator poseEstimator;

  private double targetAzimuthAngleRad = 0;
  private Double manualAzimuthAngle = null;
  private boolean isAiming = false;
  private Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  private Pose2d turretPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

  private final SysIdRoutine sysIdRoutine;

  public TurretSys(PoseEstimator poseEstimator) {

    this.poseEstimator = poseEstimator;

    leftFlyWheelMtr = new SparkFlex(CANDevices.leftFlyWheelMtrID, MotorType.kBrushless);
    SparkFlexConfig leftFlyWheelMtrSparkFlexConfig = new SparkFlexConfig();
    leftFlyWheelPID = leftFlyWheelMtr.getClosedLoopController();
    leftFlyWheelEnc = leftFlyWheelMtr.getEncoder();

    rightFlyWheelMtr = new SparkFlex(CANDevices.rightFlyWheelMtrID, MotorType.kBrushless);
    SparkFlexConfig rightFlyWheelMtrSparkFlexConfig = new SparkFlexConfig();
    rightFlyWheelPID = rightFlyWheelMtr.getClosedLoopController();
    rightFlyWheelEnc = rightFlyWheelMtr.getEncoder();

    azimuthMtr = new SparkFlex(CANDevices.azimuthMtrID, MotorType.kBrushless);
    SparkFlexConfig azimuthMtrSparkFlexConfig = new SparkFlexConfig();
    azimuthPID = new ProfiledPIDController(
            TurretConstants.azimuthP, 0.0, TurretConstants.azimuthD,
            new TrapezoidProfile.Constraints(TurretConstants.azimuthMaxVelocityRadPerSec, TurretConstants.azimuthMaxAccelerationRadPerSecSq));
    azimuthFeedforward = new SimpleMotorFeedforward(TurretConstants.azimuthkS, TurretConstants.azimuthkV, TurretConstants.azimuthkA);
    azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    azimuthEnc = azimuthMtr.getEncoder();

    azimuthMtrSparkFlexConfig.encoder.positionConversionFactor(TurretConstants.azimuthPositionConversionFactorRadPerRot);
    azimuthMtrSparkFlexConfig.encoder.velocityConversionFactor(TurretConstants.azimuthVelocityConversionFactorRadPerRotPerSec);

    leftFlyWheelMtrSparkFlexConfig.encoder.positionConversionFactor(TurretConstants.flyWheelPositionConversionFactorRotPerRot);
    leftFlyWheelMtrSparkFlexConfig.encoder.velocityConversionFactor(TurretConstants.flyWheelVelocityConversionFactorRotPerRotPerSec);
    rightFlyWheelMtrSparkFlexConfig.encoder.positionConversionFactor(TurretConstants.flyWheelPositionConversionFactorRotPerRot);
    rightFlyWheelMtrSparkFlexConfig.encoder.velocityConversionFactor(TurretConstants.flyWheelVelocityConversionFactorRotPerRotPerSec);

    azimuthMtrSparkFlexConfig.inverted(false);
    leftFlyWheelMtrSparkFlexConfig.inverted(true);
    rightFlyWheelMtrSparkFlexConfig.inverted(false);

    azimuthMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    leftFlyWheelMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    rightFlyWheelMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    azimuthMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxAzimuthCurrentAmps);
    leftFlyWheelMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxFlyWheelCurrentAmps);
    rightFlyWheelMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxFlyWheelCurrentAmps);

    azimuthMtrSparkFlexConfig.voltageCompensation(10);
    leftFlyWheelMtrSparkFlexConfig.voltageCompensation(10);
    rightFlyWheelMtrSparkFlexConfig.voltageCompensation(10);

    azimuthMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
    azimuthMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
    azimuthMtrSparkFlexConfig.softLimit.forwardSoftLimit(TurretConstants.maximumAizmuthAngleDeg);
    azimuthMtrSparkFlexConfig.softLimit.reverseSoftLimit(TurretConstants.minimumAizmuthAngleDeg);

    rightFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flyWheelP)
        .d(TurretConstants.flyWheelD);

    leftFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flyWheelP)
        .d(TurretConstants.flyWheelD);

    // initialzing sysID routines
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // default ramp rate
            Volts.of(7), // default step voltage
            Time.ofBaseUnits(10, Seconds) // default timeout)
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
                setAzimuthVoltage(voltage.in(Volts));
            },
            (log) -> {
                    Voltage voltage = Volts.of(getCharacterizationVoltage());
                    Angle angle = Angle.ofBaseUnits(azimuthEnc.getPosition(), Radian);
                    AngularVelocity angularVelocity = AngularVelocity.ofBaseUnits(azimuthEnc.getVelocity(), RadiansPerSecond);
                    log.motor("azimuth")
                        .voltage(voltage)
                        .angularPosition(angle)
                        .angularVelocity(angularVelocity);
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
    robotPose = poseEstimator.getPose();
    turretPose = robotPose.transformBy(TurretConstants.robotToTurret);

    targetAzimuthAngleRad =
      TurretConstants.targetPose.getTranslation()
      .minus(turretPose.getTranslation())
      .getAngle()
      .minus(new Rotation2d(robotPose.getRotation().getRadians())).getRadians();

    if (DriverStation.isDisabled()) {
      azimuthPID.setGoal(azimuthEnc.getPosition());  
    } else if (isAiming && targetAzimuthAngleRad <= Units.degreesToRadians(TurretConstants.maximumAizmuthAngleDeg) && targetAzimuthAngleRad >= Units.degreesToRadians(TurretConstants.minimumAizmuthAngleDeg)) {
      azimuthPID.setGoal(targetAzimuthAngleRad);
    }  /* for troubleshooting only, remove for competition */ else if (manualAzimuthAngle != null) {
       azimuthPID.setGoal(manualAzimuthAngle);
    } else {
      azimuthPID.setGoal(Units.degreesToRadians(TurretConstants.azimuthDefaultSetpointDeg));
    }
    
    azimuthMtr.set(azimuthPID.calculate(azimuthEnc.getPosition()) + azimuthFeedforward.calculate(azimuthPID.getSetpoint().velocity));
  }

  public void setManualAzimuthAngle(Double manualAzimuthAngle) {
    this.manualAzimuthAngle = manualAzimuthAngle;
  }

  public void setIsAiming(boolean isAiming) {
    this.isAiming = isAiming;
  }

  public boolean isOnTarget(){
    return (Math.abs(getCurrentAzimuthAngleRad() - getTargetAzimuthAngleRad()) <= Units.degreesToRadians(TurretConstants.azimuthErrorTolerance));
  }

  public void setFlywheelRPM(double targetRPM) {
    leftFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
    rightFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public double getFlywheelRPM() {
    return (leftFlyWheelEnc.getVelocity() + rightFlyWheelEnc.getVelocity()) / 2.0;
  }

  public double getTargetAzimuthAngleRad() {
    return targetAzimuthAngleRad;
  }

  public double getCurrentAzimuthAngleRad(){
    return azimuthEnc.getPosition();
  }

  public Pose2d getTurretPose() {
    return turretPose;
  }

  // for sysID charachterization only
  public void setAzimuthVoltage(double voltage) {
    azimuthMtr.setVoltage(voltage);
  }

  public double getCharacterizationVoltage() {
    return azimuthMtr.getAppliedOutput() * RobotController.getBatteryVoltage();
  }
}