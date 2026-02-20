// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PrivateKey;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.PoseEstimator;

public class TurretSys extends SubsystemBase {
  private final SparkFlex leftFlyWheelMtr;
  private final SparkFlex rightFlyWheelMtr;
  private final SparkFlex azimuthMtr;

  private final RelativeEncoder azimuthEnc;

  private final SparkClosedLoopController leftFlyWheelPID;
  private final SparkClosedLoopController rightFlyWheelPID;
  private final ProfiledPIDController azimuthPID;
  private final SimpleMotorFeedforward azimuthFeedforward;
  private final PoseEstimator poseEstimator;

  private double targetAzimuthAngle = 0;
  private Double manualAzimuthAngle = null;
  private boolean isAiming = false;
  private Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
  private Pose2d turretPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));


   @SuppressWarnings("removal")
  public TurretSys(PoseEstimator poseEstimator) {

    this.poseEstimator = poseEstimator;

    leftFlyWheelMtr = new SparkFlex(CANDevices.leftFlyWheelMtrID, MotorType.kBrushless);
    SparkFlexConfig leftFlyWheelMtrSparkFlexConfig = new SparkFlexConfig();
    leftFlyWheelPID = leftFlyWheelMtr.getClosedLoopController();

    rightFlyWheelMtr = new SparkFlex(CANDevices.rightFlyWheelMtrID, MotorType.kBrushless);
    SparkFlexConfig rightFlyWheelMtrSparkFlexConfig = new SparkFlexConfig();
    rightFlyWheelPID = rightFlyWheelMtr.getClosedLoopController();

    azimuthMtr = new SparkFlex(CANDevices.azimuthMtrID, MotorType.kBrushless);
    SparkFlexConfig azimuthMtrSparkFlexConfig = new SparkFlexConfig();
    azimuthPID = new ProfiledPIDController(
            TurretConstants.azimuthP, 0.0, TurretConstants.azimuthD,
            new TrapezoidProfile.Constraints(TurretConstants.azimuthMaxVelocityDegPerSec, TurretConstants.azimuthMaxAccelerationDegPerSecSq));
    azimuthFeedforward = new SimpleMotorFeedforward(TurretConstants.azimuthkS, TurretConstants.azimuthkV, TurretConstants.azimuthkA);
    azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    azimuthEnc = azimuthMtr.getEncoder();

    azimuthMtrSparkFlexConfig.encoder.positionConversionFactor(TurretConstants.azimuthPositionConversionFactor);
    azimuthMtrSparkFlexConfig.encoder.velocityConversionFactor(TurretConstants.azimuthVelocityConversionFactor);

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
    azimuthMtrSparkFlexConfig.softLimit.forwardSoftLimit(TurretConstants.turretMaxPositionInches);
    azimuthMtrSparkFlexConfig.softLimit.reverseSoftLimit(TurretConstants.turretMinPositionInches);

    rightFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flyWheelP)
        .d(TurretConstants.flyWheelD);

    leftFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flyWheelP)
        .d(TurretConstants.flyWheelD);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   * 
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    robotPose = poseEstimator.getPose();
    turretPose = robotPose.transformBy(TurretConstants.robotToTurret);

    targetAzimuthAngle = -1.0 * 
      TurretConstants.targetPose.getTranslation()
      .minus(turretPose.getTranslation())
      .getAngle()
      .minus(robotPose.getRotation()).getRadians();

    if(isAiming && targetAzimuthAngle <= Units.degreesToRadians(TurretConstants.maximumAizmuthAngleDeg) && targetAzimuthAngle >= Units.degreesToRadians(TurretConstants.minimumAizmuthAngleDeg)) {
        azimuthPID.setGoal(targetAzimuthAngle);
    }
    
    // for troubleshooting only, remove for competition
    if(manualAzimuthAngle != null) {
      azimuthPID.setGoal(manualAzimuthAngle);
    }

    azimuthMtr.set(azimuthPID.calculate(azimuthEnc.getPosition()) + azimuthFeedforward.calculate(azimuthPID.getSetpoint().velocity));
  }

  // for sysID profiling only
  public void setAzimuthVoltage(double voltage) {
    azimuthMtr.setVoltage(voltage);
  }

  public void setManualAzimuthAngle(Double manualAzimuthAngle) {
    this.manualAzimuthAngle = manualAzimuthAngle;
  }

  public void setIsAiming(boolean isAiming) {
    this.isAiming = isAiming;
  }

  public void setFlyWheelRPM(double targetRPM) {
    leftFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
    rightFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public double getTargetAzimuthAngle() {
    return targetAzimuthAngle;
  }

  public Pose2d getTurretPose() {
    return turretPose;
  }
}