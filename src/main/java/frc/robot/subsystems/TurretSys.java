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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.drive.PoseEstimator;

public class TurretSys extends SubsystemBase {
  private final SparkFlex leftFlyWheelMtr;
  private final SparkFlex rightFlyWheelMtr;
  private final SparkFlex azimuthMtr;

  private final RelativeEncoder azimuthEnc;

  private double targetAzimuthAngle = 0;

  private final SparkClosedLoopController leftFlyWheelPID;
  private final SparkClosedLoopController rightFlyWheelPID;
  private final SparkClosedLoopController azimuthPID;

  private final PoseEstimator poseEstimator;
  private Pose2d currentPose;

  private double deltaX;
  private double deltaY;
  private double thetaHeading;
  private double deltaTheta;
  private double thetaTurret;
  private double thetaBotHeading;

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
    azimuthPID = azimuthMtr.getClosedLoopController();
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

    azimuthMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.azimuthP)
        .d(TurretConstants.azimuthD);

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
    currentPose = poseEstimator.getPose();

    deltaX = TurretConstants.blueTargetPose.getX() - currentPose.getX();
    deltaY = TurretConstants.blueTargetPose.getY() - currentPose.getY();
    deltaTheta = Math.atan(deltaY / deltaX);
    thetaBotHeading = currentPose.getRotation().getDegrees();
    thetaHeading = thetaBotHeading - deltaTheta;
    thetaTurret = -thetaHeading;

    // this is where a bunch of logic makes me wanna cry
    azimuthPID.setSetpoint(deltaTheta, ControlType.kVelocity);

  }
}