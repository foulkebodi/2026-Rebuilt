// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;

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
  private final SparkClosedLoopController azimuthPID;
  // private final SimpleMotorFeedforward azimuthFeedforward;
  private final PoseEstimator poseEstimator;

  private Double manualAzimuthAngle = 0.0;
  private Double manualFlywheelRPM = 0.0;
  private boolean isAiming = false;
  private boolean isFiring = false;
  private double flywheelOffsetRPM = 0.0;
  private final SysIdRoutine sysIdRoutine;
  private Field2d field = new Field2d();

  @SuppressWarnings("removal")
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
    // azimuthPID = new ProfiledPIDController(
    //     TurretConstants.azimuthP, 0.0, TurretConstants.azimuthD,
    //     new TrapezoidProfile.Constraints(TurretConstants.azimuthMaxVelocityRadPerSec,
    //         TurretConstants.azimuthMaxAccelerationRadPerSecSq));
    // azimuthFeedforward = new SimpleMotorFeedforward(TurretConstants.azimuthkS, TurretConstants.azimuthkV,
    //     TurretConstants.azimuthkA);
    azimuthPID = azimuthMtr.getClosedLoopController();
    azimuthEnc = azimuthMtr.getEncoder();

    azimuthMtrSparkFlexConfig.encoder
        .positionConversionFactor(TurretConstants.azimuthPositionConversionFactorRadPerRot);
    azimuthMtrSparkFlexConfig.encoder
        .velocityConversionFactor(TurretConstants.azimuthVelocityConversionFactorRadPerRotPerSec);

    leftFlyWheelMtrSparkFlexConfig.encoder
        .positionConversionFactor(TurretConstants.flyWheelPositionConversionFactorRot);
    leftFlyWheelMtrSparkFlexConfig.encoder
        .velocityConversionFactor(TurretConstants.flyWheelVelocityConversionFactorRPM);
    rightFlyWheelMtrSparkFlexConfig.encoder
        .positionConversionFactor(TurretConstants.flyWheelPositionConversionFactorRot);
    rightFlyWheelMtrSparkFlexConfig.encoder
        .velocityConversionFactor(TurretConstants.flyWheelVelocityConversionFactorRPM);

    azimuthMtrSparkFlexConfig.inverted(true);
    leftFlyWheelMtrSparkFlexConfig.inverted(true);
    rightFlyWheelMtrSparkFlexConfig.inverted(false);

    azimuthMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    leftFlyWheelMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
    rightFlyWheelMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);

    azimuthMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxAzimuthCurrentAmps);
    leftFlyWheelMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxFlyWheelCurrentAmps);
    rightFlyWheelMtrSparkFlexConfig.smartCurrentLimit(TurretConstants.maxFlyWheelCurrentAmps);

    azimuthMtrSparkFlexConfig.voltageCompensation(12);
    leftFlyWheelMtrSparkFlexConfig.voltageCompensation(12);
    rightFlyWheelMtrSparkFlexConfig.voltageCompensation(12);

    azimuthMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false); // MAKE THESE TRUE AFTER TUNING
    azimuthMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    // azimuthMtrSparkFlexConfig.softLimit.forwardSoftLimit(TurretConstants.maximumAizmuthAngleDeg);
    // azimuthMtrSparkFlexConfig.softLimit.reverseSoftLimit(TurretConstants.minimumAizmuthAngleDeg);

    leftFlyWheelMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    leftFlyWheelMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);
    rightFlyWheelMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    rightFlyWheelMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    rightFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flywheelkP)
        .d(TurretConstants.flywheelkD).feedForward.kS(TurretConstants.flywheelkS)
        .kV(TurretConstants.flywheelkV);
    // .kA(TurretConstants.flywheelkA);

    leftFlyWheelMtrSparkFlexConfig.closedLoop
        .p(TurretConstants.flywheelkP)
        .d(TurretConstants.flywheelkD)
        .feedForward
        .kS(TurretConstants.flywheelkS)
        .kV(TurretConstants.flywheelkV);
    // .kA(TurretConstants.flywheelkA);
    azimuthMtrSparkFlexConfig.closedLoop
      .p(TurretConstants.azimuthP)
      .d(TurretConstants.azimuthD)
      .feedForward
      .kS(TurretConstants.azimuthkS)
      .kV(TurretConstants.azimuthkV)
      .kA(TurretConstants.azimuthkA);
    leftFlyWheelMtr.configure(leftFlyWheelMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightFlyWheelMtr.configure(rightFlyWheelMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    azimuthMtr.configure(azimuthMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // initialzing sysID routines
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // default ramp rate
            Volts.of(12), // default step voltage
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
            this));
  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    // azimuthPID.setSetpoint(getCurrentAzimuthAngleRad(), ControlType.kPosition);
    // } else if (isAiming
    // && calculateTargetAzimuthAngle() <=
    // Units.degreesToRadians(TurretConstants.maximumAizmuthAngleDeg)
    // && calculateTargetAzimuthAngle() >=
    // Units.degreesToRadians(TurretConstants.minimumAizmuthAngleDeg)) {
    // azimuthPID.setSetpoint(calculateTargetAzimuthAngle(), ControlType.kPosition);
    // } /* for troubleshooting only, remove for competition */ else if
    // (manualAzimuthAngle != null) {
    // azimuthPID.setSetpoint(Units.degreesToRadians(manualAzimuthAngle), ControlType.kPosition);
    // } else {
    // azimuthPID.setSetpoint(Units.degreesToRadians(TurretConstants.azimuthDefaultSetpointDeg), ControlType.kPosition);
    // }
  
    if (isFiring) {
      setFlywheelRPM(calculateTargetFlywheelRPM());
    } else if (manualFlywheelRPM == 0.0) {
      leftFlyWheelMtr.stopMotor();
      rightFlyWheelMtr.stopMotor();
    } /*
       * for troubleshooting only, remove for competition (manualFlywheelRPM != null)
       */ else {
      setFlywheelRPM(manualFlywheelRPM + flywheelOffsetRPM);
    } // else {
    // setFlywheelRPM(0.0);
    // }
  }

  public void setManualAzimuthAngle(Double manualAzimuthAngle) {
    this.manualAzimuthAngle = manualAzimuthAngle;
  }

  public void setManualFlywheelRPM(Double TargetFlywheelRPM) {
    this.manualFlywheelRPM = TargetFlywheelRPM;
  }

  public double getManualFlywheelRPM() {
    return manualFlywheelRPM;
  }

  public boolean getIsFiring() {
    return isFiring;
  }

  public void setIsAiming(boolean isAiming) {
    this.isAiming = isAiming;
  }

  public boolean isAtSpeed() {
    return Math.abs(getFlywheelRPM() - calculateTargetFlywheelRPM())
        - 30.0 <= TurretConstants.flywheelRPMErrorTolerance;
  }

  public Pose2d calculateTurretPose() {
    return poseEstimator.getPose().transformBy(TurretConstants.robotToTurret);
  }

  public double calculateTargetAzimuthAngle() {
    return TurretConstants.targetPoseBlue.getTranslation()
        .minus(calculateTurretPose().getTranslation())
        .getAngle()
        .minus(new Rotation2d(poseEstimator.getPose().getRotation().getRadians())).getRadians();
  }
  // public double getTargetAzimuthRad() {
  //   return azimuthPID.get;
  // }

  public double calculateDistanceToTarget() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
      return calculateTurretPose().getTranslation().getDistance(TurretConstants.targetPoseBlue.getTranslation());
    } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return calculateTurretPose().getTranslation().getDistance(TurretConstants.targetPoseRed.getTranslation());
    } else {
      return calculateTurretPose().getTranslation().getDistance(TurretConstants.targetPoseBlue.getTranslation());
    }
  }

  public double incrementFlywheelOffsetRPM() {
    flywheelOffsetRPM += TurretConstants.flywheelOffsetRPMIncrement;
    return flywheelOffsetRPM;
  }

  public double decrementFlywheelOffsetRPM() {
    flywheelOffsetRPM -= TurretConstants.flywheelOffsetRPMIncrement;
    return flywheelOffsetRPM;
  }

  public double calculateTargetFlywheelRPM() {
    return flywheelOffsetRPM +
        TurretConstants.zerothDegreeFitConstant +
        TurretConstants.firstDegreeFitConstant * calculateDistanceToTarget() +
        TurretConstants.secondDegreeFitConstant * Math.pow(calculateDistanceToTarget(), 2);
  }

  public boolean isOnTarget() {
    return (Math.abs(getCurrentAzimuthAngleRad() - calculateTargetAzimuthAngle()) <= Units
        .degreesToRadians(TurretConstants.azimuthErrorTolerance));
  }

  public void setFlywheelRPM(double targetRPM) {
    leftFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
    rightFlyWheelPID.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public void setIsFiring(boolean isFiring) {
    this.isFiring = isFiring;
  }

  public double getFlywheelRPM() {
    return (leftFlyWheelEnc.getVelocity() + rightFlyWheelEnc.getVelocity()) / 2.0;
  }

  public double getFlywheelOffsetRPM() {
    return (flywheelOffsetRPM);
  }

  public double getCurrentAzimuthAngleRad() {
    return azimuthEnc.getPosition();
  }

  public double getAzimuthManualTargetDeg() {
    return manualAzimuthAngle;
  }

  public Pose2d getTurretPose() {
    return calculateTurretPose();
  }

  public Field2d getTurretField() {
    field.setRobotPose(getTurretPose());
    return field;
  }
  
  // for sysID charachterization only
  public void setAzimuthVoltage(double voltage) {
    azimuthMtr.setVoltage(voltage);
  }

  public double getCharacterizationVoltage() {
    return azimuthMtr.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

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
}