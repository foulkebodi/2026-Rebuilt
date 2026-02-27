// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.StartIntaking;
import frc.robot.commands.climber.SetHookPosition;
import frc.robot.commands.climber.incrimentClimberPosition;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.LockCmd;
import frc.robot.commands.intake.SetIntakeActuatorInches;
import frc.robot.commands.intake.SetIntakeRollerRPM;
import frc.robot.commands.spindexer.SetSpindexerRPM;
import frc.robot.commands.tower.SetTowerRPM;
import frc.robot.commands.turret.DecrementFlywheelOffset;
import frc.robot.commands.turret.IncrementFlywheelOffset;
import frc.robot.commands.turret.SetManualAzimuthAngle;
import frc.robot.commands.turret.SetManualFlywheelRPM;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.subsystems.ClimberSys.ElevatorState;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final PoseEstimator poseEstimator = new PoseEstimator(swerveDrive);
	private final IntakeSys intakeSys = new IntakeSys();
	private final IndexerSys indexerSys = new IndexerSys();
	private final TurretSys turretSys = new TurretSys(poseEstimator);
	private final ClimberSys climberSys = new ClimberSys();

	private final CommandXboxController driverController = new CommandXboxController(
			ControllerConstants.kDriverControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(
			ControllerConstants.kOperatorControllerPort);

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		// register named commands
		NamedCommands.registerCommand("exampleCommand", new Command() {
		});

		// configure autobuilder
		AutoBuilder.configure(
				poseEstimator::getPose,
				poseEstimator::resetPose,
				swerveDrive::getRobotRelativeSpeeds,
				(chassisSpeeds, feedforward) -> swerveDrive.driveRobotRelative(chassisSpeeds),
				new PPHolonomicDriveController(
						new PIDConstants(SwerveDriveConstants.autoTranslationKp,
								SwerveDriveConstants.autoTranslationKd),
						new PIDConstants(SwerveDriveConstants.autoRotationKp, SwerveDriveConstants.autoRotationKd)),
				new RobotConfig(RobotConstants.massKg, RobotConstants.momentOfInertiaKgMetersSq,
						SwerveModuleConstants.moduleConfig, SwerveDriveConstants.kinematics.getModules()),
				() -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				swerveDrive);

		// create auto
		// new PathPlannerAuto("TranslationTestOne");

		// build auto chooser
		autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");

		// send auto chooser to dashboard
		SmartDashboard.putData("auto chooser", autoChooser);

		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		// driver controls for competition
		swerveDrive.setDefaultCommand(new ArcadeDriveCmd(
				() -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
				() -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
				() -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
				true,
				swerveDrive,
				poseEstimator));

		driverController
				.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.tiggerPressedThreshold)
				.onTrue(new LockCmd(swerveDrive));

		driverController
				.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
				.onTrue(new StartIntaking(indexerSys, intakeSys))
				.onFalse(new SetIntakeRollerRPM(intakeSys, 0));

		// operator bindings for competition
		operatorController.rightBumper().onTrue(new IncrementFlywheelOffset(turretSys));
		operatorController.leftBumper().onTrue(new DecrementFlywheelOffset(turretSys));

		// operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value,
		// ControllerConstants.tiggerPressedThreshold)
		// .onTrue(new ManualAdjustIntakeCommand(intakeSys));
		// operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value,
		// ControllerConstants.tiggerPressedThreshold)
		// .onTrue(new ManualAdjustIntakeCommand(intakeSys));

		// operatorController.a().onTrue(new SetIntakeActuatorInches(intakeSys,
		// IntakeConstants.actuatorInPositionInches));
		// operatorController.y().onTrue(new SetIntakeActuatorInches(intakeSys,
		// IntakeConstants.actuatorOutPositionInches));

		// operatorController.b().onTrue(new SetHookPosition(climberSys,
		// Constants.ClimberConstants.hookOutPositionRotations));
		// operatorController.x().onTrue(new SetHookPosition(climberSys, 0.0));

		// operatorController.povUp().onTrue(new incrimentClimberPosition(climberSys));
		// operatorController.povDown().onTrue(new
		// incrimentClimberPosition(climberSys));

		// binding commands for swerve sysID
		// driverController.a().onTrue(swerveDrive.driveSysIdDynamicForward());
		// driverController.b().onTrue(swerveDrive.driveSysIdDynamicReverse());
		// driverController.x().onTrue(swerveDrive.driveSysIdQuasistaticForward());
		// driverController.y().onTrue(swerveDrive.driveSysIdQuasistaticReverse());

		// binding commands for turret sysID
		// driverController.a().onTrue(turretSys.sysIdDynamicForward());
		// driverController.b().onTrue(turretSys.sysIdDynamicReverse());
		// driverController.x().onTrue(turretSys.sysIdQuasistaticForward());
		// driverController.y().onTrue(turretSys.sysIdQuasistaticReverse());

		// turret manual control bindings for testing (DISABLE SOFT LIMITS BEFORE USING)
		// operatorController.a().onTrue(new SetManualAzimuthAngle(turretSys, 0.0));
		// operatorController.b().onTrue(new SetManualAzimuthAngle(turretSys, 25.0));
		// operatorController.x().onTrue(new SetManualAzimuthAngle(turretSys, 120.0));
		// operatorController.y().onTrue(new SetManualAzimuthAngle(turretSys, -90.0));

		// flywheel RPM control bindings for testing
		// operatorController.a().onTrue(new SetManualFlywheelRPM(turretSys, 0.0));
		// operatorController.b().onTrue(new SetManualFlywheelRPM(turretSys, 2000.0));
		// operatorController.x().onTrue(new SetManualFlywheelRPM(turretSys, 1000.0));
		// driverController.y().onTrue(new SetManualFlywheelRPM(turretSys, 7000.0))

		// spindexer RPM control bindings for testing
		// operatorController.a().onTrue(new SetSpindexerRPM(indexerSys, 0.0));
		// operatorController.b().onTrue(new SetSpindexerRPM(indexerSys, 100.0));
		// operatorController.x().onTrue(new SetSpindexerRPM(indexerSys, 1000.0));
		// operatorController.y().onTrue(new SetSpindexerRPM(indexerSys, 7000.0));

		// tower RPM control bindings for testing
		// operatorController.a().onTrue(new SetTowerRPM(indexerSys, 0.0));
		// operatorController.b().onTrue(new SetTowerRPM(indexerSys, 100.0));
		// driverController.x().onTrue(new SetTowerRPM(indexerSys,1000.0));
		// operatorController.y().onTrue(new SetTowerRPM(indexerSys, 7000.0));

		// intake roller RPM control bindings for testing
		// operatorController.a().onTrue(new SetIntakeRollerRPM(intakeSys, 0.0));
		// operatorController.b().onTrue(new SetIntakeRollerRPM(intakeSys, 0.0));
		// operatorController.x().onTrue(new SetIntakeRollerRPM(intakeSys, 2200.0));
		// operatorController.y().onTrue(new SetIntakeRollerRPM(intakeSys, 7000.0));

		// intake actuator position control bindings for testing
		// operatorController.a().onTrue(new SetIntakeActuatorInches(intakeSys, 0.0));
		// operatorController.b().onTrue(new SetIntakeActuatorInches(intakeSys, 8.0));
		// operatorController.x().onTrue(new SetIntakeActuatorInches(intakeSys, 0.0));
		// operatorController.y().onTrue(new SetIntakeActuatorInches(intakeSys,
		// IntakeConstants.actuatorOutPositionInches));

		// example operator bindings
		// operatorController.a().onTrue(new Command() {});
		// operatorController.b().onTrue(new Command() {});
		// operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value,
		// ControllerConstants.tiggerPressedThreshold)
		// .onTrue(new Command() {});
	}

	public Command getAutonomousCommand() {
		// return autoChooser.getSelected();
		// return new ExampleCommand(null);
		return new Command() {
		};
	}

	public void updateDashboard() {
		// drive base
		SmartDashboard.putNumber("FL CANcoder", swerveDrive.getCanCoderAngles()[0].getDegrees());
		SmartDashboard.putNumber("FR CANcoder", swerveDrive.getCanCoderAngles()[1].getDegrees());
		SmartDashboard.putNumber("BL CANcoder", swerveDrive.getCanCoderAngles()[2].getDegrees());
		SmartDashboard.putNumber("BR CANcoder", swerveDrive.getCanCoderAngles()[3].getDegrees());
		// pose info from pose estimator
		SmartDashboard.putNumber("pos-x", poseEstimator.getPose().getX());
		SmartDashboard.putNumber("pos-y", poseEstimator.getPose().getY());
		SmartDashboard.putNumber("pos-rot", poseEstimator.getHeading().getDegrees());
		SmartDashboard.putNumber("gyro heading", swerveDrive.getHeading().getDegrees());
		// turret azimuth info
		SmartDashboard.putNumber("current azimuth angle rad", turretSys.getCurrentAzimuthAngleRad());
		SmartDashboard.putNumber("target azimuth angle rad", turretSys.calculateTargetAzimuthAngle());
		SmartDashboard.putBoolean("on target", turretSys.isOnTarget());
		SmartDashboard.putNumberArray("turret pose", new double[] {
				turretSys.getTurretPose().getTranslation().getX(),
				turretSys.getTurretPose().getTranslation().getY() });
		// turret flywheel
		SmartDashboard.putNumber("flywheel current RPM", turretSys.getFlywheelRPM());
		SmartDashboard.putNumber("flywheel target RPM", turretSys.calculateTargetFlywheelRPM());
		SmartDashboard.putNumber("flywheel manual rpm", turretSys.getManualFlywheelRPM());
		SmartDashboard.putNumber("distance to target", turretSys.calculateDistanceToTarget());
		SmartDashboard.putBoolean("is Firing", turretSys.getIsFiring());
		// indexer info
		SmartDashboard.putNumber("tower RPM", indexerSys.getTowerRPM());
		SmartDashboard.putNumber("spindexer RPM", indexerSys.getSpindexerRPM());
		// intake info
		SmartDashboard.putNumber("actuator position inches", intakeSys.getActuatorPositionInches());
		SmartDashboard.putNumber("roller RPM", intakeSys.getRollerRPM());
		// climber info
		SmartDashboard.putNumber("climber extension inches", climberSys.getClimberPosition());
		SmartDashboard.putNumber("climber hook position degrees", climberSys.getHookPosition());
		SmartDashboard.putString("climber current state", climberSys.getCurrentState().toString());
	}
}