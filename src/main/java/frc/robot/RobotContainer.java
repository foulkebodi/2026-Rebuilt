// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.StartIntaking;
import frc.robot.commands.StartShooting;
import frc.robot.commands.StopShooting;
import frc.robot.commands.climber.SetClimberPositon;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.LockCmd;
import frc.robot.commands.intake.SetIntakeActuatorInches;
import frc.robot.commands.intake.SetIntakeRollerRPM;
import frc.robot.commands.spindexer.SetSpindexerRPM;
import frc.robot.commands.tower.SetTowerRPM;
import frc.robot.commands.turret.DecrementAzimuthOffset;
import frc.robot.commands.turret.DecrementFlywheelOffset;
import frc.robot.commands.turret.IncrementAzimuthOffset;
import frc.robot.commands.turret.IncrementFlywheelOffset;
import frc.robot.commands.turret.SetManualFlywheelRPM;
import frc.robot.commands.turret.StartAiming;
import frc.robot.commands.turret.StopAiming;
import frc.robot.commands.turret.StopManualAzimuthAngle;
import frc.robot.commands.turret.StopManualFlywheelRPM;
import frc.robot.commands.turret.ToggleIsPassing;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.TurretSys;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
	private RobotConfig config;

	public RobotContainer() {

		// register named commands
		NamedCommands.registerCommand("StartIntaking", new StartIntaking(intakeSys));
		NamedCommands.registerCommand("StartShooting", new StartShooting(turretSys, indexerSys, intakeSys));
		NamedCommands.registerCommand("StopShooting", new StopShooting(turretSys, indexerSys, intakeSys));
		NamedCommands.registerCommand("ClimberUp", new Command() {
		});
		NamedCommands.registerCommand("ClimberDown", new Command() {
		});

		// configure autobuilder
		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
		}

		AutoBuilder.configure(
				poseEstimator::getPose,
				poseEstimator::resetPose,
				swerveDrive::getRobotRelativeSpeeds,
				(chassisSpeeds, feedforward) -> swerveDrive.driveRobotRelative(chassisSpeeds),
				new PPHolonomicDriveController(
						new PIDConstants(SwerveDriveConstants.autoTranslationKp,
								SwerveDriveConstants.autoTranslationKd),
						new PIDConstants(SwerveDriveConstants.autoRotationKp, SwerveDriveConstants.autoRotationKd)),
				config,
				() -> {
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				swerveDrive);

		// create test autos
		new PathPlannerAuto("SquigglePathTest");
		new PathPlannerAuto("TranslationTest");
		new PathPlannerAuto("TurningWhileMovingTest");

		// create competition autos
		new PathPlannerAuto("LoadingStation");

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

		// driverController.start().onTrue(Commands.runOnce(
				// () -> poseEstimator.resetPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0))), poseEstimator));
		driverController.start().onTrue(Commands.runOnce(() -> poseEstimator.resetHeading(), poseEstimator));

		driverController
				.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.tiggerPressedThreshold)
				.onTrue(new LockCmd(swerveDrive))
				.onTrue(new StartShooting(turretSys, indexerSys, intakeSys))
				.onFalse(new StopShooting(turretSys, indexerSys, intakeSys));

		driverController
				.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
				.onTrue(new StartIntaking(intakeSys))
				.onFalse(new SetIntakeRollerRPM(intakeSys, 0));

		// operator bindings for competition
		operatorController.povUp().onTrue(new IncrementFlywheelOffset(turretSys));
		operatorController.povDown().onTrue(new DecrementFlywheelOffset(turretSys));
		operatorController.povLeft().onTrue(new IncrementAzimuthOffset(turretSys));
		operatorController.povRight().onTrue(new DecrementAzimuthOffset(turretSys));
		operatorController.rightStick().onTrue(new ToggleIsPassing(turretSys));
		operatorController.a().onTrue(new SetClimberPositon(climberSys, 0.0));
		operatorController.b().onTrue(new SetClimberPositon(climberSys, 3.5));
		operatorController.y().onTrue(new SetClimberPositon(climberSys, ClimberConstants.ElevatorMaxInches));

		
		// operatorController.x().onTrue(new StopManualFlywheelRPM(turretSys));
		// operatorController.a().onTrue(new SetManualFlywheelRPM(turretSys, 1500.0));
		// operatorController
		// 		.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.tiggerPressedThreshold)
		// 		.onTrue(new SetIntakeRollerRPM(intakeSys, IntakeConstants.intakingRollerRPM))
		// 		.onTrue(new SetSpindexerRPM(indexerSys, IndexerConstants.spindexerAgitatingRPM))
		// 		.onTrue(new SetTowerRPM(indexerSys, IndexerConstants.towerShootingRPM))
		// 		.onFalse(new SetIntakeRollerRPM(intakeSys, 0))
		// 		.onFalse(new SetSpindexerRPM(indexerSys, 0))
		// 		.onFalse(new SetTowerRPM(indexerSys, 0));
		
		// operatorController.povRight().onTrue(new SetIntakeActuatorInches(intakeSys, IntakeConstants.actuatorOutPositionInches));
		// operatorController.povLeft().onTrue(new SetIntakeActuatorInches(intakeSys, 5.0));

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
		// operatorController.a().onTrue(new StartAiming(turretSys));
		// operatorController.b().onTrue(new StopAiming(turretSys));
		// operatorController.x().onTrue(new StopManualAzimuthAngle(turretSys));

		// flywheel RPM control bindings for testing
		// operatorController.x().onTrue(new SetManualFlywheelRPM(turretSys, 0.0));
		// operatorController.b().onTrue(new SetManualFlywheelRPM(turretSys, 2000.0));
		// operatorController.a().onTrue(new SetManualFlywheelRPM(turretSys, 1000.0));
		// operatorController.y().onTrue(new SetManualFlywheelRPM(turretSys, 4800.0));

		// spindexer RPM control bindings for testing
		// operatorController.a().onTrue(new SetSpindexerRPM(indexerSys, 0.0));
		// operatorController.b().onTrue(new SetSpindexerRPM(indexerSys, 100.0));
		// operatorController.x().onTrue(new SetSpindexerRPM(indexerSys, 1000.0));
		// operatorController.y().onTrue(new SetSpindexerRPM(indexerSys, 7000.0));

		// tower RPM control bindings for testing
		// operatorController.a().onTrue(new SetTowerRPM(indexerSys, 1000.0));
		// operatorController.b().onTrue(new SetTowerRPM(indexerSys, 3000.0));
		// operatorController.x().onTrue(new SetTowerRPM(indexerSys, 0.0));
		// operatorController.y().onTrue(new SetTowerRPM(indexerSys, 4000.0));

		// intake roller RPM control bindings for testing
		// operatorController.a().onTrue(new SetIntakeRollerRPM(intakeSys, 0.0));
		// operatorController.x().onTrue(new SetIntakeRollerRPM(intakeSys, 2200.0));
		// operatorController.y().onTrue(new SetIntakeRollerRPM(intakeSys, 7000.0));

		// intake actuator position control bindings for testing
		// operatorController.b().onTrue(new SetIntakeActuatorInches(intakeSys, 8.0));
		// operatorController.x().onTrue(new SetIntakeActuatorInches(intakeSys, 0.0));
		// operatorController.y().onTrue(new SetIntakeActuatorInches(intakeSys,IntakeConstants.actuatorOutPositionInches));

		// climber position control bindings for testing
		// operatorController.a().onTrue(new SetClimberPositon(climberSys, 0.0));
		// operatorController.b().onTrue(new SetClimberPositon(climberSys, 3.5));
		// operatorController.y().onTrue(new SetClimberPositon(climberSys, ClimberConstants.ElevatorMaxInches));

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
		// return new ExampleCommand(null);
		// return new Command() {
	};

	public void updateDashboard() {
		// drive base
		SmartDashboard.putNumber("FL CANcoder", swerveDrive.getCanCoderAngles()[0].getDegrees());
		SmartDashboard.putNumber("FR CANcoder", swerveDrive.getCanCoderAngles()[1].getDegrees());
		SmartDashboard.putNumber("BL CANcoder", swerveDrive.getCanCoderAngles()[2].getDegrees());
		SmartDashboard.putNumber("BR CANcoder", swerveDrive.getCanCoderAngles()[3].getDegrees());
		SmartDashboard.putNumber("field relative speed", swerveDrive.getRobotVelocity());

		// pose info from pose estimator
		SmartDashboard.putNumber("pos-x", poseEstimator.getPose().getX());
		SmartDashboard.putNumber("pos-y", poseEstimator.getPose().getY());
		SmartDashboard.putNumber("pos-rot", poseEstimator.getHeading().getDegrees());
		// SmartDashboard.putNumber("gyro heading", swerveDrive.getHeading().getDegrees());

		// turret azimuth info
		SmartDashboard.putNumber("manual azimuth angle rads", turretSys.getAzimuthManualTargetDeg());
		SmartDashboard.putNumber("current azimuth angle rad", turretSys.getCurrentAzimuthAngleRad());
		SmartDashboard.putNumber("target azimuth angle rad", turretSys.calculateTargetAzimuthAngleShooting());
		SmartDashboard.putBoolean("on target", turretSys.isOnTarget());
		SmartDashboard.putNumberArray("turret pose", new double[] {
				turretSys.getTurretPose().getTranslation().getX(),
				turretSys.getTurretPose().getTranslation().getY()});
		SmartDashboard.putBoolean("is Aiming", turretSys.getIsAiming());
		SmartDashboard.putBoolean("is Passing", turretSys.getIsPassing());

		// turret flywheel
		SmartDashboard.putNumber("flywheel current RPM", turretSys.getFlywheelRPM());
		SmartDashboard.putNumber("flywheel target RPM", turretSys.calculateTargetFlywheelRPM());
		SmartDashboard.putNumber("flywheel manual rpm", turretSys.getManualFlywheelRPM());
		SmartDashboard.putNumber("distance to target", turretSys.calculateDistanceToTarget());
		SmartDashboard.putBoolean("is Firing", turretSys.getIsFiring());
		SmartDashboard.putNumber("flywheel offset RPM", turretSys.getFlywheelOffsetRPM());

		// indexer info
		SmartDashboard.putNumber("tower RPM", indexerSys.getTowerRPM());
		SmartDashboard.putNumber("spindexer RPM", indexerSys.getSpindexerRPM());

		// intake info
		SmartDashboard.putNumber("actuator position inches", intakeSys.getActuatorPositionInches());
		SmartDashboard.putNumber("roller RPM", intakeSys.getRollerRPM());

		// climber info
		SmartDashboard.putNumber("climber extension inches", climberSys.getClimberPosition());

		// field
		SmartDashboard.putData("robot field", poseEstimator.getField());
		SmartDashboard.putData("turret field", turretSys.getTurretField());
	}
}