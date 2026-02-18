// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.LockCmd;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.IndexerSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.SpindexerSys;
import frc.robot.subsystems.TowerSys;
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
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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

	private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.kOperatorControllerPort);

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		// register named commands
		NamedCommands.registerCommand("exampleCommand", new Command() {});

		// configure autobuilder
		AutoBuilder.configure(
			poseEstimator::get,
			poseEstimator::resetPose,
			swerveDrive::getRobotRelativeSpeeds, 
			(chassisSpeeds, feedforward) -> swerveDrive.driveRobotRelative(chassisSpeeds),
			new PPHolonomicDriveController(
				new PIDConstants(SwerveDriveConstants.autoTranslationKp, SwerveDriveConstants.autoTranslationKd),
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
		new PathPlannerAuto("TranslationTestOne");

		// build auto chooser
		autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");

		// send auto chooser to dashboard
		SmartDashboard.putData("auto chooser", autoChooser);
	
		// Configure the trigger bindings
		configureBindings();
	}

	
	private void configureBindings() {
		// drive controls
		swerveDrive.setDefaultCommand(new ArcadeDriveCmd(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
			true,
			swerveDrive,
			poseEstimator));
			
		driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.tiggerPressedThreshold)
		.onTrue(new LockCmd(swerveDrive));

		// binding commands for sysID
		driverController.a().onTrue(swerveDrive.sysIdDynamicForward());
		driverController.b().onTrue(swerveDrive.sysIdDynamicReverse());
		driverController.x().onTrue(swerveDrive.sysIdQuasistaticForward());
		driverController.y().onTrue(swerveDrive.sysIdQuasistaticReverse());

		// example operator bindings
		operatorController.a().onTrue(new Command() {});
		operatorController.b().onTrue(new Command() {});
	
		operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
		.onTrue(new Command() {});
	}

	public Command getAutonomousCommand() {
		// return autoChooser.getSelected();
		// return new ExampleCommand(null);
		return new Command() {};
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("FL CANcoder", swerveDrive.getCanCoderAngles()[0].getDegrees());
		SmartDashboard.putNumber("FR CANcoder", swerveDrive.getCanCoderAngles()[1].getDegrees());
		SmartDashboard.putNumber("BL CANcoder", swerveDrive.getCanCoderAngles()[2].getDegrees());
		SmartDashboard.putNumber("BR CANcoder", swerveDrive.getCanCoderAngles()[3].getDegrees());
		SmartDashboard.putNumber("pos-x", poseEstimator.get().getX());
		SmartDashboard.putNumber("pos-y", poseEstimator.get().getY());
		SmartDashboard.putNumber("pos-rot", poseEstimator.get().getRotation().getDegrees());
	}
}