package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

public class AimToHubCmd extends Command {

    /**
     * Command to allow for driver input in teleop
     */
    private final SwerveDrive swerveSys;
    private final PoseEstimator poseEstimator;

    private Translation2d targetTranslation;

    private final ProfiledPIDController aimController;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
   

    /**
     * Constructs a new ArcadeDriveCmd.
     * 
     * <p>ArcadeDriveCmd is used to control the swerve drive base with arcade drive.
     * 
     */
    public AimToHubCmd(SwerveDrive swerveSys, PoseEstimator poseEstimator) {
        this.swerveSys = swerveSys;
        this.poseEstimator = poseEstimator;

         aimController = new ProfiledPIDController(
            Constants.SwerveDriveConstants.autoAimkP, 0.0, Constants.SwerveDriveConstants.autoAimkD,
            new Constraints(
                Constants.SwerveDriveConstants.autoAimTurnSpeedRadPerSec,
                Constants.SwerveDriveConstants.autoAimTurnAccelRadPerSecSq));

        aimController.enableContinuousInput(-Math.PI, Math.PI);

        

        

        addRequirements(swerveSys, poseEstimator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            targetTranslation = Constants.FieldConstants.redAllianceHubPose;
        }
        else {
            targetTranslation = Constants.FieldConstants.blueAllianceHubPose;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d robotTranslation = poseEstimator.getPose().getTranslation();
Translation2d targetOffset = targetTranslation.minus(robotTranslation);

// Desired heading
Rotation2d targetHeading = Rotation2d.fromRadians(
    MathUtil.angleModulus(targetOffset.getAngle().getRadians())
);

SmartDashboard.putNumber("target heading deg", targetHeading.getDegrees());


// Simple PID aim
if (Math.abs(poseEstimator.getHeading().getRadians() - targetHeading.getRadians())
        > Constants.SwerveDriveConstants.autoAimToleranceRad) {

    double aimRadPerSec = aimController.calculate(
        poseEstimator.getHeading().getRadians(),
        targetHeading.getRadians()
    );

    swerveSys.setOmegaOverrideRadPerSec(Optional.of(aimRadPerSec));
} else {
    swerveSys.setOmegaOverrideRadPerSec(Optional.of(0.0));
}
        
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
         swerveSys.setOmegaOverrideRadPerSec(Optional.empty());
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}