package frc.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.util.LimelightHelpers;

public class PoseEstimator extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDrive swerveDrive;
    
    private boolean trustLimelightOne = true;
    private boolean trustLimelightTwo = true;

    public PoseEstimator(SwerveDrive swerveDrive) {

        this.swerveDrive = swerveDrive;

        poseEstimator = new SwerveDrivePoseEstimator(
            SwerveDriveConstants.kinematics,
            swerveDrive.getHeading(),
            swerveDrive.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(
                VisionConstants.odomTranslationStdDevMeters,
                VisionConstants.odomTranslationStdDevMeters,
                VisionConstants.odomRotationStdDevRad),
            VecBuilder.fill(
                VisionConstants.visionTranslationStdDevMeters,
                VisionConstants.visionTranslationStdDevMeters,
                VisionConstants.visionRotationStdDevRad));
    }

    @Override
    public void periodic() {
        // filter and update pose based on vision from limelight one
        LimelightHelpers.SetRobotOrientation(VisionConstants.limelightOneName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), getAngularVelocityDegPerSec(), 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limeLightOnePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightOneName);
        trustLimelightOne = true;
        if(limeLightOnePose == null || limeLightOnePose.tagCount == 0 || swerveDrive.getAngularVelocityDegPerSec() > VisionConstants.angularVelocityDegPerSecThreshold) {
            trustLimelightOne = false;
        }
        if(trustLimelightOne) {
            poseEstimator.addVisionMeasurement(
            limeLightOnePose.pose,
            limeLightOnePose.timestampSeconds);
        }

        // filter and update pose based on vision from limelight two
        LimelightHelpers.SetRobotOrientation(VisionConstants.limelightTwoName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), getAngularVelocityDegPerSec(), 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limeLightTwoPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightTwoName);
        trustLimelightTwo = true;
        if(limeLightTwoPose == null || limeLightTwoPose.tagCount == 0 || swerveDrive.getAngularVelocityDegPerSec() > VisionConstants.angularVelocityDegPerSecThreshold) {
            trustLimelightTwo = false;
        }
        if(trustLimelightTwo) {
            poseEstimator.addVisionMeasurement(
            limeLightTwoPose.pose,
            limeLightTwoPose.timestampSeconds);
        }

        // update pose based on odometry
        poseEstimator.update(swerveDrive.getHeading(), swerveDrive.getModulePositions());
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(swerveDrive.getHeading(), swerveDrive.getModulePositions(), pose);
    }

    public void resetHeading() {
        poseEstimator.resetPosition(swerveDrive.getHeading(), swerveDrive.getModulePositions(), 
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
        // TODO: change based on field mirroring
        // DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)));
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public double getAngularVelocityDegPerSec() {
        return swerveDrive.getAngularVelocityDegPerSec();
    }
}