package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.util.LimelightHelpers;

public class PoseEstimator extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Supplier<Rotation2d> gyroHeadingSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;
    private final DoubleSupplier angularVelocitySupplier;

    private boolean trustLimelightOne = true;
    private boolean trustLimelightTwo = true;

    public PoseEstimator(
        SwerveDriveKinematics swerveKinematics,
        Supplier<Rotation2d> gyroHeadingSupplier,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier,
        DoubleSupplier angularVelocitySupplier) {

        poseEstimator = new SwerveDrivePoseEstimator(
            swerveKinematics,
            gyroHeadingSupplier.get(),
            modulePositionsSupplier.get(),
            new Pose2d(),
            VecBuilder.fill(
                VisionConstants.odomTranslationStdDevMeters,
                VisionConstants.odomTranslationStdDevMeters,
                VisionConstants.odomRotationStdDevRad),
            VecBuilder.fill(
                VisionConstants.visionTranslationStdDevMeters,
                VisionConstants.visionTranslationStdDevMeters,
                VisionConstants.visionRotationStdDevRad));

        this.gyroHeadingSupplier = gyroHeadingSupplier;
        this.modulePositionsSupplier = modulePositionsSupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
    }

    @Override
    public void periodic() {
        // filter and update pose based on vision from limelight one
        LimelightHelpers.SetRobotOrientation(VisionConstants.limelightOneName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limeLightOnePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightOneName);
        trustLimelightOne = true;
        if(angularVelocitySupplier.getAsDouble() > 360) {
            trustLimelightOne = false;
        }
        if(limeLightOnePose.tagCount == 0) {
            trustLimelightOne = false;
        }
        if(trustLimelightOne) {
            poseEstimator.addVisionMeasurement(
            limeLightOnePose.pose,
            limeLightOnePose.timestampSeconds);
        } 

        // filter and update pose based on vision from limelight two
        LimelightHelpers.SetRobotOrientation(VisionConstants.limelightTwoName, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate limeLightTwoPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.limelightTwoName);
        trustLimelightTwo = true;
        if(angularVelocitySupplier.getAsDouble() > 360) {
            trustLimelightTwo = false;
        }
        if(limeLightTwoPose.tagCount == 0) {
            trustLimelightTwo = false;
        }
        if(trustLimelightTwo) {
            poseEstimator.addVisionMeasurement(
            limeLightTwoPose.pose,
            limeLightTwoPose.timestampSeconds);
        }

        // update pose based on odometry
        poseEstimator.update(gyroHeadingSupplier.get(), modulePositionsSupplier.get());
    }

    public Pose2d get() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyroHeadingSupplier.get(), modulePositionsSupplier.get(), pose);
    }

    public void resetHeading() {
        poseEstimator.resetPosition(gyroHeadingSupplier.get(), modulePositionsSupplier.get(), 
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
        // TODO: change based on field mirroring
        // DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)));
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return gyroHeadingSupplier.get();
    }

    public double getAngularVelocityDegPerSec() {
        return angularVelocitySupplier.getAsDouble();
    }

    public Pose3d getPose3d() {
        Pose3d position3d = new Pose3d(getPose().getX(), getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, getPose().getRotation().getRadians()) );
        return position3d;
    }
}