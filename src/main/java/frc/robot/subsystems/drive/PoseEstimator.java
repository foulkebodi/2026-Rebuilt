package frc.robot.subsystems.drive;

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
    

    public PoseEstimator(
        SwerveDriveKinematics swerveKinematics,
        Supplier<Rotation2d> gyroHeadingSupplier,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier) {

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
    }

    @Override
    public void periodic() {
        if (LimelightHelpers.getTV(VisionConstants.limelightOneName)) {
            poseEstimator.addVisionMeasurement(getlimelightOnePose(), getLimelightOneTimestamp());
        }
        if (LimelightHelpers.getTV(VisionConstants.limelightTwoName)) {
            poseEstimator.addVisionMeasurement(getlimelightTwoPose(), getLimelightTwoTimestamp());
        }
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

    public Pose2d getlimelightOnePose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightOneName);
        return limelightMeasurement.pose;
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return gyroHeadingSupplier.get(); // or your own heading method
    }

    public Pose3d getPose3d() {
        Pose3d position3d = new Pose3d(getPose().getX(), getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, getPose().getRotation().getRadians()) );
        return position3d;
    }

    public double getLimelightOneTimestamp() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightOneName);
        return limelightMeasurement.timestampSeconds;
    }    

    public Pose2d getlimelightTwoPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightTwoName);
        return limelightMeasurement.pose;
    }

    public double getLimelightTwoTimestamp() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.limelightTwoName);
        return limelightMeasurement.timestampSeconds;
    }
}