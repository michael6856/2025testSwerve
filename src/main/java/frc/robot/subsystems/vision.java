package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase{
    Field2d m_Field2d;
    AprilTagFieldLayout aprilTagFieldLayout;
    Pose3d robotPose;
    PhotonPipelineResult result;
    PhotonTrackedTarget target;
    PhotonPoseEstimator photonPoseEstimator;
    Pose2d prevEstimatedRobotPose;

    public PhotonCamera cameraFR = new PhotonCamera("CameraFR");
    private Transform3d robotTocam = new Transform3d(-0.2667, 0.2667, 0.22, new Rotation3d(0, Math.toRadians(20), Math.toRadians(-35)));

    public vision(){
        try{
            aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
        }catch (Exception error){
            aprilTagFieldLayout = null;
        }
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotTocam);
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(result);
        }

    public Pose3d pose3d(){
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotTocam);
        return photonPoseEstimator.getReferencePose();
    }
    

    // public Pose3d robotPose3d(){
    //     result = cameraFR.getLatestResult();
    //     target = result.getBestTarget();

    //      if(target != null){
    //         Transform3d cameraToTarget = target.getBestCameraToTarget();

    //         Optional<Pose3d> TagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

    //         if(TagPose.isPresent() && target.getFiducialId() != -1){
    //             robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, TagPose.get(), robotTocam);
    //         }
    //         else {
    //             robotPose = new Pose3d();
    //         }
    //         return robotPose;
    //     }
    //     return new Pose3d();
    // }
    
    @Override
    public void periodic(){
        double aaa;
        pose3d();
        //robotPose3d();
        // if(robotPose3d()!= null){
        //     m_Field2d.setRobotPose(robotPose3d().toPose2d());
        //     SmartDashboard.putData("Field2d", m_Field2d);
        // }
        // else{
            
        // }
        if(pose3d() != null){
            aaa = pose3d().getX();
        }
        else{
            aaa = 0;
        }
        SmartDashboard.putNumber("jjjj", aaa);
        //SmartDashboard.putBoolean("hasss", result.hasTargets());
       
    }
}
