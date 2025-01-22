package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class photonvision extends SubsystemBase {

    public PhotonCamera camera = new PhotonCamera("limelight3");
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;

    public AprilTagFieldLayout aprilTagFieldLayout;
    public PhotonPoseEstimator photonPoseEstimator;
    public Field2d m_Field2d = new Field2d();    
    public boolean hasTargets;
    public boolean hasYaw;
    public double YawDegree;
    public double RobotYaw;
    public double YawToTarget;
    
    Rotation2d Yaw;
    Transform3d cameraTorobot = new Transform3d(-0.1225, 0.0, 0.32, new Rotation3d(0, 22.5, 0));
    Transform3d robotTocamera = new Transform3d(new Translation3d(0.42, 0, 0.22), new Rotation3d(0, 0, 0));
    Pose3d robotPose;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/limelight3");

    // AprilTag Field Layout
    public photonvision(){
        try{
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            System.out.println("apriltagfield loadSuccess");

        }catch (Exception error){
            aprilTagFieldLayout = null;
            System.out.println("apriltagfield null");
        }
        SmartDashboard.putData("Field", m_Field2d);
    }

    // Get RobotPose
    public Pose2d getRobotPose(){
        result = camera.getLatestResult();
        target = result.getBestTarget();

        if(target != null){
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Optional<Pose3d> TagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

            if(TagPose.isPresent() && target.getFiducialId() != -1){
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, TagPose.get(), cameraTorobot);
            }
            else {
                robotPose = new Pose3d();
            }
            System.out.println("Pose get");
            return robotPose.toPose2d();
        }
        System.out.println("Pose is null");
        return new Pose2d();
    }

    public Rotation2d getYawToPose(){
        if(target != null && robotPose != null){
        Optional<Pose3d> TagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        
            if(target.getFiducialId() != -1)
                Yaw = PhotonUtils.getYawToPose(robotPose.toPose2d(), TagPose.get().toPose2d());
                YawDegree = Yaw.getDegrees();
                System.out.println("Yaw get");
                hasYaw = true;
                return Yaw;
        }
        System.out.println("Yaw is null");
        hasYaw = false;
        return Yaw = null;
    }

    public double getYawToTarget(){    
        if(target != null){
            
            if(target.getFiducialId() == 3 || target.getFiducialId() == 4){
            
                if(Yaw != null){
                    RobotYaw = -robotPose.toPose2d().getRotation().getDegrees();
    
                    if(YawDegree > 0){
                        YawDegree = Yaw.getDegrees();
                    }
                    else if (YawDegree < 0){
                        YawDegree = Yaw.getDegrees();
                    } 
                    else{
                        YawDegree = 0;
                    }
        
                    if(RobotYaw > 0){
                        RobotYaw = robotPose.toPose2d().getRotation().getDegrees();
                    }
                    else if (RobotYaw < 0){
                        RobotYaw = robotPose.toPose2d().getRotation().getDegrees();
                    } 
                    else if (RobotYaw == 0){
                        RobotYaw = 0;
                    }
                    else if (!hasYaw) {
                        RobotYaw = 0;
                    }
                    
                    if(RobotYaw - YawDegree < -180){
                        YawToTarget = YawDegree - RobotYaw - 360;
                    }
                    else if(RobotYaw - YawDegree > 180){
                        YawToTarget = YawDegree - RobotYaw + 360;
                    }
                    return YawToTarget;
                }  
                return 0;
            }
            else if(target.getFiducialId() == 7 || target.getFiducialId() == 8){

                if(Yaw != null){
                    RobotYaw = -robotPose.toPose2d().getRotation().getDegrees() ;
    
                    if(YawDegree > 0){
                        YawDegree = Yaw.getDegrees();
                    }
                    else if (YawDegree < 0){
                        YawDegree = Yaw.getDegrees();
                    } 
                    else{
                        YawDegree = 0;
                    }
        
                    if(RobotYaw > 0){
                        RobotYaw = 180 - RobotYaw;
                    }
                    else if (RobotYaw < 0){
                        RobotYaw = -180 - RobotYaw;
                    } 
                    else if (RobotYaw == 0){
                        RobotYaw = 0;
                    }
                    else if (!hasYaw) {
                        RobotYaw = 0;
                    }

                    if(YawDegree - RobotYaw < -180){
                        YawToTarget = YawDegree - RobotYaw + 360;
                    }
                    else if(YawDegree - RobotYaw > 180){
                        YawToTarget = YawDegree - RobotYaw - 360;
                    }
                }
                else{
                    return 0;
                }
            }
        }
        return 0;
    }

        
    /*swere
    // Multitag Estimator
    public void PhotonPoseEstimator(){
        if (target != null){
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotTocamera);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            
        }
        else{
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotTocamera); 
        }
    }
    */
    
    @Override
    public void periodic(){
        getRobotPose();
        getYawToPose();
        getYawToTarget();
        result = camera.getLatestResult();
        target = result.getBestTarget();
        SmartDashboard.putData("Field", m_Field2d);
        m_Field2d.setRobotPose(getRobotPose());
        SmartDashboard.putBoolean("hasTarget", hasTargets);
        SmartDashboard.putBoolean("hasYaw", hasYaw);
        SmartDashboard.putNumber("YawDegree", YawDegree);
        SmartDashboard.putNumber("RobotYaw", RobotYaw);
        SmartDashboard.putNumber("YawToTarget", YawToTarget);
    }


    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update(result);

    }

    public void update(){
        
    }

    public void updateOdometry(Pose2d pose){
        
    }
}
