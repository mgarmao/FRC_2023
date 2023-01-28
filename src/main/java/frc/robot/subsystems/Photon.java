package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera("OV5647");
//     // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);    
    double CAMERA_HEIGHT_METERS = 1;
    double TARGET_HEIGHT_METERS =1;
    double yaw = 0;
    double kCameraHeight = 1;
    double kTargetHeight = 1;
    double kCameraPitch = 1;
    double kTargetPitch = 1;
//     // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

    public Photon() {
        SmartDashboard.putBoolean("Photon",true);
    }
    
    public double getYaw(){
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }   
        return yaw;
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();
        List<PhotonTrackedTarget> targets = result.getTargets();

        SmartDashboard.putBoolean("Has Target",hasTargets);
        if(hasTargets){
            // SmartDashboard.putNumber("Target 1 ID",targets.get(1).getFiducialId());
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            int id = target.getFiducialId();
            double poseAmb = target.getPoseAmbiguity();
            Transform3d targetToCamera = target.getBestCameraToTarget();
            
            SmartDashboard.putNumber("yaw",yaw);
            SmartDashboard.putNumber("pitch",pitch);
            SmartDashboard.putNumber("area",area);
            SmartDashboard.putNumber("skew",skew);
            SmartDashboard.putNumber("id",id);
            SmartDashboard.putNumber("poseAmb",poseAmb);
            SmartDashboard.putNumber("targetToCamera X",targetToCamera.getX());
            SmartDashboard.putNumber("targetToCamera Y",targetToCamera.getY());
            SmartDashboard.putNumber("targetToCamera Z",targetToCamera.getZ());

        }
    }


        


        
//         // double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);


//     // public void autoAline(){
//     //     var result = camera.getLatestResult();
//     //     PhotonTrackedTarget target = result.getBestTarget();

//     //     double range = PhotonUtils.calculateDistanceToTargetMeters(
//     //         CAMERA_HEIGHT_METERS,
//     //         TARGET_HEIGHT_METERS,
//     //         CAMERA_PITCH_RADIANS,
//     //         Units.degreesToRadians(result.getBestTarget().getPitch()));

//     //     Pose2D robotPose = PhotonUtils.estimateFieldToRobot(kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);
//     //     double forwardSpeed = -controller.calculate(range, 2);
//     //     double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);

//     // }
}