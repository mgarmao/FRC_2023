package frc.robot.subsystems;

import java.util.List;

import org.opencv.core.Mat;
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

    double xDistance;
    double yDistance;
    double zDistance;
    double h;
//     // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

    public Photon() {
        SmartDashboard.putBoolean("Photon",true);
    }
    
    public double getApriltagYaw(){
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }   
        return yaw;
    }

    public double apriltagDistanceX(){
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d targetToCamera = target.getBestCameraToTarget();
            xDistance = targetToCamera.getX();
        }
        else{
            xDistance = 0;
        }
        return xDistance;
    }

    public double apriltagDistanceY(){
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d targetToCamera = target.getBestCameraToTarget();
            yDistance = targetToCamera.getY();
        }
        else{
            yDistance = 0;
        }
        return yDistance;
    }

    public double apriltagZDistance(){
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d targetToCamera = target.getBestCameraToTarget();
            zDistance = targetToCamera.getY();
        }
        else{
            zDistance = 0;
        }
        return zDistance;
    }

    public double apriltagHypot(){
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d targetToCamera = target.getBestCameraToTarget();
            double y = targetToCamera.getY();
            double x = targetToCamera.getX();
            double xSqr = Math.pow(x,2);
            double ySqr = Math.pow(y,2);
            double hSqr = xSqr + ySqr;
            h = Math.sqrt(hSqr);
        }
        else{
            h = 0;
        }
        return h;
    }
    
    public double getConeYaw(){
        double yaw = 0;
        camera.setPipelineIndex(1);         
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets(); 
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }
        return yaw;
    }

    public double getAngle(int pipline){
        camera.setPipelineIndex(pipline);         
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        double angle = 0;
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d targetToCamera = target.getBestCameraToTarget();
            double x = targetToCamera.getX();
            double y = targetToCamera.getY();
            double tanTheta = y/x;
            angle = Math.atan(tanTheta);
        }

        return angle;
    }

    public boolean apriltagHasTarget(){
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        return hasTargets;
    }

    public boolean coneHasTarget(){
        camera.setPipelineIndex(1);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        return hasTargets;
    }

    @Override
    public void periodic() {
        camera.setPipelineIndex(0);
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();
        SmartDashboard.putBoolean("Has Target",hasTargets);

        if(hasTargets){
            // SmartDashboard.putNumber("AprilTag Yaw",getApriltagYaw());
            // SmartDashboard.putNumber("Cone Yaw",getConeYaw());
        }
    }
}