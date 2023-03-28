package frc.robot.subsystems;

import java.util.List;

import javax.print.attribute.standard.MediaSize.NA;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase{
    public PhotonCamera camera = new PhotonCamera("OV5647");
    // Angle between horizontal and the camera.
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
    
    int requestedPipeline = 0;

    int apriltagPipeline = 0;
    int conePipeline = 1;
    int cubePipeline = 2;

    // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

    public Photon() {
        SmartDashboard.putBoolean("Photon",true);
        camera.setPipelineIndex(0);
    }
    
    public void setPipline(int m_pipline){
        requestedPipeline = m_pipline;
    }

    public double getApriltagYaw(){
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }   
        return yaw;
    }

    public double apriltagDistanceX(){
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

    public double apriltagDistanceY(int id){
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            
            // PhotonTrackedTarget target = result.getBestTarget();
            // Transform3d targetToCamera = target.getBestCameraToTarget();
            // yDistance = targetToCamera.getY();

            List<PhotonTrackedTarget> targets = result.getTargets();
            for(PhotonTrackedTarget target:targets){
                Transform3d thisTarget = target.getBestCameraToTarget();
                SmartDashboard.putNumber("This ID", target.getFiducialId());
                if(target.getFiducialId()==id){
                    yDistance = thisTarget.getY();                
                }
            }
        }
        else{
            yDistance = 0;
        }
        return yDistance;
    }

    public double apriltagDistanceYBest(){
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d thisTarget = target.getBestCameraToTarget();
            yDistance = thisTarget.getY();                
        }
        else{
            yDistance = 0;
        }
        return yDistance;
    }

    public int apriltagID(){
        int id;
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            id = target.getFiducialId();
        }
        else{
            id = 100;
        }
        return id;
    }

    public double apriltagZDistance(){
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();        
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d targetToCamera = target.getBestCameraToTarget();
        zDistance = targetToCamera.getY();
        
    
        return zDistance;
    }

    public double apriltagHypot(){
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
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets(); 
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }
        return yaw;
    }

    public double getCubeYaw(int pipline){
        // camera.setPipelineIndex(pipline);
        double yaw = 0;
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets(); 
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = target.getYaw();
        }
        return yaw;
    }

    public double getAngle(int pipline){
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
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        return hasTargets;
    }

    public boolean coneHasTarget(){
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        return hasTargets;
    }

    @Override
    public void periodic() {
        camera.setPipelineIndex(requestedPipeline);
    }
}