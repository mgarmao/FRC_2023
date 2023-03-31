package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Photon extends SubsystemBase{
    public PhotonCamera camera = new PhotonCamera("F-Camera");
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);    

    
    

    int apriltagPipeline = 0;
    int conePipeline = 1;
    int cubePipeline = 2;

    // AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2022RapidReact.m_resourceFile));

    public Photon() {
        SmartDashboard.putBoolean("Photon",true);
    }
    
    public void setPipline(int m_pipline){
        Constants.requestedPipeline = m_pipline;
    }

    public double getApriltagYaw(int ID){
        double yaw = 0;
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();    
        if(hasTargets){
            List<PhotonTrackedTarget> targets = result.getTargets();
            for(PhotonTrackedTarget target:targets){
                SmartDashboard.putNumber("This ID", target.getFiducialId());
                if(target.getFiducialId()==ID){
                    yaw = target.getYaw();                
                }
            }
        }    
        return yaw;
    }

    public double getApriltagDistanceX(int ID){
        double xDistance = 0;
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
                if(target.getFiducialId()==ID){
                    xDistance = thisTarget.getY();                
                }
            }
        }
        return xDistance;
    }

    public double getApriltagDistanceY(int id){
        double yDistance = 0;
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
        return yDistance;
    }

    public double getApriltagDistanceYBest(){
        double yDistance = 0;
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets();        
        if(hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d thisTarget = target.getBestCameraToTarget();
            yDistance = thisTarget.getY();                
        }
        return yDistance;
    }

    public int getApriltagID(){
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

    // public double apriltagZDistance(){
    //     var result = camera.getLatestResult();
    //     boolean hasTargets = result.hasTargets();        
    //     PhotonTrackedTarget target = result.getBestTarget();
    //     Transform3d targetToCamera = target.getBestCameraToTarget();
    //     zDistance = targetToCamera.getY();
        
    //     return zDistance;
    // }
    
    public boolean hasApriltag(int inputID){
        var result = camera.getLatestResult();         
        boolean hasTargets = result.hasTargets(); 
        boolean hasTargetWithID = false;       
        if(hasTargets){            
            // PhotonTrackedTarget target = result.getBestTarget();
            // Transform3d targetToCamera = target.getBestCameraToTarget();
            // yDistance = targetToCamera.getY();

            List<PhotonTrackedTarget> targets = result.getTargets();
            for(PhotonTrackedTarget target:targets){
                SmartDashboard.putNumber("This ID", target.getFiducialId());
                if(target.getFiducialId()==inputID){
                    hasTargetWithID = true;           
                }
            }
        }
        if(hasTargetWithID){
            return true;
        }
        else{
            return false;
        }
    }

    public double getApriltagHypot(){
        double h = 0;
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

    public double getCubeYaw(){
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

    public double getAngle(){
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
        camera.setPipelineIndex(Constants.requestedPipeline);
    }
}