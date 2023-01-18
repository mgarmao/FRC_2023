package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  boolean red = true;
  long startTime = System.nanoTime();
  long endTime = System.nanoTime();
  private NetworkTable table;
  private String tableName;
  /** Creates a new Limelight. */
  public Limelight() {
    tableName = "limelight";
    table = NetworkTableInstance.getDefault().getTable(tableName);
  }

  public double ConeY(){
    setPipeline(1);
    NetworkTableEntry ty = table.getEntry("ty");
    double ConeY = ty.getDouble(0.0);
    return ConeY;
  }

  public double ConeX(){
    setPipeline(1);
    NetworkTableEntry tx = table.getEntry("tx");
    double ConeX = tx.getDouble(0.0);
    return ConeX;
  }


  public double redBallX(){
    setPipeline(2);
    NetworkTableEntry ty = table.getEntry("ty");
    double blueBallY = ty.getDouble(0.0);
    return blueBallY;
  }

  public void setPipeline(Integer pipeline) {
    NetworkTableEntry pipelineEntry = table.getEntry("pipeline");
    pipelineEntry.setNumber(pipeline);
  }

  @Override
  public void periodic() {
    startTime = System.nanoTime();
    if(red){
      SmartDashboard.putNumber("Cone X:",ConeX());
      SmartDashboard.putNumber("Cone Y:",ConeY());
      // red=false;
    }
    else{
      SmartDashboard.putNumber("Red Ball X:",redBallX());
      red=true;
    }
    endTime = System.nanoTime();
  }
}

