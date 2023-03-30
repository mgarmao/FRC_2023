package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  
  private SerialPort serialPort;

  public LEDs() {
    try{
      serialPort = new SerialPort(9600, SerialPort.Port.kUSB); // Initialize serial communication at 9600 baud over USB
    }
    catch(Exception error){
      SmartDashboard.putBoolean("LEDs", false);
    }
  }

  public void sendData() {
    try{
      byte data = 1; // Define the integer value to send as a byte
      byte[] buffer = new byte[] { data }; // Create a byte array with the data to send
      serialPort.write(buffer, buffer.length); // Write the byte array to the serial port
      SmartDashboard.putBoolean("Writing Data",true);
      SmartDashboard.putString("Data From Arduino:",serialPort.readString());
    }
    catch(Exception err){

    }
  }
  
  public void sendData2(){
    try{
      byte data = 2; // Define the integer value to send as a byte
      byte[] buffer = new byte[] { data }; // Create a byte array with the data to send
      serialPort.write(buffer, buffer.length); // Write the byte array to the serial port
      SmartDashboard.putBoolean("Writing Data",true);
      SmartDashboard.putString("Data From Arduino:",serialPort.readString());
    }
    catch(Exception err){
      
    }
  }
    @Override
    public void periodic() {
    
    }
}