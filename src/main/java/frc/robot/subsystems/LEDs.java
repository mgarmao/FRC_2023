package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  private SerialPort serial;
  private String buffer;

  public LEDs() {
    serial = new SerialPort(9600, SerialPort.Port.kUSB);
    buffer = "";
  }

  public void sendData(String data) {
    serial.writeString(data + "\n");
  }

  public String receiveData() {
    String receivedData = buffer + serial.readString();
    String[] messages = receivedData.split("\n");
    if (messages.length > 1) {
      buffer = messages[messages.length - 1];
      return messages[0];
    } else {
      buffer = receivedData;
      return null;
    }
  }
  
  @Override
  public void periodic() {
    receiveData();
  }
}