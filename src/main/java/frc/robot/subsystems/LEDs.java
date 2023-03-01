package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final SerialPort arduino = new SerialPort(9600, SerialPort.Port.kUSB1); // initialize serial port for Arduino on USB1 port

    public LEDs() {
    }    

    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {
        if (arduino.getBytesReceived() > 0) { // check if there is any data available on the serial port
            String data = arduino.readString(); // read the data from the serial port
            int value = Integer.parseInt(data.trim()); // convert the data to an integer
            System.out.println("Received value from Arduino: " + value); // print the received value to the console
        }
    }
}

