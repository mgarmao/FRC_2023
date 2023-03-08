package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffect {
    DigitalInput sensor;

    public HallEffect(int channel){
        sensor = new DigitalInput(channel);
    }
    
    public boolean getData(){
        return !sensor.get();
    }
}