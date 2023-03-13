// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class Shortcuts extends SubsystemBase {
    private static final XboxController m_operator = new XboxController(Constants.CONTROLLER_OPERATOR);
    public static final Arm arm = new Arm();
    public static final Wrist wrist = new Wrist();

    int wristDown = 10;
    int armDown = 10;

    public Shortcuts() {
          
    }


    public void foward(){
        // arm.setPosition(24.8);
        // wrist.setPosition(18.8);
    }

    public void back(){

    }

    public void retract(){

    }

    public boolean getDPadUp() {
        int dPadValue = m_operator.getPOV();
        var direction = 0;
        return (dPadValue == direction) || (dPadValue == (direction + 45) % 360)|| (dPadValue == (direction + 315) % 360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("DPAD UP", getDPadUp());
        if(getDPadUp()){
            foward();
        }
    }
}