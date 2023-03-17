// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private RelativeEncoder armEncoder;
    private static final XboxController m_operator = new XboxController(Constants.CONTROLLER_OPERATOR);
    private double kP = 0.06;
    private double kI = 0.00;
    private double kD = 0.01;
    private double armCommanded = 0;
    double desiredPosition= 0;
    boolean opControl = false;
    
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Arm() {
        arm = new CANSparkMax(Constants.ARM, MotorType.kBrushless);
        armEncoder = arm.getEncoder();
        arm.restoreFactoryDefaults();

        arm.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_UPPER_LIMIT);
        arm.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_LOWER_LIMIT);
        
        arm.enableSoftLimit(SoftLimitDirection.kForward, true);
        arm.enableSoftLimit(SoftLimitDirection.kReverse, true);


        // arm.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_UPPER_LIMIT);
        // arm.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_LOWER_LIMIT);
        
        // arm.enableSoftLimit(SoftLimitDirection.kForward, true);
        // arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        arm.setIdleMode(IdleMode.kBrake);        
        armEncoder.setPosition(0);    
    }


    public void retract(){
        armCommanded = 0;
        arm.set(-Constants.ARM_POWER);
    }

    public void extend(){
        // elevatorCommanded = 160;
        arm.set(Constants.ARM_POWER); 
    }

    public void stop() {
        arm.stopMotor();
    }

    public void controller(double input){
        arm.set(input);
        Constants.ARM_IN_POSITION = false;
        opControl = true;
    }

    public void setPosition(double m_desiredPosition){
        desiredPosition = m_desiredPosition;
        opControl = false;
        if(armEncoder.getPosition()>desiredPosition){
            arm.set(Constants.ARM_POWER); 
        }
        else if(armEncoder.getPosition()<desiredPosition){
            arm.set(-Constants.ARM_POWER); 
        }
        else{
            Constants.ARM_IN_POSITION = true;
        }
    }

    @Override
    public void periodic() {
        if(m_operator.getRightY()!=0||opControl){
            controller(m_operator.getRightY());
        }

        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
        SmartDashboard.putBoolean("Arm OPCONTROL", opControl);

        
        if(m_operator.getPOV()==Constants.CONE_FRONT_PICKUP_POV){
            setPosition(Constants.CONE_FRONT_PICKUP_ARM);
        }

        if(m_operator.getPOV()==Constants.RETRACT_POV){
            setPosition(Constants.RETRACT_ARM);
        }

        if((m_operator.getPOV()==Constants.CUBE_SCORE_HIGH_POV)&&Constants.elInPosition){
            setPosition(Constants.CUBE_SCORE_HIGH_ARM);
        }

        
        
        if(!opControl){
            if(Constants.elInPosition){
                if(armEncoder.getPosition()>desiredPosition){
                    arm.set(-Constants.ARM_POWER);
                }
                if(armEncoder.getPosition()<desiredPosition){
                    arm.set(Constants.ARM_POWER);
                }
                if((armEncoder.getPosition()-desiredPosition<5)&&(armEncoder.getPosition()-desiredPosition>-5)){
                    Constants.ARM_IN_POSITION = true;
                    opControl = true;
                }
            }
            else{
                stop();
            }
        }

        // if(goingToPosition){
        //     if(armEncoder.getPosition()>desiredPosition){
        //         arm.set(Constants.ARM_POWER); 
        //     }
        //     else if(armEncoder.getPosition()<desiredPosition){
        //         arm.set(-Constants.ARM_POWER); 
        //     }
        //     else{
        //         stop();
        //         goingToPosition = false;
        //     }
        // }
        
        // SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition()); 
        
        // if(XboxController.Button.kLeftStick.value>=0){
        //     extend();
        // }
        // if(XboxController.Button.kLeftStick.value<=0){
        //     retract();
        // }
        
        // SmartDashboard.putNumber("CLimber Position", encoder.getPosition()); 
        // PID = pid.calculate(encoder.getPosition(), elevatorCommanded);
        // SmartDashboard.putNumber("Climber PID", PID);

        // elevator.set(PID); 
    }
}