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
    double desiredPosition= 0;
    boolean opControl = false;
    
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Arm() {
        arm = new CANSparkMax(Constants.ARM, MotorType.kBrushless);
        armEncoder = arm.getEncoder();
        arm.restoreFactoryDefaults();

        arm.setSmartCurrentLimit(50);
        arm.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_UPPER_LIMIT);
        arm.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_LOWER_LIMIT);
        
        arm.enableSoftLimit(SoftLimitDirection.kForward, true);
        arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        
        arm.setIdleMode(IdleMode.kBrake);        
        armEncoder.setPosition(0);    
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

    public double armDistance(){
        double elDistanceUp = (Constants.elPosition/Constants.EL_GEAR_RATIO)*Constants.EL_GEAR_CIRCUMFRANCE;
        double bumperToEl = elDistanceUp*Math.sin(33.5);
        // 460/42 = how many degrees per tick; *ticks -90 to get to horizontal = degrees 
        double armPivotToEndOfArm = Constants.ARM_LENGTH*Math.sin((460/42)*armEncoder.getPosition()-90);
        return bumperToEl+armPivotToEndOfArm;
    }

    @Override
    public void periodic() {
        if(m_operator.getRightY()!=0||opControl){
            controller(m_operator.getRightY());
        }

        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
        SmartDashboard.putBoolean("Arm OPCONTROL", opControl);

        //Setpoints
        if(m_operator.getPOV()==Constants.CONE_FRONT_PICKUP_POV){
            setPosition(Constants.CONE_FRONT_PICKUP_ARM);
        }

        if(m_operator.getPOV()==Constants.RETRACT_POV){
            setPosition(Constants.RETRACT_ARM);
        }

        if((m_operator.getPOV()==Constants.CUBE_SCORE_HIGH_POV)&&(Constants.elPosition<=Constants.MIN_EL_EXTENTION_FOR_ARM)){
            setPosition(Constants.CUBE_SCORE_HIGH_ARM);
        }
        
        if((m_operator.getPOV()==Constants.CONE_SCORE_MID_POV)&&(Constants.elPosition<=Constants.MIN_EL_EXTENTION_FOR_ARM)){
            setPosition(Constants.CONE_SCORE_MID_ARM);
        }

        //Not Hitting El prevention
        // if((Constants.elPosition>=Constants.MIN_EL_EXTENTION_FOR_ARM)&&((armEncoder.getPosition()>=Constants.LOWER_ARM_DEADZONE)&&(armEncoder.getPosition()<=Constants.UPPER_ARM_DEADZONE))&&opControl){
        //     arm.set(0);
        // }
        
        
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
    }
}