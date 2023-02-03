// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private CANSparkMax m_climber;
    private RelativeEncoder m_encoder;

    private double kP = 0.06;
    private double kI = 0.00;
    private double kD = 0.01;
    private double ClimberCommanded = 0;
    double PID = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public Climber() {
        m_climber = new CANSparkMax(Constants.CLIMBER, MotorType.kBrushless);
        m_encoder = m_climber.getEncoder();
        m_climber.setIdleMode(IdleMode.kBrake);
        m_climber.setSoftLimit(SoftLimitDirection.kForward, -180);
        m_climber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_climber.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_climber.enableSoftLimit(SoftLimitDirection.kReverse, false);
        m_encoder.setPosition(0);
    }

    public void climberDown(){
        ClimberCommanded = 0;
    }

    public void climberUp(){
        ClimberCommanded= 160;
    }

    public void stop() {
        m_climber.stopMotor();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("CLimber Position", m_encoder.getPosition()); 
        PID = pid.calculate(m_encoder.getPosition(), ClimberCommanded);
        SmartDashboard.putNumber("Climber PID", PID);

        m_climber.set(PID); 
    }
}