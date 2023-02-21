// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
    private CANSparkMax motor;

    public IntakeWheels() {
        /** Create a new object to control the SPARK MAX motor controllers. */
        motor = new CANSparkMax(Constants.INTAKE_WHEELS, MotorType.kBrushless);
        /**
         * Restore motor controller parameters to factory default until the next controller 
         * reboot.
         */
        motor.restoreFactoryDefaults();

        /**
         * When the SPARK MAX is receiving a neutral command, the idle behavior of the motor 
         * will effectively disconnect all motor wires. This allows the motor to spin down at 
         * its own rate. 
         */
        motor.setIdleMode(IdleMode.kCoast);


    }

    /** Retrieve cargo for transportation. */
    public void retrieve() {
        motor.set(Constants.INTAKE_SPEED);
    }

    /** Eject cargo from the robot. */
    public void eject() {
        motor.set(-Constants.INTAKE_SPEED);
    }
    
    /** This function is called once each time the the command ends or is interrupted. */
    public void stop() {
        motor.stopMotor();
    }

    /** This method will be called once per scheduler run. */
    @Override
    public void periodic() {}

}