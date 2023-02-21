// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final TalonSRX leftDriveTalon = new TalonSRX(Constants.OperatorConstants.leftDrivePort);
  private final TalonSRX rightDriveTalon = new TalonSRX(Constants.OperatorConstants.rightDrivePort);
  public DriveTrain() {
    leftDriveTalon.setInverted(true);
  }
  public void tankDrive(double lPower, double rPower){
    leftDriveTalon.set(ControlMode.PercentOutput, lPower);
    rightDriveTalon.set(ControlMode.PercentOutput, rPower);
  }
  @Override
  public void periodic() {
    tankDrive(RobotContainer.getJoy1().getY(), RobotContainer.getJoy1().getThrottle());
    //using logitech controller to move robot :D
    
  }
}
