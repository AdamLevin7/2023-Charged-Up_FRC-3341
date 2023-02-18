// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Pinch;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  TalonSRX clawTalon = new TalonSRX(Constants.OperatorConstants.clawPort);
  TalonSRX wristTalon = new TalonSRX(Constants.OperatorConstants.wristPort);
  TalonSRX flyOneTalon = new TalonSRX(Constants.OperatorConstants.flyWheelOnePort);
  TalonSRX flyTwoTalon = new TalonSRX(Constants.OperatorConstants.flyWheelTwoPort);
  //Talons are placeholders until we know the final claw design

  Servo leftServo = new Servo(Constants.OperatorConstants.leftServoPort);

  boolean clawed = false;
  //Clay either has 3 talons or 1
  public Claw() {
    clawTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    wristTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }
  public double getClawPos(){
    return clawTalon.getSelectedSensorPosition();
  }
  public double getWristAngle(){
    return wristTalon.getSelectedSensorPosition();
  }
  public void pinchManual(double power){
    clawTalon.set(ControlMode.PercentOutput, power);
  }
  public void pinchButton(boolean clawed){

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //pinchManual(RobotContainer.getJoy().getY());
    if(RobotContainer.getJoy1().getRawButtonPressed(2)){
      leftServo.set(0);
    }
    if(RobotContainer.getJoy1().getRawButtonPressed(3)){
      leftServo.set(0.2);
    }
    if(RobotContainer.getJoy1().getRawButtonPressed(4)){
      leftServo.set(0.4);
    }
    if(RobotContainer.getJoy1().getRawButtonPressed(5)){
      leftServo.set(0.6);
    }
    if(RobotContainer.getJoy1().getRawButtonPressed(6)){
      leftServo.set(0.8);
    }
    if(RobotContainer.getJoy1().getRawButtonPressed(7)){
      leftServo.set(1);
    }
    if(RobotContainer.getJoy1().getTriggerPressed()){
      clawed = !clawed;
    }
    pinchButton(clawed);
    SmartDashboard.putNumber("Voltage", clawTalon.getMotorOutputVoltage());
    SmartDashboard.putNumber("Servo Position", leftServo.getPosition());
  }
}
