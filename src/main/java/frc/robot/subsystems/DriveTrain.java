/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(new WPI_VictorSPX(DriveConstants.leftMotor1Port), new WPI_VictorSPX(DriveConstants.leftMotor2Port));

  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new WPI_VictorSPX(DriveConstants.rightMotor1Port), new WPI_VictorSPX(DriveConstants.rightMotor2Port));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }


  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
