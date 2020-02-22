/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class DriveTrain extends SubsystemBase {

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(
      new PWMVictorSPX(MotorConstants.kLeftMotor1Port), new PWMVictorSPX(MotorConstants.kLeftMotor2Port));

  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(new PWMVictorSPX(MotorConstants.kRightMotor1Port), new PWMVictorSPX(MotorConstants.kRightMotor2Port));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  private final Encoder m_leftEncoder = new Encoder(0,1);
  private final Encoder m_rightEncoder = new Encoder(2,3);

  

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
