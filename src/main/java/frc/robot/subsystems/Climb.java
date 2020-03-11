/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ValueConstants;

public class Climb extends SubsystemBase {

  private DoubleSolenoid m_ClimbSolenoid = new DoubleSolenoid(PneumaticsConstants.kClimbSolenoidPort1, PneumaticsConstants.kClimbSolenoidPort2);

  private Spark m_ClimbMotor1 = new Spark(MotorConstants.kClimbMotor1Port);
  private Spark m_ClimbMotor2 = new Spark(MotorConstants.kClimbMotor2Port);

  private SpeedControllerGroup m_ClimbMotors = new SpeedControllerGroup(m_ClimbMotor1, m_ClimbMotor2);

  public void raiseClimb() {
    m_ClimbSolenoid.set(Value.kForward);
  }

  public void winchClimb() {
    m_ClimbSolenoid.set(Value.kReverse);
    m_ClimbMotors.set(ValueConstants.kClimbSpeedScalar);
  }

  public void stopClimb() {
    m_ClimbMotors.set(0);
  }
  /**
   * Creates a new Climb.
   */
  public Climb() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
