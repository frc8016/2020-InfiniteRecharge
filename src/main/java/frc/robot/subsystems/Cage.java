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

public class Cage extends SubsystemBase {

  private Spark m_UpperMotor1 = new Spark(MotorConstants.kCageUpperMotor1Port);
  private Spark m_UpperMotor2 = new Spark(MotorConstants.kCageUpperMotor2Port);
  private Spark m_LowerMotor1 = new Spark(MotorConstants.kCageLowerMotor1Port);
  private Spark m_LowerMotor2 = new Spark(MotorConstants.kCageLowerMotor2Port);

  private SpeedControllerGroup m_UpperMotors = new SpeedControllerGroup(m_UpperMotor1, m_UpperMotor2);
  private SpeedControllerGroup m_LowerMotors = new SpeedControllerGroup(m_LowerMotor1, m_LowerMotor2);

  private SpeedControllerGroup m_CageMotors = new SpeedControllerGroup(m_UpperMotors, m_LowerMotors);

  
  private final Spark m_IntakeMotor = new Spark(MotorConstants.kIntakeMotorPort);

  private final DoubleSolenoid m_IntakeSolenoid = new DoubleSolenoid(PneumaticsConstants.kIntakeSolenoidPort1,
      PneumaticsConstants.kIntakeSolenoidPort2);

//Call these methods to set the motor direction for the intake
  public void runIntakeMotorsForward() {
    m_IntakeMotor.set(ValueConstants.kIntakeMotorSpeedScalar);
  }
  public void runIntakeMotorsBackwards() {
    m_IntakeMotor.set(-ValueConstants.kIntakeMotorSpeedScalar);
  }
  public void runIntakeMotorsOff() {
    m_IntakeMotor.set(0);
  }


//Use this to retract or extend the intake.
  public void retractIntake() {
    m_IntakeSolenoid.set(Value.kReverse);
  }
  public void extendIntake() {
    m_IntakeSolenoid.set(Value.kForward);
  }



  public void RunCageUpwards() {
    m_CageMotors.set(ValueConstants.kCageSpeedScalar);
  }

  public void RunCageDownwards() {
    m_CageMotors.set(-ValueConstants.kCageSpeedScalar);
  }

  /**
   * Creates a new Cage.
   */
  public Cage() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
