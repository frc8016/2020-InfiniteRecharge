/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ValueConstants;

public class Intake extends SubsystemBase {

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

  /**
   * Creates a new Intake.
   */
  public Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
