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

  //Create Cage motors
  private Spark m_UpperMotor = new Spark(MotorConstants.kCageUpperMotorPort);
  private Spark m_LowerMotor = new Spark(MotorConstants.kCageLowerMotorPort);

  private SpeedControllerGroup m_CageMotors = new SpeedControllerGroup(m_UpperMotor, m_LowerMotor);

  //Create Intake Motors and Solenoids
  private final Spark m_IntakeMotor = new Spark(MotorConstants.kIntakeMotorPort);

  private final DoubleSolenoid m_IntakeSolenoid = new DoubleSolenoid(PneumaticsConstants.kIntakeSolenoidPort1,
      PneumaticsConstants.kIntakeSolenoidPort2);

  //Create Flap Solenoids
  private final DoubleSolenoid m_FlapSolenoid = new DoubleSolenoid(PneumaticsConstants.kFlapSolenoidPort1, PneumaticsConstants.kFlapSolenoidPort2);
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
    m_IntakeMotor.set(0);
  }
  public void extendIntake() {
    m_IntakeSolenoid.set(Value.kForward);
    m_IntakeMotor.set(ValueConstants.kIntakeMotorSpeedScalar);
  }



  public void runCageUpwards() {
    m_CageMotors.set(ValueConstants.kCageSpeedScalar);
  }

  public void runCageDownwards() {
    m_CageMotors.set(-ValueConstants.kCageSpeedScalar);
  }

  public void turnCageOff() {
    m_CageMotors.set(0.0);
  }


  public void raiseFlap() {
    m_FlapSolenoid.set(Value.kForward);
  }

  public void lowerFlap() {
    m_FlapSolenoid.set(Value.kReverse);
  }

  /**
   * Creates a new Cage.
   */
  public Cage() {
    retractIntake();
    turnCageOff();
    lowerFlap();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
