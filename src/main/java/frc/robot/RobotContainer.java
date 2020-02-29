/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.subsystems.Cage;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Cage m_Cage = new Cage();

  private final Joystick m_driveStick = new Joystick(OiConstants.kDriverJoystickPort);
  private final XboxController m_operatorController = new XboxController(OiConstants.kOperatorControllerPort);
  private final JoystickButton m_ExtendIntakeButton = new JoystickButton(m_operatorController, XboxConstants.kButtonX);
  private final JoystickButton m_RetractIntakeButton = new JoystickButton(m_operatorController, XboxConstants.kButtonA);
  private final JoystickButton m_ClearIntakeJamButton = new JoystickButton(m_operatorController, XboxConstants.kButtonY);
  private final JoystickButton m_RunCageUpwardsButton = new JoystickButton(m_operatorController, XboxConstants.kButtonRight);
  private final JoystickButton m_RunCageDownwardsButton = new JoystickButton(m_operatorController, XboxConstants.kButtonLeft);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_driveTrain.setDefaultCommand(
      new RunCommand(() -> m_driveTrain.arcadeDrive(m_driveStick.getY(), m_driveStick.getX()), m_driveTrain
        
      )
    );
  
   m_ExtendIntakeButton.whenPressed(
    new InstantCommand(m_Cage::extendIntake, m_Cage).andThen(
    new RunCommand(m_Cage::runIntakeMotorsForward, m_Cage))
   );

   m_RetractIntakeButton.whenPressed(
     new InstantCommand(m_Cage::retractIntake, m_Cage).andThen(
     new InstantCommand(m_Cage::runIntakeMotorsOff))
   );

   m_ClearIntakeJamButton.whenPressed(
     new RunCommand(m_Cage::runIntakeMotorsBackwards)
   );

   m_RunCageUpwardsButton.whenHeld(
     new RunCommand(m_Cage::RunCageUpwards)
   );

   m_RunCageDownwardsButton.whenHeld(
    new RunCommand(m_Cage::RunCageDownwards)
  );

  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
