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
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.Cage;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Climb m_Climb = new Climb();

  private final Command m_autoAim = new AutoAim(m_driveTrain);

  private final Joystick m_driveStick = new Joystick(OiConstants.kDriverJoystickPort);
  private final XboxController m_operatorController = new XboxController(OiConstants.kOperatorControllerPort);
  private final JoystickButton m_ExtendIntakeButton = new JoystickButton(m_operatorController, XboxConstants.kButtonX);
  private final JoystickButton m_RetractIntakeButton = new JoystickButton(m_operatorController, XboxConstants.kButtonY);
  private final JoystickButton m_ClearIntakeJamButton = new JoystickButton(m_operatorController, XboxConstants.kButtonMenu);
  private final JoystickButton m_RunCageUpwardsButton = new JoystickButton(m_operatorController, XboxConstants.kButtonRight);
  private final JoystickButton m_RunCageDownwardsButton = new JoystickButton(m_operatorController, XboxConstants.kButtonBack);
  private final JoystickButton m_TurnCageOffButton = new JoystickButton(m_operatorController, XboxConstants.kButtonLeft);
  private final JoystickButton m_RaiseFlapButton = new JoystickButton(m_operatorController, XboxConstants.kButtonA);
  private final JoystickButton m_LowerFlapButton = new JoystickButton(m_operatorController, XboxConstants.kButtonB);
  private final JoystickButton m_raiseClimbButton = new JoystickButton(m_driveStick, 8);
  private final JoystickButton m_winchClimbButton = new JoystickButton(m_driveStick, 7);

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
    new InstantCommand(m_Cage::extendIntake, m_Cage)
   );

   m_RetractIntakeButton.whenPressed(
     new InstantCommand(m_Cage::retractIntake, m_Cage)
   );

   m_ClearIntakeJamButton.whenPressed(
     new RunCommand(m_Cage::runIntakeMotorsBackwards, m_Cage)
   );

   m_RunCageUpwardsButton.whenPressed(
     new RunCommand(m_Cage::runCageUpwards, m_Cage)
   );

   m_RunCageDownwardsButton.whenPressed(
    new RunCommand(m_Cage::runCageDownwards, m_Cage)
  );

  m_TurnCageOffButton.whenPressed(
    new RunCommand(m_Cage::turnCageOff, m_Cage)
  );

  m_RaiseFlapButton.whenPressed(
    new RunCommand(m_Cage::raiseFlap, m_Cage)
  );

  m_LowerFlapButton.whenPressed(
    new RunCommand(m_Cage::lowerFlap, m_Cage)
  );

  m_raiseClimbButton.whenPressed(
    new InstantCommand(m_Climb::raiseClimb, m_Climb)
  );

  m_winchClimbButton.whenHeld(
    new RunCommand(m_Climb::winchClimb, m_Climb)
  );

  m_winchClimbButton.whenReleased(
    new RunCommand(m_Climb::stopClimb, m_Climb)
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
  public Command getAutonomousCommand() {
    return m_autoAim;
  }
}
