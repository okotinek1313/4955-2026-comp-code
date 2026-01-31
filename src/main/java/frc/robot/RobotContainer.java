// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.RobotMap.OperatorConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      
    public static Drive drivebase = new Drive();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  



    configureBindings();
    Driving();
  }

  private void configureBindings() {
  
  }

  public static void Driving() {
    m_driverController.leftTrigger().onTrue(Commands.run(()-> drivebase.runDD()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
  }
}
