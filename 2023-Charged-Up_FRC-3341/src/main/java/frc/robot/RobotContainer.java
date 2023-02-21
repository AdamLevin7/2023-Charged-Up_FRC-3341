package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Extend;
import frc.robot.commands.Rotate;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Arm arm = new Arm();
  public final Claw claw = new Claw();
  public final DriveTrain dt = new DriveTrain();

  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    JoystickButton triggerStowPos = new JoystickButton(leftJoystick, 5);
    triggerStowPos.onTrue(new Rotate(arm, 0));

    JoystickButton triggerMiddlePos = new JoystickButton(leftJoystick, 4);
    triggerMiddlePos.onTrue(new Rotate(arm, 10));

    JoystickButton triggerOtherPos = new JoystickButton(leftJoystick, 3);
    triggerOtherPos.onTrue(new Rotate(arm, 30));

    JoystickButton triggerGroundPos = new JoystickButton(leftJoystick, 6);
    triggerGroundPos.onTrue(new Rotate(arm, 90));
    //JoystickButton triggerMiddleExt = new JoystickButton(leftJoystick, 8);
    //triggerMiddleExt.onTrue(new Extend(arm, 10));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public static Joystick getJoy1() {
    return leftJoystick;
  }
  public static Joystick getJoy2() {
    return rightJoystick;
  }
}