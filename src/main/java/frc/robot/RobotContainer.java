// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Types.DirectionType;
import frc.robot.commands.JoystickCommandsFactory;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.escalator.EscalatorAssemblySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.Limelight;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final EscalatorAssemblySubsystem m_escalatorAssembly = new EscalatorAssemblySubsystem();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public Limelight m_limelight;

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    XboxController m_actionController = new XboxController(OIConstants.kActionControllerPort);

    private void driveWithJoystick(Boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
                * DriveConstants.kMaxSpeedMetersPerSecond;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
                * DriveConstants.kMaxSpeedMetersPerSecond;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.02))
                * DriveConstants.kMaxSpeedMetersPerSecond;

        m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, true);
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Create Limelight instance
        m_limelight = new Limelight();

        // Set limelight LED to OFF on startup
        m_limelight.setLEDOff();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> driveWithJoystick(true),
                        m_robotDrive));
    }


    /**
     * Command to toggle limelight LED ON/OFF
     * @return Command
     */
    private Command toggleLimelightLED() {
        return Commands.runOnce(() -> m_limelight.toggleLED());
    }

    /**
     * Command to detect AprilTag value and print to smart dashboard
     * @return Command
     */
    private void printAprilTag() {
        Double offsetX = m_limelight.getTargetOffsetX();
        SmartDashboard.putNumber("LimelightX", offsetX);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        new JoystickButton(m_driverController, XboxController.Button.kA.value)
                .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive));

        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(m_robotDrive.driveXMetersPID(1));


        /* temporarily removed to test limelight functionality
        new JoystickButton(m_actionController, XboxController.Button.kB.value)
                .toggleOnTrue(m_intake.RunIntakeForwardCommand());

        new JoystickButton(m_actionController, XboxController.Button.kX.value)
                .toggleOnTrue(m_intake.RunIntakeBackwardCommand());
        */

        // toggle LED ON/OFF
        new JoystickButton(m_actionController, XboxController.Button.kB.value)
                .toggleOnTrue(this.toggleLimelightLED());
        // print detected AprilTag value to smart dashboard
        new JoystickButton(m_actionController, XboxController.Button.kX.value)
                .toggleOnTrue(Commands.runOnce(() -> this.printAprilTag()));


       
        
        new JoystickButton(m_actionController, XboxController.Button.kBack.value)
        .onTrue(m_escalatorAssembly.ResetElevatorEncoderCommand());

        new JoystickButton(m_actionController, XboxController.Button.kStart.value)
        .onTrue(m_escalatorAssembly.ResetEscalatorEncoderCommand());



        new JoystickButton(m_actionController, XboxController.Button.kY.value)
                .whileTrue(m_escalatorAssembly.RunElevatorToPositionCommand(0).andThen(JoystickCommandsFactory.RumbleControllerTillCancel(m_actionController)));

        new JoystickButton(m_actionController, XboxController.Button.kA.value)
                .whileTrue(m_escalatorAssembly.RunElevatorToPositionCommand(59).andThen(JoystickCommandsFactory.RumbleControllerTillCancel(m_actionController)));



        new JoystickButton(m_actionController, XboxController.Button.kLeftBumper.value)
                .whileTrue(m_escalatorAssembly.RunEscalatorToPositionCommand(0).andThen(JoystickCommandsFactory.RumbleControllerTillCancel(m_actionController)));

                new JoystickButton(m_actionController, XboxController.Button.kRightBumper.value)
                .whileTrue(m_escalatorAssembly.RunEscalatorToHighScore().andThen(JoystickCommandsFactory.RumbleControllerTillCancel(m_actionController)));

        new JoystickButton(m_actionController, XboxController.Button.kLeftStick.value)
                .whileTrue(m_escalatorAssembly.RunEscalatorToMidScore().andThen(JoystickCommandsFactory.RumbleControllerTillCancel(m_actionController)));

        

        //this is not a button. So we need to manually define the trigger
        //All a joystick button is is a wrapper for a trigger, that generates a condition.
        //I.E. new JoystickButton(m_actionController, XboxController.Button.kRightBumper.value) generates something like:
        // ()->{return  XboxController.Button.kRightBumper.value==1;}
        //Here, we are creating our own condition:
        // ()->{return m_actionController.getRightTriggerAxis() > 0;}, which means that we are 'triggered' once we have some value on the axis.
        new Trigger(() -> {
                return m_actionController.getRightTriggerAxis() > 0;
                }).whileTrue(m_escalatorAssembly.RunEscalatorManualSpeedCommand(() -> m_actionController.getRightTriggerAxis()));
        new Trigger(() -> {
                return m_actionController.getLeftTriggerAxis() > 0;
                }).whileTrue(m_escalatorAssembly.RunEscalatorManualSpeedCommand(() -> -m_actionController.getLeftTriggerAxis()));


        //using .02 here as a deadband
        new Trigger(() -> {
                return m_actionController.getRightY() > 0.02 || m_actionController.getRightY() < -0.02 ;
        }).and((new JoystickButton(m_actionController, XboxController.Button.kBack.value)).negate()).whileTrue(m_escalatorAssembly.RunElevatorManualSpeedCommand(() -> m_actionController.getRightY(), false));

       
        new Trigger(() -> {
                return m_actionController.getRightY() > 0.02 || m_actionController.getRightY() < -0.02 ;
        }).and(new JoystickButton(m_actionController, XboxController.Button.kBack.value)).whileTrue(m_escalatorAssembly.RunElevatorManualSpeedCommand(() -> m_actionController.getRightY(), true));

       
        //the POV hat works differently;
        //it returns a value from 0-360, where 0 is up, 90 is right, 180 is down, and 270 is left.
        //you can get values in between, so we need to define a range.
        new Trigger(() -> {
                return m_actionController.getPOV() > 20 && m_actionController.getPOV()  < 180;
                }).onTrue(m_escalatorAssembly.RunFlipperToPositionCommand(DirectionType.UP));

        new Trigger(() -> {
                return m_actionController.getPOV() <180 && m_actionController.getPOV()  < 360;
                }).onTrue(m_escalatorAssembly.RunFlipperToPositionCommand(DirectionType.DOWN));

        
        /* 
        new Trigger(() -> {
                return m_actionController.getRightTriggerAxis() > 0;
        }).whileTrue(m_escalatorAssembly.RunFlipperManualSpeedCommand(() -> m_actionController.getRightTriggerAxis()));
        new Trigger(() -> {
        return m_actionController.getLeftTriggerAxis() > 0;
        }).whileTrue(m_escalatorAssembly.RunFlipperManualSpeedCommand(() -> -m_actionController.getLeftTriggerAxis()));
        */

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     public Command getAutonomousCommand(){
        return TrajectoryCommandsFactory.generateAutoTrajectoryCommand(m_robotDrive, m_intake, m_escalatorAssembly);
        /*
        return TrajectoryCommandsFactory.generateTrajectoryCommand(m_robotDrive, 
        new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
        new Pose2d(new Translation2d(2,1), new Rotation2d(1.57)),
        true);
        */

     }

}
