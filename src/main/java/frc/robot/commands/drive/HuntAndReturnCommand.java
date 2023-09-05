package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.PiCamera;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.PathPlanFromDynamicStartCommand;

/**
 * @brief This is a complex class that combines multiple actions into 1 command.
 *        It is complex, because we need to require state from an earlier
 *        command to use in a later state. Built in command sequences do not
 *        support this.
 * 
 * @note This is implemented as a state machine, starting with HUNT for target,
 *       goint to RETURN TO START.
 * 
 * @note If escalatorCommnad is not provided, we will treat this as a teleop,
 *       where the user will seperately control intake and escalator.
 * 
 * @note This will ensure intake starts when we begin hunting, turns off when we
 *       finish, and once it is off, we will raise the elevator up. We ensure
 *       the elevator is all the way up before returning.
 */
public class HuntAndReturnCommand extends CommandBase {

    enum State {
        IDLE,
        HUNT,
        RETURN_TO_START
    }

    private State m_state;
    private PathPlanFromDynamicStartCommand m_PathPlanFromDynamicStartCommand;
    private DriveToTargetCommand m_driveToTargetCommand;
    private Command m_runElevatorCommand;
    private Command m_runIntakeCommand;
    private Pose2d m_startPose;
    private final DriveSubsystem m_robotDrive;

    private final PiCamera m_picam;
    private final double m_speed;
    private final double m_maxDistance;
    private final boolean m_usePid;
    private int elevatorDelay;

    public HuntAndReturnCommand(DriveSubsystem robotDrive, PiCamera picam, double speed, double maxDistance) {
        this(robotDrive, null, null, picam, speed, maxDistance, true);
    }

    public HuntAndReturnCommand(DriveSubsystem robotDrive, Command escalatorCommand, Command intakeCommand,
            PiCamera picam, double speed, double maxDistance, boolean usePid) {
        m_state = State.IDLE;
        m_robotDrive = robotDrive;
        m_picam = picam;
        m_speed = speed;
        m_usePid = usePid;
        m_maxDistance = maxDistance;
        m_runIntakeCommand = intakeCommand;
        m_runElevatorCommand = escalatorCommand;
        addRequirements(robotDrive);
    }

    @Override
    public void initialize() {

        m_startPose = m_robotDrive.getPose(); // save our pose before we start hunting

        // create our sub-commands
        m_driveToTargetCommand = new DriveToTargetCommand(m_robotDrive, m_picam, m_speed, m_maxDistance);
        m_PathPlanFromDynamicStartCommand = new PathPlanFromDynamicStartCommand(m_robotDrive::getPose, m_robotDrive,
                m_startPose, m_usePid);

        // initialize our first command, the hunt/drive to target
        m_driveToTargetCommand.initialize();

        // if we are in auto with elevator and intake, initialize those now too.
        if (m_runElevatorCommand != null) {
            m_runElevatorCommand.initialize();
            elevatorDelay = 0;
        }
        if (m_runIntakeCommand != null) {
            m_runIntakeCommand.initialize();
        }
        m_state = State.HUNT; // We started, so we are now hunting!
    }

    @Override
    public void end(boolean interrupted) {
        switch (m_state) {
            case HUNT:
                m_driveToTargetCommand.end(interrupted); // ensure we stop/end the drive command if it hasnt finished
                                                         // yet
                break;
            case RETURN_TO_START:
                m_PathPlanFromDynamicStartCommand.end(interrupted); // ensure we stop path planning
                break;
            default:
                break;
        }
        System.out.println("-------------Done Hunt-----------------");

        // if we are not in auto (like button press during teleop), then we need to stop
        // the robot.
        if (m_runElevatorCommand == null) {
            // need to do this if we expect to stop after finishing.
            m_robotDrive.drive(0, 0, 0, false);
        }
        m_state = State.IDLE;
    }

    @Override
    public void execute() {
        switch (m_state) {
            case HUNT:
                // keep running the intake, and keep driving to target.
                m_driveToTargetCommand.execute(); // Happy Hunting!
                m_runIntakeCommand.execute();
                break;
            case RETURN_TO_START:

                // use path planning lib to keep returing to start
                m_PathPlanFromDynamicStartCommand.execute();

                // we dont want to lift elevator right away (maybe cone is still coming in)
                // so delay it 20 counts, which is 20 *.02s = .4 seconds
                if (m_runElevatorCommand != null) {
                    elevatorDelay++;
                    if (elevatorDelay > 20) {
                        m_runElevatorCommand.execute();
                    }
                }

                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {

        boolean ret = false;
        switch (m_state) {
            case HUNT:
                if (m_driveToTargetCommand.isFinished()) {
                    m_state = State.RETURN_TO_START;
                    m_driveToTargetCommand.end(false); // stop the hunt before moving on

                    // now that we have starting position (got in this initialize), and now that we
                    // reached our final hunt position, we can go ahead and create our trajectory (in initialize)
                    m_PathPlanFromDynamicStartCommand.initialize();
                    m_runIntakeCommand.end(true); // stop the intake before raising the elevator
                }
                ret = false;
                break;
            case RETURN_TO_START:
                ret = m_PathPlanFromDynamicStartCommand.isFinished()
                        && (m_runElevatorCommand == null || m_runElevatorCommand.isFinished());
                break;
            default:
                ret = false;
                break;
        }
        return ret;
    }
}
