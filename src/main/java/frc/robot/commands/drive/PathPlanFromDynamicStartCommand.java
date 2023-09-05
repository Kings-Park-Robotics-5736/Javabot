package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.commands.TrajectoryCommandsFactory;

/**
 * @brief this is a wrapper around the path planning lib to allow us to go to a
 *        target based on our current starting position
 * 
 * @note Usual pplib requires giving the trajectory on command creation ('new'
 *       operator). This wrapper allows us to delay that until we call
 *       initialize().
 */
public class PathPlanFromDynamicStartCommand extends CommandBase {

    private PPSwerveControllerCommand m_ppscc;
    private final Supplier<Pose2d> m_initialPoseSupplier;
    private final DriveSubsystem m_robotDrive;
    private Pose2d m_endPose;
    private final boolean m_usePid;

    /**
     * @param initialPoseSupplier method to obtain the current robot pose to determine initial state.
     * @param robotDrive
     * @param endPose
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose) {
        this(initialPoseSupplier, robotDrive, endPose, true);

    }

    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose, boolean usePid) {
        m_initialPoseSupplier = initialPoseSupplier;
        m_robotDrive = robotDrive;
        m_endPose = endPose;
        m_usePid = usePid;
        addRequirements(robotDrive); // this effectively locks out the joystick controlls.
        // without this, the 0,0 from the joystick still happens, causing jittery
        // movement
    }

    public void SetEndPose(Pose2d endPose) {
        m_endPose = endPose;
    }

    @Override
    public void initialize() {

        //delay trajectory creation until this initialize.
        var traj = TrajectoryCommandsFactory.generateTrajectory(m_initialPoseSupplier.get(), m_endPose);
        m_ppscc = TrajectoryCommandsFactory.generatePPTrajectoryCommand(m_robotDrive, traj, m_usePid);
        m_ppscc.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        m_ppscc.end(interrupted);
    }

    @Override
    public void execute() {
        m_ppscc.execute();
    }

    @Override
    public boolean isFinished() {
        return m_ppscc.isFinished();
    }

}
