package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.PiCamera;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @brief This command centers the robot on the target. It does NOT drive to it
 *        It is useful for a user to manually hold down a button, for instance,
 *        and manually drive to an object
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls will be
 *       locked out. As such, we will manually lock out the rotate joystick.
 */
public class CenterToTargetCommand extends CommandBase {

    private DriveSubsystem m_drive;
    private boolean m_infinite;

    private final PiCamera m_picam;

    private final double PICAM_ERROR_THRESH = 2;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond, 2);

    private final ProfiledPIDController m_controller_theta = new ProfiledPIDController(1, 0, 0.000, m_constraints,
            Constants.kDt);

    public CenterToTargetCommand(DriveSubsystem robot_drive, PiCamera picam, boolean infinite) {
        this.m_drive = robot_drive;

        this.m_picam = picam;
        this.m_infinite = infinite;

        // NOTE - we explicitly do not do the below line, or else we can't drive while
        // centering
        // addRequirements(m_drive);

    }

    @Override
    public void initialize() {
        m_drive.setJoystickRotateLockout(true);

        m_controller_theta.reset(m_drive.getHeadingInRadians());

        // m_controller_theta.enableContinuousInput(-Math.PI, Math.PI);
        m_controller_theta.setTolerance(0.01);

    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
        m_drive.setJoystickRotateLockout(false);
    }

    @Override
    public void execute() {
        centerOnTarget();
    }

    @Override
    public boolean isFinished() {
        return !m_infinite && checkTurningDone();
    }

    private boolean checkTurningDone() {
        return Math.abs(m_picam.getPiCamAngle()) < PICAM_ERROR_THRESH;
    }

    private void centerOnTarget() {

        double rotationVel = DriveCommandsCommon.calculateRotationToTarget(m_drive.getHeadingInRadians(),
                m_picam.getPiCamAngle(), m_controller_theta);
        if (rotationVel > -100) {
            m_drive.drive(0, 0, rotationVel, false, false);
        }

        SmartDashboard.putBoolean("At Target 1", checkTurningDone());
        SmartDashboard.putBoolean("At Target 2", m_controller_theta.atGoal());

    }
}
