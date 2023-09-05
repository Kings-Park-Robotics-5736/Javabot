package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.MathUtils;

/**
 * @brief Helper utilities that are common across the various drive routines
 */
public class DriveCommandsCommon {

    /**
     * @brief Given a heading and a degree of a target off from the center, calculate how much we need to rotate, and return the velocity of the theta controller
     * 
     * @param heading  The current robot heading
     * @param degPi    The reported angle of the target (i.e. from a raspberryPi camera)
     * @param m_controller_theta  The motion controller for theta. It computes the profile.
     * 
     * @return Return the velocity, in rad/s to send to drive() (as if it came from a joystick)
     */
    public static  double calculateRotationToTarget(double heading, double degPi, ProfiledPIDController m_controller_theta) {
       
        double finalVelTheta = -100;
        if (degPi > -1000) {
          SmartDashboard.putNumber("degPi", degPi);
          SmartDashboard.putNumber("Heading", heading);
          SmartDashboard.putNumber("Dest", heading - MathUtils.degreesToRadians(degPi));
    
          final double turnOutput = m_controller_theta.calculate(heading,
              heading - MathUtils.degreesToRadians(degPi));
    
          SmartDashboard.putNumber("vel", m_controller_theta.getSetpoint().velocity);
    
          double vel_pid_theta = turnOutput * DriveConstants.kMaxSpeedMetersPerSecond;
    
          finalVelTheta = m_controller_theta.getSetpoint().velocity + vel_pid_theta;
    
          
        }
        return finalVelTheta;
      }
}
