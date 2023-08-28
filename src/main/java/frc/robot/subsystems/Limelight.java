package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * Limelight camera class
 */
public class Limelight {
    private static NetworkTable table;
    private static NetworkTableEntry tx;
    private static NetworkTableEntry ty;
    private static NetworkTableEntry tv;
    private static NetworkTableEntry ta;
    private static NetworkTableEntry camMode;
    private static NetworkTableEntry ledMode;

    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3)
    }

    public enum CamMode {
        VISION(0),
        DRIVER(1)
    }

    /**
     * Camera sends data to network table, get table and values when creating instance of Limelight
     */
    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx"); // horizontal offset (-29.8 - 29.8 degrees)
        ty = table.getEntry("ty"); // vertical offset (-24.85 - 24.85 degrees)
        tv = table.getEntry("tv"); // valid target (0 - 1)
        ta = table.getEntry("ta"); // target area (0% - 100% of image)
        ledMode = table.getEntry("ledMode"); // LED state (0-3)
        camMode = table.getEntry("camMode"); // operation mode (0-1)
    }

    /**
     * Get current LED mode
     * @return LEDMode(0-4)
     */
    public getLEDMode() {
        return ledMode;
    }

    /**
     * Set LED mode
     * @param mode LEDMode(0-4)
     */
    public setLEDMode(LEDMode mode) {
        ledMode.setNumber(mode);
    }

    /**
     * Set LED mode to ON
     */
    public setLEDOn() {
        ledMode.setNumber(LEDMode.ON);
    }

    /**
     * Set LED mode to OFF
     */
    public setLEDOff() {
        ledMode.setNumber(LEDMODE.OFF);
    }

    /**
     * Set LED mode to BLINK
     */
    public setLEDBlink() {
        ledMode.setNumber(LEDMODE.BLINK);
    }

    /**
     * Get current camera mode
     * @return CamMode(0-1)
     */
    public getCamMode() {
        return camMode;
    }

    /**
     * Set camera mode
     * @param mode CamMode(0-1)
     */
    public setCamMode(CamMode mode) {
        camMode.setNumber(mode);
    }

    /**
     * Preset operation mode - LED set to OFF, camera set to DRIVER
     */
    public setModeDriver() {
        this.setLEDMode(LEDMode.OFF);
        this.setCamMode(CamMode.DRIVER);
    }

    /**
     * Preset operation mode - LED set to ON, camera set to VISION
     */
    public setModeVision() {
        this.setLEDMode(LEDMode.ON);
        this.setCamMode(CamMode.VISION);
    }

    /**
     * Get horizontal offset to target
     * @return -29.8 - 29.8 degrees
     */
    public getTargetOffsetX() {
        return tx.getDouble(0.0);
    }

    /**
     * Get vertical offset to target
     * @return -24.85 - 24.85 degrees
     */
    public getTargetOffsetY() {
        return ty.getDouble(0.0);
    }

    /**
     * Check for a detected target
     * @return boolean - true if target is found else false
     */
    public checkValidTarget() {
        if (tv.getNumber(0).intValue() == 1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Get target area
     * @return 0% - 100% of  image
     */
    public getTargetArea() {
        return ta.getDouble(0.0);
    }

}
