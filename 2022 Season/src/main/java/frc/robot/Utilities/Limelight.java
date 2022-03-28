package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Configuration.Constants;

public class Limelight {
    
    private NetworkTableEntry _validTargets;
    private NetworkTableEntry _camMode;
    private NetworkTableEntry _ledMode;

    public static Limelight _instance;
    private double _limelightHeight = 3; // Feet
    private double _goalHeight = 8.58; // Feet
    private double _measurementHeight = _goalHeight - _limelightHeight; // Feet
    private double _theta = 35; // Degrees

    /**
     * Returns the limelight instance
     * @return - Instance of limelight
     */
    public static Limelight getInstance()
    {
        if (_instance == null)
        {
            _instance = new Limelight();
        }

        return _instance;
    }

    /**
     * Limelight Constructor to create a LimeLight Object
     */
    public Limelight()
    {
        _validTargets = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kValidTargetKey);
        _ledMode = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kLEDModeKey);
        _camMode = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kCamModeKey);
    }

    /**
     * Returns the limelight value from {@link NetworkTableInstance}
     * @return - Angle value to target
     */
    public double getAngleToTarget()
    {
        return NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetAngleXKey).getDouble(0);
    }

    /**
     * Gets distance from target, if there is no target provided it will return 0.0
     * @return - Distance from the target
     */
    public double getDistanceToTarget()
    {
        if (!hasTarget())
        {
            return 0;
        }

        var offsetX = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetAngleXKey).getDouble(0);
        var offsetY = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetAngleYKey).getDouble(0);
        
        var distance = _measurementHeight/(Math.tan(Math.toRadians(offsetY + _theta))*Math.cos(Math.toRadians(offsetX)));

        return distance;
    }

    /**
     * Returns the limelight visionTargetState
     * @return - The current VisionTargetState
     */
    public VisionTargetState getVisionTargetState()
    {
        if (!hasTarget())
        {
            return null;
        }
        
        var offset = getAngleToTarget();
        var distance = getDistanceToTarget();

        return new VisionTargetState(offset, distance);
    }

    /**
     * Checks if it has a target
     * @return - Whether or not there is a target
     */
    public boolean hasTarget()
    {
        return _validTargets.getDouble(0) >= 1;
    }

    /**
     * Checks if the bot is aimed
     * @return - Whether or not the bot is aimed
     */
    public boolean isAimbot()
    {
        return _camMode.getDouble(0) == 1;
    }

    /**
     * Sets the aim of the bot
     */
    public void setAimbot()
    {
        _ledMode.setDouble(Constants.kLEDOn);
        _camMode.setDouble(Constants.kCamModeVisionProcessing);
    }

    /** 
     * Sets the DashCame 
     */
    public void setDashcam()
    {
        _ledMode.setDouble(Constants.kLEDOff);
        _camMode.setDouble(Constants.kCamModeDriver);
    }
}
