package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Configuration.Constants;

public class Limelight {
    
    private NetworkTableEntry _validTargets;
    private NetworkTableEntry _camMode;
    private NetworkTableEntry _ledMode;

    public static Limelight _instance;
    private double _limelightHeight = 2.91; // Feet
    private double _goalHeight = 8.66; // Feet
    private double _measurementHeight = _goalHeight - _limelightHeight; // Feet
    private double _theta = 30; // Degrees

    public static Limelight getInstance()
    {
        if (_instance == null)
        {
            _instance = new Limelight();
        }

        return _instance;
    }

    public Limelight()
    {
        _validTargets = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kValidTargetKey);
        _ledMode = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kLEDModeKey);
        _camMode = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kCamModeKey);
    }

    public double getAngleToTarget()
    {
        return NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetAngleXKey).getDouble(0);
    }

    public double getDistanceToTarget()
    {
        if (hasTarget())
        {
            return 0;
        }
        
        var offsetX = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetAngleXKey).getDouble(0);
        var offsetY = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetAngleYKey).getDouble(0);
        
        var distance = _measurementHeight/(Math.tan(Math.toRadians(offsetY + _theta))*Math.cos(Math.toRadians(offsetX)));

        return distance;
    }

    public boolean hasTarget()
    {
        return _validTargets.getDouble(0) >= 1;
    }

    public boolean isAimbot()
    {
        return _camMode.getDouble(0) == 1;
    }

    public void setAimbot()
    {
        _ledMode.setDouble(Constants.kLEDOn);
        _camMode.setDouble(Constants.kCamModeVisionProcessing);
    }

    public void setDashcam()
    {
        _ledMode.setDouble(Constants.kLEDOff);
        _camMode.setDouble(Constants.kCamModeDriver);
    }
}
