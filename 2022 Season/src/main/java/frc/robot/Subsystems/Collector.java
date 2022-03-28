package frc.robot.Subsystems;

import java.nio.channels.Channel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Lib.Photoeye;

public class Collector implements ISubsystem {

private static Collector _instance;   
    
    private CANSparkMax _collectorMotor;
    private CANSparkMax _frontChamberMotor;
    private CANSparkMax _backChamberMotor;
    private Solenoid _collectorSolenoid;
    private Photoeye _chamberBottomSensor;
    private Photoeye _chamberTopSensor;

    private double _collectorSpeed = 0.75;
    private double _chamberSpeed = 1.0;
    private int _ballCount = 0;
    private boolean _bottomChamberState = false;
    private boolean _topChamberState = false;

    /**
     * Get the Collector instance
     * @return - Instance of collector
     */
    public static Collector GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Collector();
        }
        return _instance;
    }

    /**
     * Constructor for the Collector
     */
    public Collector() 
    { 
        _collectorMotor = new CANSparkMax(Constants.kCollectMotorDeviceId, MotorType.kBrushless);
        _collectorMotor.setInverted(true);
        _collectorMotor.setIdleMode(IdleMode.kBrake);

        _frontChamberMotor = new CANSparkMax(Constants.kChamberFrontMotorDeviceId, MotorType.kBrushless);
        _frontChamberMotor.setIdleMode(IdleMode.kBrake);

        _backChamberMotor = new CANSparkMax(Constants.kChamberBackMotorDeviceId, MotorType.kBrushless);
        _backChamberMotor.setIdleMode(IdleMode.kBrake);

        _collectorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.kCollectorSolenoidChannel);
    
        _chamberBottomSensor = new Photoeye(Constants.kChamberBottomPort);
        _chamberTopSensor = new Photoeye(Constants.kChamberTopPort);
    }

    /** 
     * Moves collect motor to a certain position 
     * @param speed - Speed you want to set motor to
    */
    public void moveCollect(double speed) {
        _collectorMotor.set(speed);
    }

    /**
     * Stops the collect motor from moving
     */
    public void stopCollect() {
        _collectorMotor.set(0.0);
    }

    /**
     * Stops the collector chamber back and front motors
     */
    public void stopChamber() {
        _frontChamberMotor.set(0.0);
        _backChamberMotor.set(0.0);
    }

    /**
     * Moves the collector chamber back and front motors, to collect the cargo
     */
    public void intake() {
        _collectorSolenoid.set(true);
        _collectorMotor.set(-_collectorSpeed);

        if (!_chamberBottomSensor.isBlocked()){
            _frontChamberMotor.set(-_chamberSpeed);
            _backChamberMotor.set(-_chamberSpeed);
        }
    }

    /**
     * Moves the collector motors, to spit the cargo
     */
    public void spit() {
        _collectorMotor.set(_collectorSpeed);
    }

    /**
     * Moves the collector chamber back and front motors, to eject the cargo
     */
    public void eject() {
        _collectorSolenoid.set(true);
        _collectorMotor.set(_collectorSpeed);
        _frontChamberMotor.set(_chamberSpeed);
        _backChamberMotor.set(-_chamberSpeed);
    }
    
    /**
     * Moves the cargo using the front and back chamber motors, and increase ball count accordingly 
     */
    public void feed() {
        _frontChamberMotor.set(-_chamberSpeed);
        _backChamberMotor.set(_chamberSpeed);
        
        if (!_chamberTopSensor.isBlocked() && _topChamberState)
        {
            if (_ballCount > 0)
                _ballCount--;
        }

        if (_chamberBottomSensor.isBlocked() && !_bottomChamberState)
        {
            _ballCount++;
            if (_ballCount >= 2) {
                _ballCount = 2;
            }
        }

        _topChamberState = _chamberTopSensor.isBlocked();
        _bottomChamberState = _chamberBottomSensor.isBlocked();
    }

    /**
     * Sets the ballCount in the collector based on the {@param ballCount}
     * @param ballCount - The number of balls in the collector
     */
    public void setBallCount(int ballCount)
    {
        _ballCount = ballCount;
    }

    /**
     * Runs mutiple if conditions that determine the ball count and whether or not the chamber will stopped
     */
    public void index()
    {
        if (_chamberTopSensor.isBlocked())
        {
            stopChamber();
        } else {
            if (_ballCount >= 2 || _chamberBottomSensor.isBlocked())
            {
                chamberUp();
            } else {
                stopChamber();
            }
        }

        if (!_chamberTopSensor.isBlocked() && _topChamberState)
        {
            if (_ballCount > 0)
                _ballCount--;
        }

        if (_chamberBottomSensor.isBlocked() && !_bottomChamberState)
        {
            _ballCount++;
            if (_ballCount >= 2) {
                _ballCount = 2;
            }
        }

        _topChamberState = _chamberTopSensor.isBlocked();
        _bottomChamberState = _chamberBottomSensor.isBlocked();
    }

    /**
     * Moves the collected front and back motors in the negative direction of the chamber speed.
     */
    public void chamberUp() {
        _frontChamberMotor.set(-_chamberSpeed);
        _backChamberMotor.set(-_chamberSpeed);
    }

    /**
     * Sets the collector solenoid output to true
     */
    public void collectorDown() {
        _collectorSolenoid.set(true);
    }

    /**
     * Sets the collector solenoid output to false
     */
    public void collectorUp() {
        _collectorSolenoid.set(false);
    }

    /**
     * Logs collector data into the SmartDashboard
     */
    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
        // SmartDashboard.putNumber("collectorSpeed", _collectorSpeed);
        // SmartDashboard.putNumber("chamberSpeed", _chamberSpeed);
        SmartDashboard.putNumber("frontChamberOutput", _frontChamberMotor.get());
        SmartDashboard.putNumber("backChamberOutput", _backChamberMotor.get());
        SmartDashboard.putBoolean("bottomChamberSensor", _chamberBottomSensor.isBlocked());
        SmartDashboard.putBoolean("topChamberSensor", _chamberTopSensor.isBlocked());
        SmartDashboard.putNumber("ballCount", _ballCount);
    }

    /**
     * Reads collector data from SmartDashboard... At the moment this function is not implemented
     */
    public void ReadDashboardData() {
        // TODO Auto-generated method stub

        //_collectorSpeed = SmartDashboard.getNumber("collectorSpeed", 0);
        //_chamberSpeed = SmartDashboard.getNumber("chamberSpeed", 0);

        
    }

}
 
