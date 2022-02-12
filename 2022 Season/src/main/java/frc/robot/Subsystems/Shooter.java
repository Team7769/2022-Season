package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements ISubsystem {

    private TalonFX _leftMotor;
    private TalonFX _rightMotor;
    private CANSparkMax _hoodMotor;
    private DutyCycleEncoder _hoodEncoder;

    private PIDController _hoodPID;
    private TalonFXConfiguration _leftMotorConfig;
    private TalonFXConfiguration _rightMotorConfig;
    public static Shooter _instance;

    private double _hoodTarget;
    /** 
     * Gets the shooter instance
    */
    public static Shooter GetInstance() {
        if (_instance == null) {
            _instance = new Shooter();
        }
        return _instance;
    }

    /** 
     * Constructor for implementing Shooter
    */
    public Shooter() {
        _leftMotor = new TalonFX(15);
        _rightMotor = new TalonFX(14);
        _hoodMotor = new CANSparkMax(10, MotorType.kBrushless);
        _hoodMotor.setInverted(true);
        _hoodMotor.setIdleMode(IdleMode.kBrake);

        _hoodEncoder = new DutyCycleEncoder(4);

        _leftMotorConfig = new TalonFXConfiguration();
        _rightMotorConfig = new TalonFXConfiguration();
        
        _leftMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source
        _leftMotorConfig.slot0.kP = .015;
        _leftMotorConfig.slot0.kF = .0473;

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightMotorConfig.remoteFilter0.remoteSensorDeviceID = _leftMotor.getDeviceID(); //Device ID of Remote Source
		_rightMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

        _leftMotor.setInverted(true);

        _rightMotor.follow(_leftMotor, FollowerType.PercentOutput);

        _leftMotor.configAllSettings(_leftMotorConfig);
        _rightMotor.configAllSettings(_rightMotorConfig);

        _hoodPID = new PIDController(4.5, 0, 0);
        _hoodPID.setTolerance(0.05);

        _hoodTarget = 0;
    }

    public void manualHood(double speed)
    {
        _hoodMotor.set(speed);
    }

    public void readyShot()
    {
        setHoodPosition(_hoodTarget);
    }

    public void setHalfShot()
    {
        _hoodTarget = 0.5;
    }

    public void setQuarterShot()
    {
        _hoodTarget = 0.25;
    }

    public void setThreeQuarterShot()
    {
        _hoodTarget = 0.75;
    }

    public void stop()
    {
        _hoodMotor.set(0);
    }

    private void setHoodPosition(double position)
    {
        if (position != _hoodPID.getSetpoint())
        {
            _hoodPID.reset();
        }

        var output = _hoodPID.calculate(_hoodEncoder.get(), position);
        
        if (Math.abs(output) >= 0.25)
        {
            if (output > 0)
            {
                output = .25;
            } else {
                output = -.25;
            }
        }
        SmartDashboard.putNumber("hoodOutput", output);

        _hoodMotor.set(-output);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("hoodPosition", _hoodEncoder.get());
        SmartDashboard.putNumber("hoodOffset", _hoodEncoder.getPositionOffset());
        SmartDashboard.putNumber("hoodFrequency", _hoodEncoder.getFrequency());
        SmartDashboard.putNumber("hoodTarget", _hoodTarget);
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
