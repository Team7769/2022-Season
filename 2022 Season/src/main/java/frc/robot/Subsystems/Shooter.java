package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Configuration.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Lib.MagneticLimitSwitch;

public class Shooter implements ISubsystem {

    private TalonFX _leftMotor;
    private TalonFX _rightMotor;
    private CANSparkMax _hoodMotor;
    private DutyCycleEncoder _hoodEncoder;
    private MagneticLimitSwitch _limitSwitch;

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
        _leftMotor = new TalonFX(Constants.kLeftShooterMotorDeviceId);
        _rightMotor = new TalonFX(Constants.kRightShooterMotorDeviceId);
        _hoodMotor = new CANSparkMax(Constants.kHoodMotorDeviceId, MotorType.kBrushless);
        _hoodMotor.setInverted(true);
        _hoodMotor.setIdleMode(IdleMode.kBrake);

        _hoodEncoder = new DutyCycleEncoder(Constants.kHoodEncoderDeviceId);
        _limitSwitch = new MagneticLimitSwitch(Constants.magneticLimitSwitchId);

        _leftMotorConfig = new TalonFXConfiguration();
        _rightMotorConfig = new TalonFXConfiguration();
        
        _leftMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source
        _leftMotorConfig.slot0.kP = Constants.kLeftMotorKp;
        _leftMotorConfig.slot0.kF = Constants.kLeftMotorKf;

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightMotorConfig.remoteFilter0.remoteSensorDeviceID = _leftMotor.getDeviceID(); //Device ID of Remote Source
		_rightMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

        _leftMotor.setInverted(true);

        _rightMotor.follow(_leftMotor, FollowerType.PercentOutput);

        _leftMotor.configAllSettings(_leftMotorConfig);
        _rightMotor.configAllSettings(_rightMotorConfig);

        _hoodPID = new PIDController(Constants.hoodKp, Constants.hoodKi, Constants.hoodKd);
        _hoodPID.setTolerance(0.05);

        _hoodTarget = 0;
    }

    public void zeroHood()
    {
        if (_limitSwitch.isBlocked())
        {
            _hoodEncoder.reset();
        }
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
        _hoodTarget = Constants.kHalfShotValue;
    }

    public void setQuarterShot()
    {
        _hoodTarget = Constants.kQuarterShotValue;
    }

    public void setThreeQuarterShot()
    {
        _hoodTarget = Constants.kThreeQuarterShotValue;
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
        SmartDashboard.putBoolean("limitSwitchBlocked", _limitSwitch.isBlocked());
        SmartDashboard.putNumber("hoodDistance", _hoodEncoder.getDistance());
        SmartDashboard.putNumber("hoodTarget", _hoodTarget);
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
