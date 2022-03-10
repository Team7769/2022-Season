package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import frc.robot.Configuration.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements ISubsystem {

    private TalonFX _leftMotor;
    private TalonFX _rightMotor;
    private CANSparkMax _hoodMotor;
    private DutyCycleEncoder _hoodEncoder;
    private SparkMaxLimitSwitch _limitSwitch;

    private PIDController _hoodPID;
    private TalonFXConfiguration _leftMotorConfig;
    private TalonFXConfiguration _rightMotorConfig;
    public static Shooter _instance;

    private double _hoodTarget;
    private double _shooterTarget;
    private String _targetName;
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

        _hoodEncoder = new DutyCycleEncoder(Constants.kHoodEncoderPort);
        _limitSwitch = _hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        _leftMotorConfig = new TalonFXConfiguration();
        _rightMotorConfig = new TalonFXConfiguration();
        
        _leftMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source
        _leftMotorConfig.slot0.kP = Constants.kShooterKp;
        _leftMotorConfig.slot0.kF = Constants.kShooterKf;

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightMotorConfig.remoteFilter0.remoteSensorDeviceID = _leftMotor.getDeviceID(); //Device ID of Remote Source
		_rightMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

        _leftMotor.setInverted(true);

        _rightMotor.follow(_leftMotor, FollowerType.PercentOutput);

        _leftMotor.configAllSettings(_leftMotorConfig);
        _rightMotor.configAllSettings(_rightMotorConfig);

        _hoodPID = new PIDController(Constants.kHoodKp, Constants.kHoodKi, Constants.kHoodKd);
        _hoodPID.setTolerance(0.05);

        _hoodTarget = 0;
        _shooterTarget = 0;

        setFarShot();
    }

    public void zeroHood()
    {
        if (_limitSwitch.isPressed())
        {
            _hoodEncoder.reset();
        }
        //_hoodEncoder.reset();
    }

    public void manualHood(double speed)
    {
        _hoodMotor.set(speed);
    }

    public void readyShot()
    {
        setHoodPosition(_hoodTarget);
        setSpeed(_shooterTarget);
    }
    
    public boolean goShoot()
    {
        boolean shooterAtSpeed = false;
        if (_shooterTarget == Constants.kFarShotSpeed) {
            shooterAtSpeed = (Math.abs(_leftMotor.getClosedLoopError()) < 1000);
        } else {
            shooterAtSpeed = (Math.abs(_leftMotor.getClosedLoopError()) < 500);
        }
        
        return shooterAtSpeed && _hoodPID.atSetpoint();
    }

    public void setPukeShot()
    {
        _shooterTarget = Constants.kPukeShotSpeed;
        _hoodTarget = Constants.kThreeQuarterShotValue;
        _targetName = "Puke Shot";
    }

    public void setZoneShot()
    {
        _shooterTarget = Constants.kZoneShotSpeed;
        _hoodTarget = Constants.kHalfShotValue;
        _targetName = "Zone Shot";
    }

    public void setCloseShot()
    {
        _shooterTarget = Constants.kCloseShotSpeed;
        _hoodTarget = Constants.kQuarterShotValue;
        _targetName = "Close Shot";
    }

    public void setFarShot()
    {
        _shooterTarget = Constants.kFarShotSpeed;
        _hoodTarget = Constants.kThreeQuarterShotValue;
        _targetName = "Far Shot";
    }
    
    private void setSpeed(double speed)
    {
        _leftMotor.set(TalonFXControlMode.Velocity, speed);
    }

    public void stop()
    {
        _leftMotor.set(TalonFXControlMode.PercentOutput, 0);
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
        SmartDashboard.putNumber("hoodSparkOutput", _hoodMotor.get());
        _hoodMotor.set(-output);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("hoodPosition", _hoodEncoder.get());
        SmartDashboard.putNumber("hoodOffset", _hoodEncoder.getPositionOffset());
        SmartDashboard.putNumber("hoodFrequency", _hoodEncoder.getFrequency());
        SmartDashboard.putBoolean("limitSwitchBlocked", _limitSwitch.isPressed());
        SmartDashboard.putNumber("hoodDistance", _hoodEncoder.getDistance());
        SmartDashboard.putNumber("hoodTarget", _hoodTarget);
        SmartDashboard.putString("currentShot", _targetName);
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
