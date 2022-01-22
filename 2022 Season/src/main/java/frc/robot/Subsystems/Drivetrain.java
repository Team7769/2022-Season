package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Drivetrain implements ISubsystem {
    
    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightRearMotor;
    private DifferentialDrive _robotDrive;
    private Encoder _leftDriveEncoder;
    private Encoder _rightDriveEncoder;
    private AHRS _gyro;

    private static Drivetrain _instance;

    public static Drivetrain GetInstance()
    {
        if (_instance == null)
        {
            _instance = new Drivetrain();
        }
        return _instance;
    }

    public Drivetrain()
    {
        _leftFrontMotor = new CANSparkMax(Constants.kLeftFrontDriveDeviceId, MotorType.kBrushless);
        _leftRearMotor = new CANSparkMax(Constants.kLeftRearDriveDeviceId, MotorType.kBrushless);

        _rightFrontMotor = new CANSparkMax(Constants.kRightFrontDriveDeviceId, MotorType.kBrushless);
        _rightFrontMotor.setInverted(true);

        _rightRearMotor = new CANSparkMax(Constants.kRightRearDriveDeviceId, MotorType.kBrushless);
        _rightRearMotor.setInverted(true);

        _leftRearMotor.follow(_leftFrontMotor);
        _rightRearMotor.follow(_rightFrontMotor);

        _gyro = new AHRS(Port.kMXP);

        _leftDriveEncoder = new Encoder(Constants.kLeftEncoderPortA, Constants.kLeftEncoderPortB);
        _leftDriveEncoder.setDistancePerPulse(Constants.kDriveDistancePerPulse);

        _rightDriveEncoder = new Encoder(Constants.kRightEncoderPortA, Constants.kRightEncoderPortB, true);
        _rightDriveEncoder.setDistancePerPulse(Constants.kDriveDistancePerPulse);

        _robotDrive = new DifferentialDrive(_leftFrontMotor, _rightFrontMotor);
    }

    public void drive(double throttle, double turn)
    {
        _rightFrontMotor.setIdleMode(IdleMode.kCoast);
        _leftFrontMotor.setIdleMode(IdleMode.kCoast);
        _robotDrive.arcadeDrive(throttle, turn);
    }
    
    public double getHeading() {
        return Math.IEEEremainder(_gyro.getAngle(), 360) * (-1.0);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("leftDriveDistance", _leftDriveEncoder.getDistance());
        SmartDashboard.putNumber("leftDriveRate", _leftDriveEncoder.getRate());
        SmartDashboard.putNumber("rightDriveDistance", _rightDriveEncoder.getDistance());
        SmartDashboard.putNumber("rightDriveRate", _rightDriveEncoder.getRate());
        SmartDashboard.putNumber("gyroHeading", getHeading());
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
