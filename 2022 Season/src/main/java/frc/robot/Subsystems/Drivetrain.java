package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Configuration.Constants;

public class Drivetrain implements ISubsystem {
    
    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightRearMotor;
    private DifferentialDrive _robotDrive;

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

        _robotDrive = new DifferentialDrive(_leftFrontMotor, _rightFrontMotor);
    }

    public void drive(double throttle, double turn)
    {
        _rightFrontMotor.setIdleMode(IdleMode.kCoast);
        _leftFrontMotor.setIdleMode(IdleMode.kCoast);
        _robotDrive.arcadeDrive(throttle, turn);
    }

    public void LogTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public void ReadDashboardData() {
        // TODO Auto-generated method stub
        
    }
}
