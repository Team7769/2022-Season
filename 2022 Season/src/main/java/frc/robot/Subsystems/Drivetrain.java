package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain {
    
    private CANSparkMax _leftFrontMotor;
    private CANSparkMax _leftRearMotor;
    private CANSparkMax _rightFrontMotor;
    private CANSparkMax _rightRearMotor;

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
        _leftFrontMotor = new CANSparkMax(3, MotorType.kBrushless);

        _leftRearMotor = new CANSparkMax(4, MotorType.kBrushless);

        _rightFrontMotor = new CANSparkMax(5, MotorType.kBrushless);

        _rightRearMotor = new CANSparkMax(6, MotorType.kBrushless);

        _leftRearMotor.follow(_leftFrontMotor);
        _rightRearMotor.follow(_rightFrontMotor);
    }
}
