package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//@TeleOp(name = "AutoToHP")
@Autonomous(name = "AutoToHP")

public class AutoToHP extends LinearOpMode {
    
    // Debug turns on all of the telemetry that is unessary
    private boolean debug = true;

  private double wheelLF;
  private double wheelRF;
  private double wheelLB;
  private double wheelRB;

  private DcMotor RBMotor;
  private DcMotor RFMotor;
  private DcMotor LFMotor;
  private DcMotor LBMotor;

  public void movement(double X, double Y, double Z) {

      //float Rtrigger = gamepad1.left_trigger;
      //float Ltrigger = gamepad1.right_trigger;
      //turn = Rtrigger - Ltrigger;

      //turn = gamepad1.right_stick_x;
      double turn = Z;
      double strafe = X;
      double speed = Y;


      wheelLF = speed + strafe - turn;
      wheelRF = speed - strafe + turn;
      wheelLB = speed - strafe - turn;
      wheelRB = speed + strafe + turn;

      RFMotor.setPower(wheelRF);
      RBMotor.setPower(wheelRB);
      LFMotor.setPower(wheelLF);
      LBMotor.setPower(wheelLB);
  }
    BNO055IMU imu;

    
    // this is all the slide stuff ---------------------------------------------------------------------------
    private DcMotor LSlide;
    private DcMotor RSlide;
    
    private CRServo LIntake;

    
    private boolean FineMovemnt;
    


    
    
    @Override
    public void runOpMode() {
        // the init for the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
        
        
        // the init for motors
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        // the init for slides
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        
        LIntake = hardwareMap.get(CRServo.class, "LIntake");
        
        LSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        movement(-0.75, 0, 0);

        sleep(1000);

        movement(0, 0, 0);

    }
}
