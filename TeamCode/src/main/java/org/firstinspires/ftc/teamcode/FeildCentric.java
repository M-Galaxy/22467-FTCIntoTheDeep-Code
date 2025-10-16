package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "FeildCentric")

public class FeildCentric extends LinearOpMode {
    
    // Debug turns on all of the telemetry that is unessary
    private boolean debug = false;

    // This is all movement stuffs ---------------------------------------------------------------------------------------//
    public double calculateAngle() {
        double y = -gamepad1.left_stick_y; // Invert y to match robot's forward
        double x = gamepad1.left_stick_x;
        
        // Calculate the angle based on joystick input
        double angle = Math.toDegrees(Math.atan2(y, x)); // atan2 gives angle in radians
        return angle >= 0 ? angle : angle + 360; // Normalize angle to be between 0 and 360 degrees
    }
    
    public double calculateTrueMovementAngle(double joystickAngle, float heading) {
        // Subtract the heading from the joystick angle to get true movement angle
        double trueAngle = joystickAngle - heading;
        // Normalize the angle to be between 0 and 360
        return trueAngle >= 0 ? trueAngle : trueAngle + 360;
    }
    
    // Calculate speed based on joystick distance from center
    public double calculateSpeed() {
        double y = -gamepad1.left_stick_y; // Invert y to match forward movement
        double x = gamepad1.left_stick_x;

        // Calculate speed as the magnitude of the joystick vector
        return Math.sqrt(x * x + y * y);
    }

    // Convert the true angle and speed into X and Y between -1 and 1
    public double[] calculateMovementVector(double trueAngle, double speed) {
        // Convert angle to radians for Math.cos and Math.sin
        double angleRadians = Math.toRadians(trueAngle);
        // Calculate X and Y using speed as a scaling factor
        double x = speed * Math.cos(angleRadians);
        double y = speed * Math.sin(angleRadians);
        return new double[]{x, y}; // Return X and Y as an array
    }
    
    
      private double wheelLF;
      private double wheelRF;
      private double wheelLB;
      private double wheelRB;
      
      private DcMotor RBMotor;
      private DcMotor RFMotor;
      private DcMotor LFMotor;
      private DcMotor LBMotor;
    
    double turn;
    
    public void movement(double X, double Y){
    
    //float Rtrigger = gamepad1.left_trigger;
    //float Ltrigger = gamepad1.right_trigger;
    //turn = Rtrigger - Ltrigger;
    
    turn = -1 * gamepad1.right_stick_x;
    
    double strafe = X;
    double speed = Y;
    
    
    
    wheelLF = speed + strafe - turn;
    wheelRF = speed - strafe + turn;
    wheelLB = speed - strafe - turn;
    wheelRB = speed + strafe + turn;
    
    if (debug){
        //telemetry.addData("Left Trigger", Rtrigger);
        //telemetry.addData("Right Trigger", Ltrigger);

        telemetry.addData("turn", turn);
        telemetry.addData("strafe", strafe);
        telemetry.addData("speed", speed);

        telemetry.addData("wheelLF", wheelLF);
        telemetry.addData("wheelRF", wheelRF);
        telemetry.addData("wheelLB", wheelLB);
        telemetry.addData("wheelRB", wheelRB);
    }
    }
    
    BNO055IMU imu;

    public void wheelMovement(){
        // calculates the angle of the stick
        double angle = calculateAngle();
        
        
        
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float heading = angles.firstAngle;  // This is compass heading

        
        
        // the 'True' movement angle 
        double trueMovementAngle = calculateTrueMovementAngle(angle, heading);
        
        
        // Calculate speed based on joystick input
        double speed = calculateSpeed();
        // Convert the true movement angle and speed into X and Y components
        double[] movementVector = calculateMovementVector(trueMovementAngle, speed);
        double x = movementVector[0];
        double y = movementVector[1];

        if (debug){
            telemetry.addData("Stick Angle", angle);
            telemetry.addData("Heading", heading);
            telemetry.addData("True Movement Angle", trueMovementAngle);
            telemetry.addData("X", x); // X between -1 and 1
            telemetry.addData("Y", y); // Y between -1 and 1
            
            telemetry.update();
        }
        
        movement(x,y);
    
        RFMotor.setPower(wheelRF);
        RBMotor.setPower(wheelRB);
        LFMotor.setPower(wheelLF);
        LBMotor.setPower(wheelLB);
    }

        

    
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
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            wheelMovement();
            
        }
    }
}
