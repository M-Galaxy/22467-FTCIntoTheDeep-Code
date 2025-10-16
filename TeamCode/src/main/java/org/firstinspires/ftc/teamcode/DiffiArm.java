package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
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


@TeleOp(name = "DiffiArm")

public class DiffiArm extends LinearOpMode {
    
    // Debug turns on all of the telemetry that is unessary
    private boolean debug = false;
    
    private DcMotor LSlide;
    private DcMotor RSlide;
    
    private DcMotor LArm;
    private DcMotor RArm;
    
    private CRServo LIntake;
    private CRServo RIntake;
    
    private boolean FineMovemnt;
    
    public void slideMovement() {
        double x = gamepad2.right_stick_x;
        
        double[] raising = new double[]{0, 0};
        double[] tilting = new double[]{0, 0};
        
        if(gamepad2.dpad_up){
            raising = new double[]{-1, 1};
        }
        else{if(gamepad2.dpad_down){
            raising = new double[]{1, -1};
        }else{
            raising = new double[]{0, 0};
        }}
        
        
        if(gamepad2.left_bumper){
            tilting = new double[]{1, 1};
        }
        else{if(gamepad2.right_bumper){
            tilting = new double[]{-1, -1};
        }else{
            tilting = new double[]{0, 0};
        }}
        
        double intake = 0;
        if(gamepad2.a){
            intake += 1;
        }
        if(gamepad2.b){
            intake -= 1;
        }
        
        LIntake.setPower(intake  * -1);
        RIntake.setPower(intake);
        
        double[] tempVal = new double[]{raising[1] + tilting[1], raising[0] + tilting[0]};
        // temporary motor value
    
        double[] abs = new double[]{Math.abs(tempVal[0]), Math.abs(tempVal[1])};
    
        double max = Math.max(abs[0], abs[1]);
    
        double scale = Math.max(1, max);
    
        double[] out = new double[]{tempVal[0] / scale, tempVal[1] / scale};
    
        LSlide.setPower(out[0]);
        RSlide.setPower(out[1]);
        
        double y = -gamepad2.left_stick_y;
        
        LArm.setPower(y);
        RArm.setPower(y);
        
        if (debug){
            telemetry.addData("output", out);
            telemetry.addData("X", x); // X between -1 and 1
            telemetry.addData("Y", y); // Y between -1 and 1
            
            telemetry.update();
        }
    }

    
    
    @Override
    public void runOpMode() {
        
        
        // the init for motors
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        
        LArm = hardwareMap.get(DcMotor.class, "LArm");
        RArm = hardwareMap.get(DcMotor.class, "RArm");
        
        LIntake = hardwareMap.get(CRServo.class, "LIntake");
        RIntake = hardwareMap.get(CRServo.class, "RIntake");
        
        LSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        LArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        RArm.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            slideMovement();
            
        }
    }
}
