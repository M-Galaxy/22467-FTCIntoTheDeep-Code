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


@TeleOp(name = "DiffiSlides")

public class DiffiSlides extends LinearOpMode {
    
    // Debug turns on all of the telemetry that is unessary
    private boolean debug = false;
    
    private DcMotor LSlide;
    private DcMotor RSlide;
    
    public void slideMovement() {
        double y = -gamepad2.left_stick_y;
        double x = gamepad2.right_stick_x;
    
        double[] raising = new double[]{y * -1, y};
        double[] tilting = new double[]{x, x};
    
        double[] tempVal = new double[]{raising[1] + tilting[1], raising[0] + tilting[0]};
        // temporary motor value
    
        double[] abs = new double[]{Math.abs(tempVal[0]), Math.abs(tempVal[1])};
    
        double max = Math.max(abs[0], abs[1]);
    
        double scale = Math.max(1, max);
    
        double[] out = new double[]{tempVal[0] / scale, tempVal[1] / scale};
    
        LSlide.setPower(out[0]);
        RSlide.setPower(out[1]);
        
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
        
        LSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            slideMovement();
            
        }
    }
}
