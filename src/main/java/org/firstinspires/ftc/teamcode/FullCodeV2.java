package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "FullCodeV2")

public class FullCodeV2 extends LinearOpMode {
    
    // Debug turns on all of the telemetry that is unessary
    private boolean debug = true;

    GoBildaPinpointDriver odo;

    
    
    
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
    
    turn = gamepad1.right_stick_x;
    
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

    public void wheelMovement() {
        // Calculate the angle of the joystick (stick)
        double angle = calculateAngle();

        Pose2D pos = odo.getPosition();
        float heading =  (float)pos.getHeading(DEGREES) + 180;

        // Get the heading from the IMU (assumed to be between -180 and 180)
        //float heading = 100 * ((float) odo.getHeading());  // Getting current heading from GoBilda IMU

        // Normalize the heading to be between -180 and 180 degrees
        //heading = normalizeAngle(heading);

        // The 'True' movement angle is adjusted by the robot's heading (for field-centric driving)
        double trueMovementAngle = calculateTrueMovementAngle(angle, heading);

        // Calculate the speed based on joystick input
        double speed = calculateSpeed();

        // Convert the true movement angle and speed into X and Y components
        double[] movementVector = calculateMovementVector(trueMovementAngle, speed);
        double x = movementVector[0];
        double y = movementVector[1];

        if (debug) {
            telemetry.addData("Stick Angle", angle);
            telemetry.addData("Heading", heading);
            telemetry.addData("True Movement Angle", trueMovementAngle);
            telemetry.addData("X", x); // X between -1 and 1
            telemetry.addData("Y", y); // Y between -1 and 1
        }

        movement(x, y);

        // Apply motor power based on calculated wheel speeds
        RFMotor.setPower(wheelRF);
        RBMotor.setPower(wheelRB);
        LFMotor.setPower(wheelLF);
        LBMotor.setPower(wheelLB);
    }





    // slide ---------------------------
// Debug turns on all of the telemetry that is unnecessary
    private DcMotor LSlide;
    private DcMotor RSlide;
    private CRServo intake;
    private Servo goodtilt;     // Added goodtilt as a standard servo

    private double[] offsets = {0, 0};

    private double armGoal;
    private double armGoalDeadzone = 100;
    private double positon;

    private double top = 2000;

    private boolean slidesInPos = false;

    // Tilt control thresholds
    private double positionThreshold = 1500; // Slide position at which the tilt activates
    private boolean tiltMoved = false;

    public void moveSlides(boolean input, double endPoint) {

        int LPos = LSlide.getCurrentPosition();
        int RPos = RSlide.getCurrentPosition();

        positon = ((LSlide.getCurrentPosition() + offsets[0]) + (RSlide.getCurrentPosition() + offsets[1])) / 2;

        if (input) {
            double up = 0;

            if (gamepad2.dpad_up) {
                up = 25;
            } else if (gamepad2.dpad_down) {
                up = -25;
            }

            armGoal += up;

            if (armGoal > top) {
                armGoal = top;
            }
            if (armGoal < 0) {
                armGoal = 0;
            }
        } else {
            armGoal = endPoint;
        }

        // Calculate the error (distance to goal)
        double error = armGoal - positon;

        // Use proportional control with speed reduction near the target
        double out = error * 0.1;

        // Clamp power to a maximum and minimum range
        double minPower = 0.1; // Prevent stalling near the target
        double maxPower = 1.0;

        // Scale the power down as it gets closer
        out = Math.signum(out) * Math.max(minPower, Math.min(Math.abs(out), maxPower));

        // Deadzone check
        boolean insideDeadzone = Math.abs(error) <= armGoalDeadzone;

        if (!insideDeadzone) {
            LSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            LSlide.setPower(out);
            RSlide.setPower(out);
        } else {
            LSlide.setTargetPosition((int) (armGoal + offsets[0]));
            RSlide.setTargetPosition((int) (armGoal + offsets[1]));

            LSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            LSlide.setPower(0.3); // Low power for holding position
            RSlide.setPower(0.3);
        }

        // Adjust tilt servo based on position
        adjustTiltBasedOnPosition();

        // Debug telemetry
        if (debug) {
            telemetry.addData("Speed on Arm: ", out);
            telemetry.addData("Arm Goal: ", armGoal);
            telemetry.addData("Position: ", positon);
            telemetry.addData("Tilt Moved: ", tiltMoved);
            telemetry.update();
        }

        // Check if slides are in position
        slidesInPos = !input && insideDeadzone;
    }

    double tiltDown = 0.85;

    // Function to simulate moving the tilt CRServo to a "position"
    private void adjustTiltBasedOnPosition() {
        // Move tilt forward if slides are above the threshold
        if (positon > positionThreshold) {
            goodtilt.setPosition(tiltDown);
        }

        // Reverse tilt if slides go back below the threshold
        if (positon < positionThreshold) {
            goodtilt.setPosition(1.0);
        }
    }

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(96, -137.25);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        
        
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
        // Initialization for motors
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        goodtilt = hardwareMap.get(Servo.class, "goodtilt");
        intake = hardwareMap.get(CRServo.class, "LIntake");

        // Set motors to use encoders
        LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        offsets[0] = -LSlide.getCurrentPosition();
        offsets[1] = -RSlide.getCurrentPosition();

        LSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        positon = 0;
        armGoal = 0;

        positon = ((LSlide.getCurrentPosition() + offsets[0]) + (RSlide.getCurrentPosition() + offsets[1])) / 2;

        LSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        goodtilt.setPosition(1.0);
        
        
        waitForStart();
        
        while (opModeIsActive()) {
            odo.update();
            wheelMovement();
            
            telemetry.update();


            moveSlides(true, 0);

            if (gamepad2.a) {
                goodtilt.setPosition(1.0); // Set goodtilt to max position
            }
            if (gamepad2.b) {
                goodtilt.setPosition(tiltDown); // Set goodtilt to min position
            }
            if (gamepad2.y) {
                moveSlides(false, top);
            }
            if (gamepad2.x) {
                moveSlides(false, 10);
            }

            if (gamepad2.left_bumper) {
                intake.setPower(0.2);
            } else if (gamepad2.right_bumper) {
                intake.setPower(-0.2);
            } else {
                intake.setPower(0);
            }
        }
    }
}
