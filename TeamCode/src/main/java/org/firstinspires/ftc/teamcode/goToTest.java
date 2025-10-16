/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name="goToTest", group="Linear OpMode")
//@Disabled

public class goToTest extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;


    boolean debug = false;


    private double wheelLF;
    private double wheelRF;
    private double wheelLB;
    private double wheelRB;

    private DcMotor RBMotor;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;

    double[] goal = {0, 0}; // Goal position (X, Y)
    double kp = 0.005; // Proportional control constant

    public void movement(double X, double Y){
        double turn = gamepad1.right_stick_x;
        double strafe = X;
        double speed = Y;

        wheelLF = speed + strafe - turn;
        wheelRF = speed - strafe + turn;
        wheelLB = speed - strafe - turn;
        wheelRB = speed + strafe + turn;

        if (debug){
            telemetry.addData("turn", turn);
            telemetry.addData("strafe", strafe);
            telemetry.addData("speed", speed);

            telemetry.addData("wheelLF", wheelLF);
            telemetry.addData("wheelRF", wheelRF);
            telemetry.addData("wheelLB", wheelLB);
            telemetry.addData("wheelRB", wheelRB);
        }

        // Set motor powers
        LFMotor.setPower(wheelLF);
        RFMotor.setPower(wheelRF);
        LBMotor.setPower(wheelLB);
        RBMotor.setPower(wheelRB);
    }

    @Override
    public void runOpMode() {
        // Motor initialization
        RBMotor = hardwareMap.get(DcMotor .class, "RBMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(96, -137.25);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double currentX = pos.getX(DistanceUnit.MM);
            double currentY = pos.getY(DistanceUnit.MM);

            // Calculate errors
            double errorX = goal[0] - currentY;
            double errorY = goal[1] - currentX;

            // Apply proportional control
            double moveX = errorX * kp;
            double moveY = errorY * kp;

            // Dead zone threshold to stop movement when close to the goal
            if (Math.abs(errorX) < 10 && Math.abs(errorY) < 10) {
                moveX = 0;
                moveY = 0;
            }

            // Feed movement values
            movement(moveX, moveY);

            // Telemetry data
            telemetry.addData("Goal X", goal[0]);
            telemetry.addData("Goal Y", goal[1]);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.update();

            if (gamepad1.a) {
                odo.resetPosAndIMU();
            }

            if (gamepad1.b) {
                odo.recalibrateIMU();
            }
        }
    }
}
