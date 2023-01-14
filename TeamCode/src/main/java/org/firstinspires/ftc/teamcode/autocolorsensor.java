package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "autocolorsensor", preselectTeleOp = "drive (Blocks to Java)")
public class autocolorsensor extends LinearOpMode {

    private Servo graber;
    private DcMotor armmotor;
    private DcMotor backleft;
    private DcMotor frontright;
    private ColorSensor colorsensor_REV_ColorRangeSensor;
    private DcMotor backright;
    private DcMotor frontleft;
    private CRServo arm;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int loop_count;

        graber = hardwareMap.get(Servo.class, "graber");
        armmotor = hardwareMap.get(DcMotor.class, "arm motor");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        colorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "color sensor");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        arm = hardwareMap.get(CRServo.class, "arm");

        graber.setPosition(0.1);
        waitForStart();
        armmotor.setPower(1);
        sleep(1500);
        armmotor.setPower(0);
        sleep(1000);
        if (opModeIsActive()) {
            update_colors();
            telemetry.update();
            backleft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontright.setDirection(DcMotorSimple.Direction.REVERSE);
            loop_count = 0;
            while (4 < ((DistanceSensor) colorsensor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM)) {
                loop_count += 1;
                drive(0.1, 0.25);
                if (loop_count > 30) {
                    sleep(25000);
                }
            }
            drive(0.75, 0.25);
            update_colors();
            if (read_color().equals("red")) {
                crab_walk_left(0.9, 0.8);
                update_colors();
            } else if (read_color().equals("green")) {
                crab_walk_left(0, 0);
                update_colors();
            } else {
                crab_walk_left(0.9, -0.8);
                update_colors();
            }
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void update_colors() {
        telemetry.addData("Color value: ", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.argb()));
        telemetry.addData("Red value: ", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.red()));
        telemetry.addData("Blue value: ", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.blue()));
        telemetry.addData("Green value:", JavaUtil.colorToText(colorsensor_REV_ColorRangeSensor.green()));
        telemetry.addData("Color distance: ", ((DistanceSensor) colorsensor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
        telemetry.addData("Read Color Function Output", read_color());
    }

    /**
     * Describe this function...
     */
    private void drive(double time2, double Power) {
        backleft.setPower(Power);
        backright.setPower(Power);
        frontleft.setPower(Power);
        frontright.setPower(Power);
        sleep((long) (time2 * 1000));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void crab_walk_left(double time2, double Power) {
        backleft.setPower(Power);
        backright.setPower(-Power);
        frontleft.setPower(-Power);
        frontright.setPower(Power);
        sleep((long) (time2 * 1000));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void arm2(double time2, double Power) {
        arm.setPower(Power);
        sleep((long) (time2 * 1000));
        arm.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void turn(double time2, double Power) {
        backleft.setPower(Power * -1);
        backright.setPower(Power);
        frontleft.setPower(Power * -1);
        frontright.setPower(Power);
        sleep((long) (time2 * 1000));
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
    }

    /**
     * Describe this function...
     */
    private String read_color() {
        int red_value;
        String color_result;
        int blue_value;
        int green_value;

        red_value = colorsensor_REV_ColorRangeSensor.red();
        blue_value = colorsensor_REV_ColorRangeSensor.blue();
        green_value = colorsensor_REV_ColorRangeSensor.green();
        if (blue_value == JavaUtil.maxOfList(JavaUtil.createListWith(red_value, blue_value, green_value))) {
            color_result = "blue";
        } else if (red_value == JavaUtil.maxOfList(JavaUtil.createListWith(red_value, blue_value, green_value))) {
            color_result = "red";
        } else {
            color_result = "green";
        }
        return color_result;
    }
}
