//Experimental SCARA support for Smoothieware, this is based on the MorganSCARA code and https://roboted.wordpress.com/fundamentals/
// ***WARNING THIS CODE COMPILES BUT HAS YET TO BE TRIED OR DEBUGGED
// Douglas.Pearless@pearless.co.nz
//
#include "SCARASolution.h"
#include <fastmath.h>
#include "checksumm.h"
#include "ActuatorCoordinates.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "StreamOutputPool.h"
//#include "Gcode.h"
//#include "SerialMessage.h"
//#include "Conveyor.h"
//#include "Robot.h"
//#include "StepperMotor.h"

#include "libs/nuts_bolts.h"

#include "libs/Config.h"

#define arm1_length_checksum         CHECKSUM("arm1_length")
#define arm2_length_checksum         CHECKSUM("arm2_length")
#define SCARA_offset_x_checksum      CHECKSUM("SCARA_offset_x")
#define SCARA_offset_y_checksum      CHECKSUM("SCARA_offset_y")
#define SCARA_scaling_x_checksum     CHECKSUM("SCARA_scaling_x")
#define SCARA_scaling_y_checksum     CHECKSUM("SCARA_scaling_y")
#define SCARA_homing_checksum        CHECKSUM("SCARA_homing")
#define SCARA_undefined_min_checksum CHECKSUM("SCARA_undefined_min")
#define SCARA_undefined_max_checksum CHECKSUM("SCARA_undefined_max")

#define SQ(x) powf(x, 2)
#define ROUND(x, y) (roundf(x * 1e ## y) / 1e ## y)

SCARASolution::SCARASolution(Config* config)
{
    // arm1_length is the length of the inner main arm from hinge to hinge
    arm1_length         = config->value(arm1_length_checksum)->by_default(150.0f)->as_number();
    // arm2_length is the length of the inner main arm from hinge to hinge
    arm2_length         = config->value(arm2_length_checksum)->by_default(150.0f)->as_number();
    // SCARA_offset_x is the x offset of bed zero position towards the SCARA tower center
    SCARA_offset_x     = config->value(SCARA_offset_x_checksum)->by_default(100.0f)->as_number();
    // SCARA_offset_y is the y offset of bed zero position towards the SCARA tower center
    SCARA_offset_y     = config->value(SCARA_offset_y_checksum)->by_default(-60.0f)->as_number();
    // Axis scaling is used in final calibration
    SCARA_scaling_x    = config->value(SCARA_scaling_x_checksum)->by_default(1.0F)->as_number(); // 1 = 100% : No scaling
    SCARA_scaling_y    = config->value(SCARA_scaling_y_checksum)->by_default(1.0F)->as_number();
    // SCARA_undefined is the ratio at which the SCARA position is undefined.
    // required to prevent the arm moving through singularity points
    // min: head close to tower
    SCARA_undefined_min  = config->value(SCARA_undefined_min_checksum)->by_default(0.95f)->as_number();
    // max: head on maximum reach
    SCARA_undefined_max  = config->value(SCARA_undefined_max_checksum)->by_default(0.95f)->as_number();

    init();
}

void SCARASolution::init() {

}

float SCARASolution::to_degrees(float radians) {
    return radians*(180.0F/3.14159265359f);
}

void SCARASolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm )
{

    float SCARA_pos[2],
          SCARA_C2,
          SCARA_Shoulder,
          SCARA_Elbow,
		  a;

    SCARA_pos[X_AXIS] = (cartesian_mm[X_AXIS] + this->SCARA_offset_x) * this->SCARA_scaling_x;  //Translate cartesian to tower centric SCARA X Y AND apply scaling factor from this offset.
    SCARA_pos[Y_AXIS] = (cartesian_mm[Y_AXIS] + this->SCARA_offset_y) * this->SCARA_scaling_y;

    a = SQ(SCARA_pos[X_AXIS])+SQ(SCARA_pos[Y_AXIS])-SQ(this->arm1_length)-SQ(this->arm2_length); //we use this more than once, so save computational time

    SCARA_C2 = (a) / (2.0f * this->arm1_length * this->arm2_length);

    // SCARA position is undefined if abs(SCARA_C2) >=1
    if (SCARA_C2 > this->SCARA_undefined_max)
        SCARA_C2 = this->SCARA_undefined_max;
    else if (SCARA_C2 < -this->SCARA_undefined_min)
        SCARA_C2 = -this->SCARA_undefined_min;

    SCARA_Shoulder = (atan2f(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-acosf(a/(2.0f*sqrtf(SQ(SCARA_pos[X_AXIS])+SQ(SCARA_pos[Y_AXIS])))));
    SCARA_Elbow   = acosf(a/(2.0f*this->arm1_length+this->arm2_length));


    actuator_mm[ALPHA_STEPPER] = to_degrees(SCARA_Shoulder);          // Shoulder
    actuator_mm[BETA_STEPPER ] = to_degrees(SCARA_Elbow);             // Elbow
    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];                // No inverse kinematics on Z - Position to add bed offset?

}

void SCARASolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) {
    // Perform forward kinematics, and place results in cartesian_mm[]
	//Actuator[X_AXIS] is the angle of the shoulder in degree
	//Actuator[Y_AXIS] is the angle of the elbow in degree
	//Actuator[Z_AXIS] is the height of the X-Y plane in mm

    float x1, x2,
		  y1, y2,
          actuator_rad[2];

    actuator_rad[X_AXIS] = actuator_mm[X_AXIS]/(180.0F/3.14159265359f); //convert degree to radions
    actuator_rad[Y_AXIS] = actuator_mm[Y_AXIS]/(180.0F/3.14159265359f);

    x1 = this->arm1_length * cosf(actuator_rad[X_AXIS]);
    x2 = this->arm2_length * cosf(actuator_rad[X_AXIS] + actuator_rad[Y_AXIS]);
    y1 = this->arm1_length * sinf(actuator_rad[X_AXIS]);
    y2 = this->arm2_length * sinf(actuator_rad[X_AXIS] + actuator_rad[Y_AXIS]);

    cartesian_mm[X_AXIS] = ((x1 + x2) * this->SCARA_scaling_x) + this->SCARA_offset_x;
    cartesian_mm[Y_AXIS] = ((y1 + y2) * this->SCARA_scaling_y) + this->SCARA_offset_y;
    cartesian_mm[Z_AXIS] = actuator_mm[Z_AXIS];

    cartesian_mm[0] = ROUND(cartesian_mm[0], 7);
    cartesian_mm[1] = ROUND(cartesian_mm[1], 7);
    cartesian_mm[2] = ROUND(cartesian_mm[2], 7);
}

bool SCARASolution::set_optional(const arm_options_t& options) {

    arm_options_t::const_iterator i;

    i= options.find('T');          // Shoulder arm1 length
    if(i != options.end()) {
        arm1_length= i->second;

    }
    i= options.find('P');          // Elbow arm2 length
    if(i != options.end()) {
        arm2_length= i->second;
    }
    i= options.find('X');          // Home initial position X
    if(i != options.end()) {
        SCARA_offset_x= i->second;
    }
    i= options.find('Y');          // Home initial position Y
    if(i != options.end()) {
        SCARA_offset_y= i->second;
    }
    i= options.find('A');          // Scaling X_AXIS
    if(i != options.end()) {
        SCARA_scaling_x= i->second;
    }
    i= options.find('B');          // Scaling Y_AXIS
    if(i != options.end()) {
        SCARA_scaling_y= i->second;
    }
    //i= options.find('C');          // Scaling Z_AXIS
    //if(i != options.end()) {
    //    SCARA_scaling_z= i->second;
    //}
    i= options.find('D');          // Undefined min
    if(i != options.end()) {
        this->SCARA_undefined_min = i->second;
    }
    i= options.find('E');          // undefined max
    if(i != options.end()) {
        this->SCARA_undefined_max = i->second;
    }

    init();
    return true;
}

bool SCARASolution::get_optional(arm_options_t& options, bool force_all) {
    options['T']= this->arm1_length;
    options['P']= this->arm2_length;
    options['X']= this->SCARA_offset_x;
    options['Y']= this->SCARA_offset_y;
    options['A']= this->SCARA_scaling_x;
    options['B']= this->SCARA_scaling_y;
    // options['C']= this->SCARA_scaling_z;
    options['D']= this->SCARA_undefined_min;
    options['E']= this->SCARA_undefined_max;

    return true;
};
