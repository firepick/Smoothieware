#ifndef SCARASOLUTION_H
#define SCARASOLUTION_H
//#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class SCARASolution : public BaseSolution {
    public:
        SCARASolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) override;

        bool set_optional(const arm_options_t& options) override;
        bool get_optional(arm_options_t& options, bool force_all) override;

    private:
        void init();
        float to_degrees(float radians);

        float arm1_length;
        float arm2_length;
        float SCARA_offset_x;
        float SCARA_offset_y;
        float SCARA_scaling_x;
        float SCARA_scaling_y;
        float SCARA_undefined_min;
        float SCARA_undefined_max;
        float slow_rate;
};

#endif // SCARASOLUTION_H
