#include "triballcontrol.h"


pros::Task* cataTask = nullptr;
triballControl::Mode mode = triballControl::Mode::reloading;
triballControl::Mode request = triballControl::Mode::idle;



void triballupdate(){
    
    opti.set_led_pwm(100);
    while(true){

        //optical
        triballControl::hue = opti.get_hue();
        if(triballControl::hue>= 80 && triballControl::hue <= 100){
            triballControl::detected = 1;
        } else{
            triballControl::detected = 0;
        }

        //slapper
        auto isTouching = limit.get_value();
        if (request < mode) mode = request;

        switch(mode){

            //slapper reload
            case triballControl::reloading:{
                //move slapper down
                slapper.move(127);
                if(isTouching){
                    mode = triballControl::idle;
                    break;
                }

            }

            //slapper idle
            case triballControl::idle:{
                slapper.set_brake_mode(MOTOR_BRAKE_HOLD);
                slapper.move(0);
                slapper.move(0);
                break;
            }

            //slapper fire
            case triballControl::fire:{
                slapper.set_brake_mode(MOTOR_BRAKE_COAST);
                slapper.move(127);
                mode = triballControl::reloading;
            }
        }
        pros::delay(10);
    }
}


void intakeandouttake(){
    
    if(!(pros::competition::is_autonomous())){
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            !triballControl::r1pressed;
            if(triballControl::r1pressed){
                intake.move(127);
                if(triballControl::detected == 1){
                    intake.move(0);
            
                }  
            } else{
                intake.move(0);
            }
            
        }
       
    }
}
void triballControl::initialize() {
    // prevent tomfoolery with the task being initialized twice
    if (cataTask == nullptr) cataTask = new pros::Task([=]() { triballupdate(); });
}

void triballControl::fireCata(){
    request = triballControl::Mode::fire;
}

