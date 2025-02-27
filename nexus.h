#ifndef NEXUS_LIBRARY_H
#define NEXUS_LIBRARY_H
#include <kipr/wombat.h>
#include <string>

// Robot structure
int second_right_motor_port = 0;              // Second right motor port. Used for 4 wheel drive.
int second_left_motor_port = 0;               // Second left motor port.
bool four_wheel_drive = false;                // If true, drive functions will control the additional motor ports

// Parameters
bool shut_down_timer_enabled = false;         // Will the robot automatically stop at the time limit
bool wait_for_light_trigger = false;          // Will the robot use the standard wait_for_light trigger at tournaments
int light_sensor_port = 0;                    // Port for the light sensor
int left_motor_port = 1;                      // Left drive motor port
int right_motor_port = 0;                     // Right drive motor port

// Constants
const int BASE_SPEED = 500;                   // Base motor speed for turning (higher for better speed control)
const int CORRECTION_FACTOR = 2;              // Proportional correction factor
const int INTEGRAL_LIMIT = 10;                // Limit for integral windup
const float DEGREE_PER_COUNT = 250.0 / 512.0; // Degrees per gyroscope count

// Function Prototypes
inline void load_parameters();                // Loads runtime parameters
inline void os_system(const std::string& command); // Runs system commands on the host
inline void drive_straight(int mpc);          // Abstract drive straight function for the robot
inline void main_code();                      // Main code execution logic
inline void turn_degrees(float degrees);      // Turns the robot by a specified number of degrees
inline double calibrate_gyro_z();             // Calibrate the gyro.
inline void drive_with_gyro(int speed, double time, double bias); //Drive using the gyro at speed for time with bias
inline void init_camera(const char* config_name); //Init the camera using the config name
inline void center_on_object(int channel);    // Use the camera to center on object 0 on channel.
inline void debug_gyro();                     // Debugging function for gyro z-axis
inline void nexus_init();                     // Init function. Must be called at the start of your program for this to work properly.

// Runtime values
bool camera_init = false;

#endif // NEXUS_LIBRARY_H

#ifndef NEXUS_LIBRARY_IMPL_H
#define NEXUS_LIBRARY_IMPL_H

// Function Definitions
inline void load_parameters(){
    if (shut_down_timer_enabled) {
        shut_down_in(119);
    }
    if (wait_for_light_trigger) {
        wait_for_light(light_sensor_port);
    }
}

inline void os_system(const std::string& command) {
    // Execute the system command
    int ret_code = system(command.c_str());
    if (ret_code != 0) {
        printf("An error has occurred on os_system: %d", ret_code);
    }
}

// Gyro functions
double calibrate_gyro_z(){
    int i, gz, iters = 1000;
    double bias, total = 0.0;

    for (i = 0; i < iters; i++) {
        gz = gyro_z();
        total += gz;
        msleep(1);
        printf("Gyro Z: %d\n", gyro_z());
    }

    bias = total / iters;
    printf("New Bias: %f\n", bias);

    return bias;
}

inline void drive_with_gyro(int speed, double time, double bias){
    double start_time = seconds();
    double delta, dev = 0;

    while ((seconds() - start_time) < time) {
        /* You may need to change this factor
         * depending on how sensitive your gyro is
         */
        delta = dev / 5;
        mav(right_motor_port, -((-1 * speed) - delta));
        mav(left_motor_port, speed + delta);
        if(four_wheel_drive){
            mav(second_right_motor_port, -((-1 * speed) - delta));
            mav(second_left_motor_port, speed + delta);
        }
        msleep(10);
        dev += gyro_z() - bias;
    }

    ao();
}

inline void turn_degrees(float degrees) {
    float current_angle = 0.0;   // Current angle (in degrees)
    float target_angle = degrees; // Target angle to reach
    float angular_velocity = 0.0; // Angular velocity in degrees per second

    float last_time = seconds(); // Track the time

    while (1) {
        short int current_GZ_raw = gyro_z();
        angular_velocity = (float)current_GZ_raw * DEGREE_PER_COUNT;

        float current_time = seconds();
        float delta_time = current_time - last_time;

        current_angle += angular_velocity * delta_time;
        last_time = current_time; // Update last time for next iteration

        if (fabs(current_angle) >= fabs(target_angle)) {
            break;  // Exit the loop if the target angle is reached
        }

        int left_speed, right_speed;
        if (target_angle > 0) { // Turning right
            left_speed = BASE_SPEED;
            right_speed = -BASE_SPEED;
        } else { // Turning left
            left_speed = -BASE_SPEED;
            right_speed = BASE_SPEED;
        }

        mav(left_motor_port, left_speed);
        mav(right_motor_port, right_speed);
        if(four_wheel_drive){
            mav(second_left_motor_port, left_speed);
            mav(second_right_motor_port, right_speed);
        }
    }

    ao();
}

inline void drive_straight(int mpc){
	clear_motor_position_counter(left_motor_port);
    clear_motor_position_counter(right_motor_port);
    while((gmpc(left_motor_port) + gmpc(right_motor_port)) < mpc){
        mav(left_motor_port, 500 - 30);
    	mav(right_motor_port, 500);
        if(four_wheel_drive){
            mav(second_left_motor_port, 500 - 30);
            mav(second_right_motor_port, 500);
        }
    }
}

inline void init_camera(const char* config_name){
    if (camera_load_config(config_name)) {
        printf("Config loaded\n");
    } else {
        printf("Config failed to load\n");
    }
    printf("Channel count is %d\n", get_channel_count());
    camera_update();
    camera_init = true;
}
inline void center_on_object(int channel){
    if(!camera_init){
    	return;
    }
	printf("Camera count of object is %d\n", get_object_count(channel));
    if(get_object_count(channel) == 0){
    	printf("No object found.\n");
        return;
    }
    while (1) {
        camera_update();
        if(get_object_count(0) > 0){
            int target_x = get_object_center_x(0, 0);

            if (target_x > 190) {
                mav(right_motor_port, 200);
                mav(left_motor_port, -200);
                if(four_wheel_drive){
                	mav(second_right_motor_port, 200);
                	mav(second_left_motor_port, -200);
                }
            } else if (target_x < 130) {
                mav(right_motor_port, -200);
                mav(left_motor_port, 200);
                if(four_wheel_drive){
                	mav(second_right_motor_port, -200);
                	mav(second_left_motor_port, 200);
                }
            } else{
                break;
                return;
            }
        } else{
        	printf("No object found.\n");
        }
    }
}

inline void debug_gyro() {
    short int initial_GZ = gyro_z();

    printf("Initial gyro z-axis value: %d\n", initial_GZ);

    while (!side_button()) {}
    short int current_GZ = gyro_z();
    printf("Current gyro z-axis value: %d\n", current_GZ);
    msleep(500);
}

inline void nexus_init(){
	load_parameters();
    printf("Nexus params loaded\n");
    printf("Camera loaded with code %d\n", camera_open_device_model_at_res(0, BLACK_2017, MED_RES));
}

#endif // NEXUS_LIBRARY_IMPL_H
