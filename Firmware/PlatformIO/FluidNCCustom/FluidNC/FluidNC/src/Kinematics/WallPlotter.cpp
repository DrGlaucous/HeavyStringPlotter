#include "WallPlotter.h"
#include "../Machine/MachineConfig.h"


#include <cmath>

namespace Kinematics {
    void WallPlotter::group(Configuration::HandlerBase& handler) {
        handler.item("left_axis", _left_axis);
        handler.item("left_anchor_x", _left_anchor_x);
        handler.item("left_anchor_y", _left_anchor_y);

        handler.item("right_axis", _right_axis);
        handler.item("right_anchor_x", _right_anchor_x);
        handler.item("right_anchor_y", _right_anchor_y);

        handler.item("segment_length", _segment_length);


        //handle spool stepping (disabling for now)
        /*
        handler.item("uses_spools", _uses_spools);
        if(_uses_spools)
        {

            handler.item("left_axis_steps_per_rotation", _l_steps_per_rotation);
            handler.item("left_axis_diameter_mm_min", _l_diam_min);
            handler.item("left_axis_diameter_mm_max", _l_diam_max);
            handler.item("left_axis_string_length", _l_string_max_len);

            _l_steps_per_mm_min = _l_steps_per_rotation / (PI * _l_diam_min);//this value should be bigger
            _l_steps_per_mm_max = _l_steps_per_rotation / (PI * _l_diam_max);//this value should be smaller

            //Debug stuff
            //log_msg_to(allChannels, "_segment_length: " << _segment_length);
            //log_msg_to(allChannels, "Steps per rotation: " << _l_steps_per_rotation);
            //log_msg_to(allChannels, "Diam_Min: " << _l_diam_min);
            //log_msg_to(allChannels, "Diam_Max: " << _l_diam_max);
            //log_msg_to(allChannels, "Left Min Steps_mm: " << _l_steps_per_mm_min);
            //log_msg_to(allChannels, "Left Max Steps_mm: " << _l_steps_per_mm_max);
            log_info("Left Min Steps_mm: " << _l_steps_per_mm_min);
            log_info("Left Max Steps_mm: " << _l_steps_per_mm_max);


            handler.item("right_axis_steps_per_rotation", _r_steps_per_rotation);
            handler.item("right_axis_diameter_mm_min", _r_diam_min);
            handler.item("right_axis_diameter_mm_max", _r_diam_max);
            handler.item("right_axis_string_length", _r_string_max_len);

            _r_steps_per_mm_min = _r_steps_per_rotation / (PI * _r_diam_min);
            _r_steps_per_mm_max = _r_steps_per_rotation / (PI * _r_diam_max);

            log_info("Right Min Steps_mm: " << _r_steps_per_mm_min);
            log_info("Right Max Steps_mm: " << _r_steps_per_mm_max);

        }
        */      
        
        //handle dynamic anchors
        handler.item("uses_dynamic_anchors", _uses_dynamic_anchors);
        if(_uses_dynamic_anchors)
        {
            //override anchor Y positions if we have this setting enabled (we can't override X positions yet; we don't know where they are)
            handler.item("anchor_config_distance", _anchor_config_distance);
            _left_anchor_y = _anchor_config_distance;
            _right_anchor_y = _anchor_config_distance;
        }

    }

    void WallPlotter::init() {
        log_info("Kinematic system: " << name());

        // We assume the machine starts at cartesian (0, 0, 0).
        // The motors assume they start from (0, 0, 0).
        // So we need to derive the zero lengths to satisfy the kinematic equations.
        xy_to_lengths(0, 0, zero_left, zero_right);
        last_motor_segment_end[0] = zero_left;
        last_motor_segment_end[1] = zero_right;
        auto n_axis               = config->_axes->_numberAxis;
        for (size_t axis = Z_AXIS; axis < n_axis; axis++) {
            last_motor_segment_end[axis] = 0.0;
        }

        init_position();
    }

    // Initialize the machine position
    void WallPlotter::init_position() {
        auto n_axis = config->_axes->_numberAxis;
        for (size_t axis = 0; axis < n_axis; axis++) {
            set_motor_steps(axis, 0);  // Set to zeros
        }
    }

    bool WallPlotter::canHome(AxisMask axisMask)
    {
        //(use G53? to move the machine, use home X to home left, use home y to home right)
        //if we have dynamic anchors, homing doesn't move the motors,
        //1: snaps home motor spool length to anchor_config_distance
        //uses that and 2nd motor spool length to calculate total distance between anchors (may not be correct, but is needed so we can home both axes)


        //sets _left_anchor_y to anchor_config_distance 
        //snaps machine coordiantes to to Y=0, X = -1/2 * total distance (assumes from lengths)

        //see processSettings.cpp for implementation in "$" code

        if(!_uses_dynamic_anchors)
        {
            log_error("This kinematic system cannot home");
        }
        else
        {
            log_error("Please use [$DMCE] and other commands to home system.");
        }
        return false;
    }

    //get coordinates and convert it into steps, I think this is a general-purpose exposure layer for the kenematics conversion
    void WallPlotter::transform_cartesian_to_motors(float* cartesian, float* motors) {
        xy_to_lengths(cartesian[0], cartesian[1], motors[_left_axis], motors[_right_axis]);
        //log_error("WallPlotter::transform_cartesian_to_motors is broken");
    }

    /*
      cartesian_to_motors() converts from cartesian coordinates to motor space.

      All linear motions pass through cartesian_to_motors() to be planned as mc_move_motors operations.

      Parameters:
        target = an n_axis array of target positions (where the move is supposed to go)
        pl_data = planner data (see the definition of this type to see what it is)
        position = an n_axis array of where the machine is starting from for this move
    */
    bool WallPlotter::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position)
    {

        //log_msg("TARGET: " <<target[0]<< "|"<<  target[1] << "|" << target[2] << "|"<< target[3] << "|"<< target[4] << "|"<< target[5] );
        //log_msg("POSITION: " << position[0] << "|"<< position[1] << "|" << position[2] << "|"<< position[3] << "|"<< position[4] << "|"<< position[5]  );

        //test to see if I can reset the positions
        //for(int p = 0; p < MAX_N_AXIS; ++p)
        //{
        //    target[p] = 0;
        //    position[p] = 30;
        //}


        if(config->_axes->_directMotorControl)//move in a cartesian manner if this setting is enabled
        {
            //no other variables to be updated, this function below (should?) keep track of motorspace positions
            int32_t* motorStepArray = get_motor_steps();
            float motor_mpos[MAX_N_AXIS];
            auto  a      = config->_axes;
            auto  n_axis = a ? a->_numberAxis : 0;
            for (size_t idx = 0; idx < n_axis; idx++) {
                motor_mpos[idx] = steps_to_mpos(motorStepArray[idx], idx);
            }

            //flip left motor (this is very screwy for some reason, with each command flipping the last one... I don't understand)
            //target[_left_axis] = -1;
            
            float x_tgt_cartesian, y_tgt_cartesian;//motor string lengths
            lengths_to_xy((0 - target[0]) + zero_left, (0 + target[1])+ zero_right, x_tgt_cartesian, y_tgt_cartesian);


            position[0] = x_tgt_cartesian;
            position[1] = y_tgt_cartesian;

            //deal with the motor segmentations that the other system uses
            copyAxes(last_motor_segment_end, position);

            //log_info("2-Last motor coordinates: left = " << steps_to_mpos(motorStepArray[_left_axis], _left_axis) << "right = " << steps_to_mpos(motorStepArray[_right_axis], _right_axis));
            //log_info("Last motor coordinates: left = " << motor_mpos[_left_axis] << "right = " << motor_mpos[_right_axis]);
            //float xy[2];
            //lengths_to_xy(motor_mpos[_left_axis], motor_mpos[_right_axis], xy[0], xy[1]);
            //log_msg("Nu-X: "<< xy[0] << " Nu-Y: " << xy[1]);
            return mc_move_motors(target, pl_data);
        }
        else
        {
            float    dx, dy, dz;     // segment distances in each cartesian axis
            uint32_t segment_count;  // number of segments the move will be broken in to.

            auto n_axis = config->_axes->_numberAxis;

            float total_cartesian_distance = vector_distance(position, target, n_axis);
            if (total_cartesian_distance == 0) {
                //TEST
                //log_msg("Not moving, total distance is 0");
                //mc_move_motors(target, pl_data); //(leaving this enabled really screws with plotter operation)
                return true;
            }

            float cartesian_feed_rate = pl_data->feed_rate;

            // calculate the total X,Y axis move distance
            // Z axis is the same in both coord systems, so it does not undergo conversion
            float xydist = vector_distance(target, position, 2);  // Only compute distance for both axes. X and Y
            // Segment our G1 and G0 moves based on yaml file. If we choose a small enough _segment_length we can hide the nonlinearity
            segment_count = xydist / _segment_length;
            if (segment_count < 1) {  // Make sure there is at least one segment, even if there is no movement
                // We need to do this to make sure other things like S and M codes get updated properly by
                // the planner even if there is no movement??
                segment_count = 1;
            }
            float cartesian_segment_length = total_cartesian_distance / segment_count;

            // Calc length of each cartesian segment - the same for all segments
            float cartesian_segment_components[n_axis];
            for (size_t axis = X_AXIS; axis < n_axis; axis++) {
                cartesian_segment_components[axis] = (target[axis] - position[axis]) / segment_count;
            }

            float cartesian_segment_end[n_axis];
            copyAxes(cartesian_segment_end, position);

            // Calculate desired cartesian feedrate distance ratio. Same for each seg.
            for (uint32_t segment = 1; segment <= segment_count; segment++) {
                // calculate the cartesian end point of the next segment
                for (size_t axis = X_AXIS; axis < n_axis; axis++) {
                    cartesian_segment_end[axis] += cartesian_segment_components[axis];
                }

                // Convert cartesian space coords to motor space
                float motor_segment_end[n_axis];
                xy_to_lengths(cartesian_segment_end[X_AXIS], cartesian_segment_end[Y_AXIS], motor_segment_end[0], motor_segment_end[1]);
                for (size_t axis = Z_AXIS; axis < n_axis; axis++) {
                    motor_segment_end[axis] = cartesian_segment_end[axis];
                }

    #ifdef USE_CHECKED_KINEMATICS
                // Check the inverse computation.
                float cx, cy;
                lengths_to_xy(motor_segment_end[0], motor_segment_end[1], cx, cy);

                if (abs(cartesian_segment_end[X_AXIS] - cx) > 0.1 || abs(cartesian_segment_end[Y_AXIS] - cy) > 0.1) {
                    // FIX: Produce an alarm state?
                }
    #endif
                // Adjust feedrate by the ratio of the segment lengths in motor and cartesian spaces,
                // accounting for all axes
                if (!pl_data->motion.rapidMotion) {  // Rapid motions ignore feedrate. Don't convert.
                                                    // T=D/V, Tcart=Tmotor, Dcart/Vcart=Dmotor/Vmotor
                                                    // Vmotor = Dmotor*(Vcart/Dcart)
                    float motor_segment_length = vector_distance(last_motor_segment_end, motor_segment_end, n_axis);
                    pl_data->feed_rate         = cartesian_feed_rate * motor_segment_length / cartesian_segment_length;
                }

                // TODO: G93 pl_data->motion.inverseTime logic?? Does this even make sense for wallplotter?

                // Remember the last motor position so the length can be computed the next time
                copyAxes(last_motor_segment_end, motor_segment_end);

                // Initiate motor movement with converted feedrate and converted position
                // mc_move_motors() returns false if a jog is cancelled.
                // In that case we stop sending segments to the planner.
                // Note that the left motor runs backward.
                // TODO: It might be better to adjust motor direction in .yaml file by inverting direction pin??
                float cables[n_axis];
                cables[0] = 0 - (motor_segment_end[0] - zero_left);
                cables[1] = 0 + (motor_segment_end[1] - zero_right);
                for (size_t axis = Z_AXIS; axis < n_axis; axis++) {
                    cables[axis] = cartesian_segment_end[axis];
                }

                //disable this, the math is too complicated for this scope
                //just before passing  values off to the motors, update steps per mm
                //if(_uses_spools)
                //    UpdateStepsPerMM(motor_segment_end[0], motor_segment_end[1]);

                if (!mc_move_motors(cables, pl_data)) {
                    // TODO fixup last_left last_right?? What is position state when jog is cancelled?
                    return false;
                }
            }
            return true;
        }

    }

    /*
      The status command uses motors_to_cartesian() to convert
      your motor positions to cartesian X,Y,Z... coordinates.

      Convert the n_axis array of motor positions to cartesian in your code.
    */
    void WallPlotter::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        // The motors start at zero, but effectively at zero_left, so we need to correct for the computation.
        // Note that the left motor runs backward.
        // TODO: It might be better to adjust motor direction in .yaml file by inverting direction pin??

        float absolute_x, absolute_y;
        lengths_to_xy((0 - motors[_left_axis]) + zero_left, (0 + motors[_right_axis]) + zero_right, absolute_x, absolute_y);

        cartesian[X_AXIS] = absolute_x;
        cartesian[Y_AXIS] = absolute_y;
        for (size_t axis = Z_AXIS; axis < n_axis; axis++) {
            cartesian[axis] = motors[axis];
        }
        // Now we have numbers that if fed back into the system should produce the same values.
    }

    /*
    Kinematic equations

    See http://paulbourke.net/geometry/circlesphere/

    First calculate the distance d between the center of the circles. d = ||P1 - P0||.

    If d > r0 + r1 then there are no solutions, the circles are separate.
    If d < |r0 - r1| then there are no solutions because one circle is contained within the other.
    If d = 0 and r0 = r1 then the circles are coincident and there are an infinite number of solutions.
    Considering the two triangles P0P2P3 and P1P2P3 we can write
    a2 + h2 = r02 and b2 + h2 = r12
    Using d = a + b we can solve for a,

    a = (r02 - r12 + d2 ) / (2 d)
    It can be readily shown that this reduces to r0 when the two circles touch at one point, ie: d = r0 ± r1

    Solve for h by substituting a into the first equation, h2 = r02 - a2

    h = sqrt(r02 - a2)
    */

    void WallPlotter::lengths_to_xy(float left_length, float right_length, float& x, float& y) {
        float distance  = _right_anchor_x - _left_anchor_x;
        float distance2 = distance * distance;

        // The lengths are the radii of the circles to intersect.
        float left_radius  = left_length;
        float left_radius2 = left_radius * left_radius;

        float right_radius  = right_length;
        float right_radius2 = right_radius * right_radius;

        // Compute a and h.
        float a  = (left_radius2 - right_radius2 + distance2) / (2 * distance);
        float a2 = a * a;
        float h  = sqrtf(left_radius2 - a2);

        // Translate to absolute coordinates.
        x = _left_anchor_x + a;
        y = _left_anchor_y - h;  // flip
    }

    void WallPlotter::xy_to_lengths(float x, float y, float& left_length, float& right_length) {
        // Compute the hypotenuse of each triangle.

        float left_dy = _left_anchor_y - y;
        float left_dx = _left_anchor_x - x;
        left_length   = hypot_f(left_dx, left_dy);

        float right_dy = _right_anchor_y - y;
        float right_dx = _right_anchor_x - x;
        right_length   = hypot_f(right_dx, right_dy);
    }



    //after-market functions
    //custom for hanging plot gantry $DMC=[0/1] (direct motor control)
    //Calibrate Location (hanging plotter exclusive) $CGL=[0/1] (calibrate gondola location)

    void WallPlotter::CalibrateEdge(bool calibrateRight)//0 for left side, 1 for right
    {
        //do a variable flip-flop depending on what side we're calibrating
        int mainAxis = _left_axis;
        float* mainAxis_y = &_left_anchor_y;
        float* mainAxis_x = &_left_anchor_x;

        int minorAxis = _right_axis;
        float* minorAxis_y = &_right_anchor_y;
        float* minorAxis_x = &_right_anchor_x;

        if(calibrateRight)
        {
            mainAxis = _right_axis;
            mainAxis_y = &_right_anchor_y;
            mainAxis_x = &_left_anchor_x;

            minorAxis = _left_axis;
            minorAxis_y = &_left_anchor_y;
            minorAxis_x = &_left_anchor_x;
        }


        //motor position 0,0 is 0,0 cartesian, regardless of how much string is spooled out
        //this means when updating, motor position will either be 0,0 for left, or
        //0+extratravelLen, 0-extratravelLen for right

        //get raw motor position array
        int32_t* motorStepArray = get_motor_steps(); //get steps
        float mposArray[MAX_N_AXIS]; //init position array
        auto  a      = config->_axes;
        auto  n_axis = a ? a->_numberAxis : 0; //if there is an axes object, use that number. if not, use 0
        for (size_t idx = 0; idx < n_axis; idx++) {
            mposArray[idx] = steps_to_mpos(motorStepArray[idx], idx);//convert each length into mm length
        }

        mposArray[_left_axis] *= -1;//left axis is backward


        //snap length of vertical string to calibration length
        //mposArray[mainAxis] = _anchor_config_distance;
        //if(mposArray[minorAxis] < _anchor_config_distance)
        //    mposArray[minorAxis] = _anchor_config_distance + 1;//account for invalid triangles (in case we're just starting in), because in order for the one side to be valid, this must be at least the same




        //get distance and place anchor X accordingly
        float anchorDist = sqrt(pow((mposArray[minorAxis] + _anchor_config_distance), 2) - pow(_anchor_config_distance, 2)   );
        

        //if its the left side, our location will be 0. if its the right, our location will be the full distance (for simplicity for now)  
        //update anchor locations
        *mainAxis_y = _anchor_config_distance;
        *minorAxis_y = _anchor_config_distance;
        _left_anchor_x = 0;
        _right_anchor_x = anchorDist;

        //update origin and raw motor locations
        if(calibrateRight)
        {
            //xy_to_lengths(anchorDist, 0, zero_left, zero_right);//right side is the origin + anchor space on the X axis
            //set_motor_steps(_left_axis, mpos_to_steps(mposArray[_left_axis] + _anchor_config_distance, _left_axis) * -1);
            set_motor_steps(_right_axis, mpos_to_steps(-1 * mposArray[minorAxis], _right_axis));//will be beginning state - however far it took to get here.
            
            
            //reset planner coordiantes, too
            gc_state.position[0] = 0; //x
            gc_state.position[1] = anchorDist; //y
        }
        else
        {
            xy_to_lengths(0, 0, zero_left, zero_right);//left side is the origin
            set_motor_steps(_left_axis, 0);
            set_motor_steps(_right_axis, 0);

            gc_state.position[0] = 0; //x
            gc_state.position[1] = 0; //y

        }


        //update raw motor locations with new coordinates

        //for (size_t axis = 0; axis < n_axis; axis++)
        //    set_motor_steps(axis, mpos_to_steps(mposArray[axis], axis));


        log_msg("Updated "<< calibrateRight <<" Motor location:\nMotor Length Left:" << mposArray[_left_axis] <<
        "\nMotor Length Right: " << mposArray[_right_axis] <<
        "\nAnchor Distance: " << anchorDist <<
        "\nLX: " << _left_anchor_x <<
        " LY: " << _left_anchor_y <<
        "\nRX: " << _right_anchor_x <<
        " RY: " << _right_anchor_y
        );
        float xy[2];
        lengths_to_xy(mposArray[_left_axis], mposArray[_right_axis], xy[0], xy[1]);
        log_msg("Nu-X: "<< xy[0] << " Nu-Y: " << xy[1]);

    }

    //intended general purpose callable to do different things based on what each kinematic section needs
    bool WallPlotter::run_setting_process(int settingNo, int actionNo)
    {
        log_msg("Settings Ran: " << settingNo << " || " << actionNo);


        switch(settingNo)
        {
            default:
                log_error("Invalid Setting")
                return false;
                break;
            case 1://home either of the coordinates
            {
                if(!_uses_dynamic_anchors)
                    {
                        log_error("Dynamic anchors are disabled in config file. Cannot run command.");
                        return false;
                    }

                //settingNo = 1 (config points)
                //actionNo = 0 for left, 1 for right
                if(actionNo == 3)
                    {
                        int32_t* motorStepArray = get_motor_steps(); //get steps
                        float mposArray[MAX_N_AXIS]; //init position array
                        auto  a      = config->_axes;
                        auto  n_axis = a ? a->_numberAxis : 0; //if there is an axes object, use that number. if not, use 0
                        for (size_t idx = 0; idx < n_axis; idx++) {
                            mposArray[idx] = steps_to_mpos(motorStepArray[idx], idx);//convert each length into mm length
                        }
                        log_msg("Current motor Left: " << mposArray[_left_axis] * -1<< "\nCurrent motor Right: " << mposArray[_right_axis]);
                    }
                else
                    CalibrateEdge(actionNo);
                break;
            }
            case 2:
            {
                //left anchor is over 0,0.
                //right anchor is at the anchor distance point
                _right_anchor_x = actionNo;
                init();//reset everything with this new value
                log_msg("Set motor distance: " << _right_anchor_x);
                break;
            }

        };



        return true;
    }


    //UNFINISHED
    //Looks at current motor spool length and determines how many steps it takes to move one mm,
    //returns a ratio of the movement (say, 0.9 or 1.1) actually chaning steps/mm will mess with the machine's ability to tell where it's at
    /*
    void WallPlotter::UpdateStepsPerMM(float left_length, float right_length)
    {
        //_l_steps_per_mm_min should be larger (more steps per mm), when the string is all the way out (smaller spool diameter)
        float newStepsMM;
        //eek by the integer problem by multiplying everything by 100 for now (may make better function later)
        newStepsMM = map(left_length * 100, 0, _l_string_max_len * 100, _l_steps_per_mm_max * 100, _l_steps_per_mm_min * 100) / 100.0;
        
        
        config->_axes->_axis[_left_axis]->_stepsPerMm = newStepsMM;

        newStepsMM = map(right_length * 100, 0, _r_string_max_len * 100, _r_steps_per_mm_max * 100, _r_steps_per_mm_min * 100) / 100.0;
        config->_axes->_axis[_right_axis]->_stepsPerMm = newStepsMM;

        //DEBUG
        //log_msg_to(allChannels, "STEP/MM UPDATE DUMP:" << left_length << "||" << right_length << "|--|" << config->_axes->_axis[_left_axis]->_stepsPerMm<< "|--|" << config->_axes->_axis[_right_axis]->_stepsPerMm);
        //log_msg_to(allChannels, "STEP/MM UPDATE DUMP:" << left_length << "||" << _l_string_max_len << "|-|" << _l_steps_per_mm_max << "|--|" << _l_steps_per_mm_min << "==" << config->_axes->_axis[_left_axis]->_stepsPerMm);
        //[MSG: STEP/MM UPDATE DUMP:673.486||4572|-|0.318|--|0.318==0.000]


    }*/


    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<WallPlotter> registration("WallPlotter");
    }
}
