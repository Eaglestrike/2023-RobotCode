#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "string"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510582097494459230
#endif

namespace GeneralConstants
{
    const double g = 9.81;

    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380;
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE / STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * M_PI) / 60) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);

    const double CONE_M = 0.652;

}

namespace FieldConstants
{
    const double FIELD_WIDTH = 8.2296;
    const double FIELD_LENGTH = 16.4592;
    // const double LEFT_TAG_X = 1.05283; //41.45 in
    // const double MIDDLE_TAG_X = 2.72923; //107.45 in
    // const double RIGHT_TAG_X = 4.40563; //173.45;

    /**
     * Tag Positions in the form of
     * TAG_XY[TagID -1] = {x, y}
     * 
     * Long axis is the x, while short one is y
     * Driver faces along the x axis, apriltag too
     */
    const double TAG_XY[8][2] = {
        {15.513558, 1.071626}, //
        {15.513558, 2.748026},
        {15.513558, 4.424426},
        {16.178784, 6.749796},
        {0.36195, 6.749796},
        {1.02743, 4.424426},
        {1.02743, 2.748026},
        {1.02743, 1.071626}};

    const double BOTTOM_CONE_Y = 0.5128;
    const double TOP_CONE_Y = 4.983;
    const double BOTTOM_CUBE_Y = 1.0716;
    const double MID_CUBE_Y = 2.748026;
    const double TOP_CUBE_Y = 4.4244;

    const double TOP_MIDDLE_CONE_Y = 3.306826;
    const double BOTTOM_MIDDLE_CONE_Y = 2.1844;

    const double BLUE_SCORING_X = 1.8+0.068 + (0.0254 * 3);
    const double RED_SCORING_X = 14.74-0.068 - (0.0254 * 3);
    const double BLUE_PS_X = TAG_XY[3][0] - 0.7;
    const double RED_PS_X = TAG_XY[4][0] + 0.7;

    const double AUTO_DOCK_Y = 2.748;
    const double BLUE_AUTO_DOCK_X = 3.825875; // 2.412
    const double RED_AUTO_DOCK_X = 12.09675;  // 14.130

    const double BOTTOM_PIECE_Y = 0.919;
    const double BOTTOM_MID_PIECE_Y = 2.138;
    const double TOP_MID_PIECE_Y = 3.358;
    const double TOP_PIECE_Y = 4.577;

    const double BLUE_PIECE_X = 7.068;
    const double RED_PIECE_X = 9.474;
}