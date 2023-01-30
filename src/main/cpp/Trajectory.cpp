#include "Trajectory.h"

std::tuple<std::array<units::radian_t, FFUConstants::trajectory_size>, 
    std::array<units::radians_per_second_t, FFUConstants::trajectory_size>, 
    std::array<units::radians_per_second_squared_t, FFUConstants::trajectory_size>> 
Trajectory::calcTraj(units::radian_t start_theta, units::radian_t end_theta, units::second_t time) {
    units::radian_t dist = end_theta - start_theta;
    double A = 0.5;
    units::radians_per_second_t h = dist / ((A/2) * time + (1-A) * time);

    auto thetas = get_thetas(h, A, time, start_theta);
    auto omegas = get_omegas(h, A, time);
    auto alphas = get_alphas(h, A, time);

    return {thetas, omegas, alphas};
}

std::array<units::radian_t, FFUConstants::trajectory_size> Trajectory::get_thetas(units::radians_per_second_t h, double A, units::second_t time, units::radian_t start_theta) {
    std::array<units::radian_t, FFUConstants::trajectory_size> areas;
    units::radian_t area1 = 0_rad;
    units::radian_t area2 = 0_rad;
    units::radian_t area3 = 0_rad;
    units::second_t triangle_1_end = (A/2) * time;
    units::second_t triangle_2_start = time - triangle_1_end;
    for (int i=0; i<FFUConstants::trajectory_size; i++) {
        units::second_t curr_t = (i/(FFUConstants::trajectory_size-1)) * time;
        double div;
        if (curr_t < triangle_1_end) {
            div = curr_t / triangle_1_end;
        } else {
            div = 1;
        }
        area1 = ((div * h) * (div * triangle_1_end)) / 2;
        if (curr_t > triangle_1_end) {
            units::second_t length;
            if (curr_t < triangle_2_start) {
                length = curr_t - triangle_1_end;
            } else {
                length = triangle_2_start - triangle_1_end;
            }
            area2 = h * length;
            if (curr_t > triangle_2_start) {
                double div;
                if (curr_t < time) {
                    div = (curr_t - triangle_2_start) / (time - triangle_2_start);
                } else {
                    div = 1;
                }
                area3 = (h + h * (1-div)) * (time - triangle_2_start) / 2;
            }
        }
        units::radian_t area = area1 + area2 + area3;
        areas[i] = area;
    }
    return areas;
}

std::array<units::radians_per_second_t, FFUConstants::trajectory_size> Trajectory::get_omegas(units::radians_per_second_t h, double A, units::second_t time) {
    std::array<units::radians_per_second_t, FFUConstants::trajectory_size> vals;
    units::second_t triangle_1_end = (A/2) * time;
    units::second_t triangle_2_start = time - triangle_1_end;
    for (int i=0; i<FFUConstants::trajectory_size; i++) {
        units::second_t curr_t = (i/(FFUConstants::trajectory_size-1)) * time;
        double div;
        units::radians_per_second_t val;
        if (curr_t < triangle_1_end) {
            div = curr_t / triangle_1_end;
            val = div * h;
        } else if (curr_t >= triangle_1_end && curr_t < triangle_2_start) {
            val = h;
        } else {
            if (curr_t < time) {
                div = (curr_t - triangle_2_start) / (time - triangle_2_start);
            } else {
                div = 1;
            }
            val = (1-div) * h;
        }
        vals[i] = val;
    }
    return vals;
}

std::array<units::radians_per_second_squared_t, FFUConstants::trajectory_size> Trajectory::get_alphas(units::radians_per_second_t h, double A, units::second_t time) {
    std::array<units::radians_per_second_squared_t, FFUConstants::trajectory_size> vals;
    units::second_t triangle_1_end = (A/2) * time;
    units::second_t triangle_2_start = time - triangle_1_end;
    for (int i=0; i<FFUConstants::trajectory_size; i++) {
        units::second_t curr_t = (i/(FFUConstants::trajectory_size-1)) * time;
        units::radians_per_second_squared_t val;
        if (curr_t < triangle_1_end) {
            val = h / triangle_1_end;
        } else if (curr_t >= triangle_1_end && curr_t < triangle_2_start) {
            val = 0_rad_per_s_sq;
        } else {
            val = -h / triangle_1_end;
        }
        vals[i] = val;
    }
    return vals;
}