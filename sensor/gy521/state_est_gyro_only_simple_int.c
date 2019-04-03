struct state state_est_GOSI(int16_t GyX, int16_t GyY, int16_t GyZ) {
    // integrate over time
	struct state s_GOSI = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize state variables to 0
    state[0] += double(GyX) * gy_conv_factor * double(this_time_step) / 1000.0;
    state[1] += double(GyY) * gy_conv_factor * double(this_time_step) / 1000.0;
    state[2] += double(GyZ) * gy_conv_factor * double(this_time_step) / 1000.0;
}