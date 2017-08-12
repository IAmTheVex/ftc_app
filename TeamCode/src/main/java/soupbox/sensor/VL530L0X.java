package soupbox.sensor;

import com.qualcomm.robotcore.hardware.I2cDevice;

public class VL530L0X extends I2CUtil {

    private int stop_variable;
    private int measurement_timing_budget_us;

    public VL530L0X(I2cDevice raw) {
        super(raw);
    }

    public void startContinuous(int period_ms)
    {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stop_variable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        if (period_ms != 0)
        {
            // continuous timed mode

            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

            int osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

            if (osc_calibrate_val != 0)
            {
                period_ms *= osc_calibrate_val;
            }

            writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

            writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
        }
        else
        {
            // continuous back-to-back mode
            writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
        }
    }

    public void stopContinuous()
    {
        writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, 0x00);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
    }

    public static class TimeoutException extends Exception {}

    public int readRangeContinuousMillimeters() throws TimeoutException
    {
        startTimeout();
        while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
            if (checkTimeoutExpired())
            {
                throw new TimeoutException();
            }
        }

        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        int range = readReg16Bit(RESULT_RANGE_STATUS + 10);

        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        return range;
    }

    public boolean setSignalRateLimit(float limit_Mcps)
    {
        if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (int)(limit_Mcps * (1 << 7)));
        return true;
    }

    // Get the return signal rate limit check value in MCPS
    public float getSignalRateLimit()
    {
        return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
    }

    public int readRangeSingleMillimeters() throws TimeoutException
    {
        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        writeReg(0x91, stop_variable);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        writeReg(SYSRANGE_START, 0x01);

        // "Wait until start bit has been cleared"
        startTimeout();
        while (0 != (readReg(SYSRANGE_START) & 0x01))
        {
            if (checkTimeoutExpired())
            {
                throw new TimeoutException();
            }
        }

        return readRangeContinuousMillimeters();
    }

    private long timeout_start_ms;
    private long timeout_max  = 0;
    private void startTimeout() {
        timeout_start_ms = System.currentTimeMillis();
    }

    private boolean checkTimeoutExpired() {
        if (0 == timeout_max) { return false; }
        long d = System.currentTimeMillis() - timeout_start_ms;
        return d > timeout_max;
    }

    public void setTimeoutMax(long t) { timeout_max = t; }


    private SequenceStepEnables getSequenceStepEnables()
    {
        int sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

        SequenceStepEnables enables = new SequenceStepEnables();

        enables.tcc          = 0 != ((sequence_config >> 4) & 0x1);
        enables.dss          = 0 != ((sequence_config >> 3) & 0x1);
        enables.msrc         = 0 != ((sequence_config >> 2) & 0x1);
        enables.pre_range    = 0 != ((sequence_config >> 6) & 0x1);
        enables.final_range  = 0 != ((sequence_config >> 7) & 0x1);

        return enables;
    }

    private static enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };


    private int decodeVcselPeriod(int reg_val) {
        return (((reg_val) + 1) << 1);
    }

    private int getVcselPulsePeriod(vcselPeriodType type)
    {
        if (type == vcselPeriodType.VcselPeriodPreRange)
        {
            return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else if (type == vcselPeriodType.VcselPeriodFinalRange)
        {
            return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else { return 255; }
    }

    private int decodeTimeout(int reg_val)
    {
        // format: "(LSByte * 2^MSByte) + 1"
        return ((reg_val & 0x00FF) <<
                ((reg_val & 0xFF00) >> 8)) + 1;
    }

    private int calcMacroPeriod(int vcsel_period_pclks) {
        return ((((int)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
    }


    private int timeoutMclksToMicroseconds(int timeout_period_mclks, int vcsel_period_pclks)
    {
        int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
    }

    private SequenceStepTimeouts getSequenceStepTimeouts(final SequenceStepEnables enables)
    {
        SequenceStepTimeouts timeouts = new SequenceStepTimeouts();

        timeouts.pre_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange);

        timeouts.msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        timeouts.msrc_dss_tcc_us =
                timeoutMclksToMicroseconds(timeouts.msrc_dss_tcc_mclks,
                        timeouts.pre_range_vcsel_period_pclks);

        timeouts.pre_range_mclks =
                decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        timeouts.pre_range_us =
                timeoutMclksToMicroseconds(timeouts.pre_range_mclks,
                        timeouts.pre_range_vcsel_period_pclks);

        timeouts.final_range_vcsel_period_pclks = getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange);

        timeouts.final_range_mclks =
                decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

        if (enables.pre_range)
        {
            timeouts.final_range_mclks -= timeouts.pre_range_mclks;
        }

        timeouts.final_range_us =
                timeoutMclksToMicroseconds(timeouts.final_range_mclks,
                        timeouts.final_range_vcsel_period_pclks);

        return timeouts;
    }


    private int getMeasurementTimingBudget() {
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        final int StartOverhead     = 1910; // note that this is different than the value in set_
        final int EndOverhead        = 960;
        final int MsrcOverhead       = 660;
        final int TccOverhead        = 590;
        final int DssOverhead        = 690;
        final int PreRangeOverhead   = 660;
        final int FinalRangeOverhead = 550;

        // "Start and end overhead times always present"
        int budget_us = StartOverhead + EndOverhead;

        enables = getSequenceStepEnables();
        timeouts = getSequenceStepTimeouts(enables);

        if (enables.tcc)
        {
            budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss)
        {
            budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        }
        else if (enables.msrc)
        {
            budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range)
        {
            budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range)
        {
            budget_us += (timeouts.final_range_us + FinalRangeOverhead);
        }

        measurement_timing_budget_us = budget_us; // store for internal reuse
        return budget_us;
    }

    // @todo
    private boolean setMeasurementTimingBudget(int budget_us) {
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        final int StartOverhead      = 1320; // note that this is different than the value in get_
        final int EndOverhead        = 960;
        final int MsrcOverhead       = 660;
        final int TccOverhead        = 590;
        final int DssOverhead        = 690;
        final int PreRangeOverhead   = 660;
        final int FinalRangeOverhead = 550;

        final int MinTimingBudget = 20000;

        if (budget_us < MinTimingBudget) { return false; }

        int used_budget_us = StartOverhead + EndOverhead;

        enables = getSequenceStepEnables();
        timeouts = getSequenceStepTimeouts(enables);

        if (enables.tcc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss)
        {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        }
        else if (enables.msrc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range)
        {
            used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range)
        {
            used_budget_us += FinalRangeOverhead;

            // "Note that the final range timeout is determined by the timing
            // budget and the sum of all other timeouts within the sequence.
            // If there is no room for the final range timeout, then an error
            // will be set. Otherwise the remaining time will be applied to
            // the final range."

            if (used_budget_us > budget_us)
            {
                // "Requested timeout too big."
                return false;
            }

            int final_range_timeout_us = budget_us - used_budget_us;

            // set_sequence_step_timeout() begin
            // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

            // "For the final range timeout, the pre-range timeout
            //  must be added. To do this both final and pre-range
            //  timeouts must be expressed in macro periods MClks
            //  because they have different vcsel periods."

            int final_range_timeout_mclks =
                    timeoutMicrosecondsToMclks(final_range_timeout_us,
                            timeouts.final_range_vcsel_period_pclks);

            if (enables.pre_range)
            {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }

            writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                    encodeTimeout(final_range_timeout_mclks));

            // set_sequence_step_timeout() end

            measurement_timing_budget_us = budget_us; // store for internal reuse
        }
        return true;
    }

    private int timeoutMicrosecondsToMclks(int timeout_period_us, int vcsel_period_pclks)
    {
        final int macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
    }

    private int encodeTimeout(int timeout_mclks)
    {
        // format: "(LSByte * 2^MSByte) + 1"

        int ls_byte = 0;
        int ms_byte = 0;

        if (timeout_mclks > 0)
        {
            ls_byte = timeout_mclks - 1;

            while ((ls_byte & 0xFFFFFF00) > 0)
            {
                ls_byte >>= 1;
                ms_byte++;
            }

            return (ms_byte << 8) | (ls_byte & 0xFF);
        }
        else { return 0; }
    }

    private boolean setSignalRateLimit(double limit_Mcps) {
        if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (int)(limit_Mcps * (1 << 7)));
        return true;
    }

    private boolean performSingleRefCalibration(int vhv_init_byte) {
        writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

        startTimeout();
        while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
            if (checkTimeoutExpired()) { return false; }
        }

        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        writeReg(SYSRANGE_START, 0x00);

        return true;
    }

    private static class SpadInfo {
        public final int count;
        public final boolean typeIsAperture;

        public SpadInfo(int c, boolean b) {
            count = c;
            typeIsAperture = b;
        }
    }


    private SpadInfo getSpadInfo() {
        return new SpadInfo(1, false);
    }

    public boolean init() {
        // "Set I2C standard mode"
        writeReg(0x88, 0x00);

        writeReg(0x80, 0x01);
        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);
        stop_variable = readReg(0x91);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        setSignalRateLimit(0.25);

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        SpadInfo spadInfo = getSpadInfo();

        int spad_count = spadInfo.count;
        boolean spad_type_is_aperture = spadInfo.typeIsAperture;

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        int[] ref_spad_map = new int[6];
        readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        writeReg(0xFF, 0x01);
        writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        writeReg(0xFF, 0x00);
        writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        int first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
        int spads_enabled = 0;

        for (int i = 0; i < 48; i++)
        {
            if (i < first_spad_to_enable || spads_enabled == spad_count)
            {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if (0 != ((ref_spad_map[i / 8] >> (i % 8)) & 0x1))
            {
                spads_enabled++;
            }
        }

        writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map);

        // -- VL53L0X_set_reference_spads() end

        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h

        writeReg(0xFF, 0x01);
        writeReg(0x00, 0x00);

        writeReg(0xFF, 0x00);
        writeReg(0x09, 0x00);
        writeReg(0x10, 0x00);
        writeReg(0x11, 0x00);

        writeReg(0x24, 0x01);
        writeReg(0x25, 0xFF);
        writeReg(0x75, 0x00);

        writeReg(0xFF, 0x01);
        writeReg(0x4E, 0x2C);
        writeReg(0x48, 0x00);
        writeReg(0x30, 0x20);

        writeReg(0xFF, 0x00);
        writeReg(0x30, 0x09);
        writeReg(0x54, 0x00);
        writeReg(0x31, 0x04);
        writeReg(0x32, 0x03);
        writeReg(0x40, 0x83);
        writeReg(0x46, 0x25);
        writeReg(0x60, 0x00);
        writeReg(0x27, 0x00);
        writeReg(0x50, 0x06);
        writeReg(0x51, 0x00);
        writeReg(0x52, 0x96);
        writeReg(0x56, 0x08);
        writeReg(0x57, 0x30);
        writeReg(0x61, 0x00);
        writeReg(0x62, 0x00);
        writeReg(0x64, 0x00);
        writeReg(0x65, 0x00);
        writeReg(0x66, 0xA0);

        writeReg(0xFF, 0x01);
        writeReg(0x22, 0x32);
        writeReg(0x47, 0x14);
        writeReg(0x49, 0xFF);
        writeReg(0x4A, 0x00);

        writeReg(0xFF, 0x00);
        writeReg(0x7A, 0x0A);
        writeReg(0x7B, 0x00);
        writeReg(0x78, 0x21);

        writeReg(0xFF, 0x01);
        writeReg(0x23, 0x34);
        writeReg(0x42, 0x00);
        writeReg(0x44, 0xFF);
        writeReg(0x45, 0x26);
        writeReg(0x46, 0x05);
        writeReg(0x40, 0x40);
        writeReg(0x0E, 0x06);
        writeReg(0x20, 0x1A);
        writeReg(0x43, 0x40);

        writeReg(0xFF, 0x00);
        writeReg(0x34, 0x03);
        writeReg(0x35, 0x44);

        writeReg(0xFF, 0x01);
        writeReg(0x31, 0x04);
        writeReg(0x4B, 0x09);
        writeReg(0x4C, 0x05);
        writeReg(0x4D, 0x04);

        writeReg(0xFF, 0x00);
        writeReg(0x44, 0x00);
        writeReg(0x45, 0x20);
        writeReg(0x47, 0x08);
        writeReg(0x48, 0x28);
        writeReg(0x67, 0x00);
        writeReg(0x70, 0x04);
        writeReg(0x71, 0x01);
        writeReg(0x72, 0xFE);
        writeReg(0x76, 0x00);
        writeReg(0x77, 0x00);

        writeReg(0xFF, 0x01);
        writeReg(0x0D, 0x01);

        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x01);
        writeReg(0x01, 0xF8);

        writeReg(0xFF, 0x01);
        writeReg(0x8E, 0x01);
        writeReg(0x00, 0x01);
        writeReg(0xFF, 0x00);
        writeReg(0x80, 0x00);

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
        writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        // -- VL53L0X_SetGpioConfig() end

        measurement_timing_budget_us = getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        setMeasurementTimingBudget(measurement_timing_budget_us);

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (!performSingleRefCalibration(0x40)) { return false; }

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!performSingleRefCalibration(0x00)) { return false; }

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // VL53L0X_PerformRefCalibration() end

        return true;
    }

    static class SequenceStepEnables
    {
        public boolean tcc, msrc, dss, pre_range, final_range;
    };

    static class SequenceStepTimeouts
    {
        public int pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

        public int msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
        public int msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    private static final int SYSRANGE_START                              = 0x00;
    private static final int SYSTEM_THRESH_HIGH                          = 0x0C;
    private static final int SYSTEM_THRESH_LOW                           = 0x0E;
    private static final int SYSTEM_SEQUENCE_CONFIG                      = 0x01;
    private static final int SYSTEM_RANGE_CONFIG                         = 0x09;
    private static final int SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04;
    private static final int SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A;
    private static final int GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84;
    private static final int SYSTEM_INTERRUPT_CLEAR                      = 0x0B;
    private static final int RESULT_INTERRUPT_STATUS                     = 0x13;
    private static final int RESULT_RANGE_STATUS                         = 0x14;
    private static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC;
    private static final int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0;
    private static final int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0;
    private static final int RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4;
    private static final int RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6;
    private static final int ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28;
    private static final int I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A;
    private static final int MSRC_CONFIG_CONTROL                         = 0x60;
    private static final int PRE_RANGE_CONFIG_MIN_SNR                    = 0x27;
    private static final int PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56;
    private static final int PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57;
    private static final int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64;
    private static final int FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67;
    private static final int FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47;
    private static final int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48;
    private static final int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    private static final int PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61;
    private static final int PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62;
    private static final int PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50;
    private static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51;
    private static final int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52;
    private static final int SYSTEM_HISTOGRAM_BIN                        = 0x81;
    private static final int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33;
    private static final int HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55;
    private static final int FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70;
    private static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71;
    private static final int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72;
    private static final int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20;
    private static final int MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46;
    private static final int SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF;
    private static final int IDENTIFICATION_MODEL_ID                     = 0xC0;
    private static final int IDENTIFICATION_REVISION_ID                  = 0xC2;
    private static final int OSC_CALIBRATE_VAL                           = 0xF8;
    private static final int GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4;
    private static final int GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5;
    private static final int GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6;
    private static final int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E;
    private static final int DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F;
    private static final int POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80;
    private static final int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89;
    private static final int ALGO_PHASECAL_LIM                           = 0x30;
    private static final int ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30;
}
