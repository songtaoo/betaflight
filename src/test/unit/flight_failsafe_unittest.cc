/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/bitarray.h"

    #include "fc/runtime_config.h"
    #include "fc/rc_modes.h"
    #include "fc/rc_controls.h"
    #include "fc/core.h"

    #include "flight/failsafe.h"
    #include "sensors/barometer.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    #include "io/beeper.h"

    #include "drivers/io.h"
    #include "drivers/time.h"
    #include "rx/rx.h"

    extern boxBitmask_t rcModeActivationMask;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t testFeatureMask = 0;
uint16_t testMinThrottle = 0;
throttleStatus_e throttleStatus = THROTTLE_HIGH;

enum {
    COUNTER_MW_DISARM = 0,
};
#define CALL_COUNT_ITEM_COUNT 1

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

void resetCallCounters(void)
{
    memset(&callCounts, 0, sizeof(callCounts));
}

#define TEST_MID_RC 1495            // something other than the default 1500 will suffice.
#define TEST_MIN_CHECK 1100;
#define PERIOD_OF_10_SCONDS 10000
#define DE_ACTIVATE_ALL_BOXES 0

uint32_t sysTickUptime;

void configureFailsafe(void)
{
    rxConfigMutable()->midrc = TEST_MID_RC;
    rxConfigMutable()->mincheck = TEST_MIN_CHECK;

    failsafeConfigMutable()->failsafe_delay = 10; // 1 second
    failsafeConfigMutable()->failsafe_off_delay = 15; // 1.5 seconds
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1;
    failsafeConfigMutable()->failsafe_throttle = 1200;
    failsafeConfigMutable()->failsafe_throttle_low_delay = 100; // 10 seconds
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;
    // NB we don't have failsafe_recovery_delay so use PERIOD_RXDATA_RECOVERY (100ms)
    sysTickUptime = 0;
}

void activateBoxFailsafe()
{
    boxBitmask_t newMask;
    memset(&newMask, 0, sizeof(newMask));
    bitArraySet(&newMask, BOXFAILSAFE);
    rcModeUpdate(&newMask);
}

void deactivateBoxFailsafe()
{
    boxBitmask_t newMask;
    memset(&newMask, 0, sizeof(newMask));
    rcModeUpdate(&newMask);
}

//
// Stepwise tests
//

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeInitialState)
{
    // given
    configureFailsafe();
    // and
    DISABLE_ARMING_FLAG(ARMED);

    // when
    failsafeInit();
    failsafeReset();

    // then
    EXPECT_FALSE(failsafeIsMonitoring());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeStartMonitoring)
{
    // when
    failsafeStartMonitoring();

    // then
    EXPECT_TRUE(failsafeIsMonitoring());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeFirstArmedCycle)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);

    // when
    sysTickUptime ++;
    failsafeOnValidDataFailed();                    // set last invalid sample to a non-zero value
    sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived();                  // cause a recovered link

    // and
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeNotActivatedWhenReceivingData)
{
    // when
    for (sysTickUptime = 0; sysTickUptime < PERIOD_OF_10_SCONDS; sysTickUptime++) {
        failsafeOnValidDataReceived();

        failsafeUpdateState();

        // then
        EXPECT_FALSE(failsafeIsActive());
        EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    }
}

/****************************************************************************************/
//
// Clean start
//
/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsRxLossAndStartsLanding)
{
    // given
    configureFailsafe();
    failsafeInit();
    
    DISABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // then
    EXPECT_TRUE(failsafeIsMonitoring());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());

    // given
    ENABLE_ARMING_FLAG(ARMED);
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeOnValidDataReceived();

    sysTickUptime = 0;
    failsafeUpdateState();

    // then
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(isArmingDisabled());

    // simulate an Rx loss for the stage 1 duration
    sysTickUptime += (failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND);
    failsafeOnValidDataFailed();
    failsafeUpdateState();

    //  should be in stage 1
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    sysTickUptime ++;                               // exceed the stage 1 period by one tick
    failsafeOnValidDataFailed();                    // confirm that we still have no valid data

    failsafeUpdateState();

    // we should now be in stage 2, landing phase
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeCausesLanding)
// note this test follows on from the previous test
{
    // exceed the stage 2 landing time
    sysTickUptime += (failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND);
    failsafeOnValidDataFailed();                    // confirm that we still have no valid data

    // when
    failsafeUpdateState();

    // expect to still be in landing phase
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());

    // adjust time to point just past the landing time
    sysTickUptime++;
    failsafeOnValidDataFailed();                    // confirm that we still have no valid data

    // when
    failsafeUpdateState();

    // expect to be in monitoring mode
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // let's wait 3 seconds and still get no signal
    sysTickUptime += PERIOD_OF_3_SECONDS;
    failsafeOnValidDataFailed();                    // confirm that we still have no valid data
    // when
    failsafeUpdateState();

    // nothing should change
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // now lets get a signal while in monitoring mode
    failsafeOnValidDataReceived();
    // we must clear the first delay in failsafeOnValidDataReceived(), for which 200ms 'works'
    sysTickUptime += PERIOD_RXDATA_RECOVERY;
    failsafeOnValidDataReceived();
    failsafeUpdateState();

    // nothing should change
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    sysTickUptime += PERIOD_RXDATA_RECOVERY;
    failsafeOnValidDataReceived();
    failsafeUpdateState();

    // nothing should change
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // One tick later
    sysTickUptime++;
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();

    // we expect failsafe to finish and to revert to idle
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM)); // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeDetectsRxLossAndJustDisarms)
//  Just Disarm is when throttle is low for >10s before signal loss
//  we should exit stage 1 directly into failsafe monitoring mode, and not enter landing mode
{
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // set to normal initial state
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_KILL;

    sysTickUptime += PERIOD_OF_3_SECONDS;           // 3s of normality
    failsafeOnValidDataReceived();                  // we have a valid signal
    // when
    failsafeUpdateState();

    // confirm that we are in idle mode
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(isArmingDisabled());


    // run throttle_low for 11s
    throttleStatus = THROTTLE_LOW;                  // for failsafe 'just-disarm' procedure
    sysTickUptime += 11000;
    failsafeOnValidDataReceived();                  // set the last valid signal to now

    failsafeUpdateState();

    // check that we are still in idle mode
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(isArmingDisabled());

    // simulate an Rx loss for the stage 1 duration
    sysTickUptime += (failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND);
    failsafeOnValidDataFailed();
    failsafeUpdateState();

    //  should remain in stage 1
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    sysTickUptime ++;                               // now we exceed stage 1
    failsafeOnValidDataFailed();                    // we still have no valid data
    failsafeUpdateState();

    // we should now be in stage 2 via Just Disarm, going direct to monitoring mode.
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // now lets get a signal while in monitoring mode
    failsafeOnValidDataReceived();
    // we must clear the first delay in failsafeOnValidDataReceived(), for which 200ms 'works'
    sysTickUptime += PERIOD_RXDATA_RECOVERY;
    failsafeOnValidDataReceived();
    failsafeUpdateState();

    // nothing should change
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    sysTickUptime += PERIOD_RXDATA_RECOVERY;
    failsafeOnValidDataReceived();
    failsafeUpdateState();

    // nothing should change
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // One tick later
    sysTickUptime++;
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();

    // we expect failsafe to finish and to revert to idle
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM)); // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());}

/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeSwitchModeKill)
{
    // given
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // set to normal initial state
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_KILL;

    sysTickUptime += PERIOD_OF_3_SECONDS;           // 3s of normality
    failsafeOnValidDataReceived();                  // we have a valid signal
    // when
    failsafeUpdateState();

    // confirm that we are in idle mode
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());

    // given
    activateBoxFailsafe();                          // activate the Kill swith

    // when
    failsafeUpdateState();                          // should failsafe immediately the kill switch is hit

    // then
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // given
    failsafeOnValidDataReceived();                  // the link is active the whole time

    // deactivate the kill switch
    deactivateBoxFailsafe();                        // receivingRxData is immediately true

    // we should go to failsafe monitoring mode, via Landing
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // since we didn't enter stage 2, we have one rxDataRecoveryPeriod delay to deal with
    // while within rxDataRecoveryPeriod in monitoring mode...
    sysTickUptime += PERIOD_RXDATA_RECOVERY;
    failsafeOnValidDataReceived();

    // when 
    failsafeUpdateState();

    // we should still be in failsafe monitoring mode
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // one tick later
    sysTickUptime ++;

    // when
    failsafeUpdateState();

    // we should now have exited failsafe
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

/****************************************************************************************/

TEST(FlightFailsafeTest, TestFailsafeSwitchModeStage1OrStage2Drop)
// should immediately stop motors and go to monitoring mode diretly
{
    // given a clean start
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // and set initial states
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_DROP_IT;

    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // we have a valid signal
    sysTickUptime += 3000;                          // 3s of normality

    // when
    failsafeUpdateState();
    
    // confirm that we are in idle mode
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());

    // given
    activateBoxFailsafe();                          // activate the stage 2 Drop switch
    failsafeOnValidDataFailed();                    // immediate stage 2 switch sets failsafeOnValidDataFailed

    // when
    failsafeUpdateState();                          // should activate stage2 immediately, even though signal is good

    // expect to be in monitoring mode, and to have disarmed
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // deactivate the failsafe switch
    deactivateBoxFailsafe();

    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());

    // by next evaluation we should be out of failsafe
    sysTickUptime ++;
    // receivingRxData is immediately true because signal exists
    failsafeOnValidDataReceived();

    // when 
    failsafeUpdateState();

    // we should now have exited failsafe
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}

/****************************************************************************************/

TEST(FlightFailsafeTest, TestFailsafeSwitchModeStage2Land200Seconds)
{
    const int16_t previousOffDelay = failsafeConfig()->failsafe_off_delay;
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();
    throttleStatus = THROTTLE_HIGH;
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;
    failsafeConfigMutable()->failsafe_off_delay = 2000;
    ASSERT_EQ(2000, failsafeConfig()->failsafe_off_delay);

    sysTickUptime = 0;
    failsafeOnValidDataReceived();
    sysTickUptime += 3000;
    failsafeUpdateState();
    activateBoxFailsafe();
    failsafeOnValidDataFailed();
    failsafeUpdateState();
    ASSERT_EQ(FAILSAFE_LANDING, failsafePhase());

    const uint32_t landingStartedAt = sysTickUptime;
    const uint32_t elapsedTimes[] = {20000, 25500, 100000, 199999, 200000};
    for (const uint32_t elapsed : elapsedTimes) {
        sysTickUptime = landingStartedAt + elapsed;
        failsafeUpdateState();
        EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    }

    sysTickUptime = landingStartedAt + 200001;
    failsafeUpdateState();
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_TRUE(isArmingDisabled());

    // These legacy tests share state; leave the switch and recovery state clean.
    deactivateBoxFailsafe();
    sysTickUptime++;
    failsafeOnValidDataReceived();
    failsafeUpdateState();
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_FALSE(isArmingDisabled());
    failsafeConfigMutable()->failsafe_off_delay = previousOffDelay;
}

TEST(FlightFailsafeTest, TestFailsafeSwitchModeStage2Land)
{
    // given a clean start
    ENABLE_ARMING_FLAG(ARMED);
    resetCallCounters();
    failsafeStartMonitoring();

    // and
    throttleStatus = THROTTLE_HIGH;                 // throttle HIGH to go for a failsafe landing procedure
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;

    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // we have a valid signal
    sysTickUptime += 3000;                          // 3s of normality

    // when
    failsafeUpdateState();
    
    // confirm that we are in idle mode
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());

    // given
    activateBoxFailsafe();                          // activate the stage 2 Drop switch
    failsafeOnValidDataFailed();                    // immediate stage 2 switch sets failsafeOnValidDataFailed

    // when
    failsafeUpdateState();                          // should activate stage2 immediately

    // expect to immediately be in landing mode, and not disarmed
    EXPECT_TRUE(failsafeIsActive());               // stick induced failsafe allows re-arming
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    
    // should stay in landing for failsafe_off_delay (stage 2 period) of 1s
    sysTickUptime += failsafeConfig()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;

    // when
    failsafeUpdateState();

    EXPECT_TRUE(failsafeIsActive());                    // stick induced failsafe allows re-arming
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));

    // one tick later, landing time has elapsed
    sysTickUptime ++;

    // when
    failsafeUpdateState();

    // now should be in monitoring mode, with switch holding signalReceived false
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));

    // given
    sysTickUptime += PERIOD_OF_3_SECONDS;               // hang around a bit waiting for switch change

    // when
    failsafeUpdateState();

    // should still be in monitoring mode because switch is still forcing receivingRxData false
    EXPECT_TRUE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));

    // deactivate the failsafe switch
    deactivateBoxFailsafe();

    // receivingRxData is immediately true
    // we go directly to failsafe monitoring mode, via Landing
    // however, because the switch also forces rxFlightChannelsValid false, emulating real failsafe
    // we have two delays to deal with before we can re-arm

    EXPECT_TRUE(failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());


    // one tick later
    sysTickUptime ++;
    failsafeOnValidDataReceived();

    // when
    failsafeUpdateState();

    // we should now have exited failsafe
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));  // disarm not called repeatedly.
    EXPECT_FALSE(isArmingDisabled());
}


/****************************************************************************************/
//
// Clean start
//
/****************************************************************************************/
TEST(FlightFailsafeTest, TestFailsafeNotActivatedWhenDisarmedAndRXLossIsDetected)
{
    // given
    resetCallCounters();
    configureFailsafe();

    // and
    failsafeInit();

    // and ** WE ARE DISARMED **
    DISABLE_ARMING_FLAG(ARMED);

    // when failsafe starts
    failsafeStartMonitoring();

    // and
    sysTickUptime = 0;                              // restart time from 0
    failsafeOnValidDataReceived();                  // we have a valid signal
    sysTickUptime += 3000;                          // 3s of normality

    // when
    failsafeUpdateState();

    // confirm that we are in idle mode
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));

    // and that arming is not disabled
    EXPECT_FALSE(isArmingDisabled());

    // simulate an Rx loss for the stage 1 duration
    sysTickUptime += (failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND);
    failsafeOnValidDataFailed();
    failsafeUpdateState();

    // within stage 1 time from a true failsafe while disarmed, stage 2 should normally not be active
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));

    // arming is disabled due to setArmingDisabled in stage 1 due to failsafeOnValidDataFailed()
    EXPECT_FALSE(failsafeIsActive());

   // given
    sysTickUptime++;                                // adjust time to point just past stage 1
    failsafeOnValidDataFailed();                    // would normally enter stage 2, but we are disarmed
    failsafeUpdateState();

    // expect that we do not enter failsafe stage 2, ie no change from above
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));

    // the lock on the aux channels persists at fixed stage 1 values until recovery time
    EXPECT_TRUE(isArmingDisabled());

    // allow signal received for the recovery time
    sysTickUptime += PERIOD_RXDATA_RECOVERY;
    failsafeOnValidDataReceived();
    failsafeUpdateState();

    // no change in failsafe state (still idle)
    EXPECT_FALSE(failsafeIsActive());
    EXPECT_EQ(FAILSAFE_IDLE, failsafePhase());
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));

    // but now arming is possible
    EXPECT_FALSE(isArmingDisabled());
}

static bool testBaroReady = true;
static bool testBaroCalibrated = true;

class LandingDetectionTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        configureFailsafe();
        failsafeInit();
        failsafeReset();
        deactivateBoxFailsafe();
        DISABLE_ARMING_FLAG(ARMED);
        unsetArmingDisabled((armingDisableFlags_e)~0U);
        sensorsSet(SENSOR_BARO | SENSOR_ACC | SENSOR_GYRO);
        testBaroReady = testBaroCalibrated = true;
        memset(&acc, 0, sizeof(acc));
        memset(&gyro, 0, sizeof(gyro));
        memset(&baro, 0, sizeof(baro));
        acc.dev.acc_1G = 2048;
        acc.accADC[Z] = 2048;
        acc.isAccelUpdatedAtLeastOnce = true;
        failsafeConfigMutable()->failsafe_ld_en = true;
        failsafeConfigMutable()->failsafe_ld_alt = 300;
        failsafeConfigMutable()->failsafe_ld_ms = 1500;
        failsafeConfigMutable()->failsafe_off_delay = 2000;
        failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE2;
        failsafeStartMonitoring();
        resetCallCounters();
        // Establish the disarmed reference, then capture it at the real arming hook.
        hold(0, 1500);
        ENABLE_ARMING_FLAG(ARMED);
        failsafeOnArm();
        failsafeOnRxResume();
        hold(1000, 1000);
        activateBoxFailsafe();
        failsafeOnValidDataFailed();
        failsafeUpdateState();
        ASSERT_EQ(FAILSAFE_LANDING, failsafePhase());
    }

    void TearDown() override
    {
        deactivateBoxFailsafe();
        failsafeOnRxResume();
        failsafeUpdateState();
        DISABLE_ARMING_FLAG(ARMED);
        failsafeUpdateState();
        failsafeConfigMutable()->failsafe_ld_en = false;
    }

    void step(float altitude, bool freshBaro = true, bool freshAcc = true)
    {
        sysTickUptime += 10;
        baro.altitude = altitude;
        if (freshBaro) {
            baro.hasAltitudeSample = true;
            baro.lastAltitudeUpdateUs = micros();
        }
        if (freshAcc) {
            acc.lastUpdateUs = micros();
        }
        failsafeUpdateState();
    }

    void hold(float altitude, unsigned duration)
    {
        for (unsigned t = 0; t < duration; t += 10) {
            step(altitude);
        }
    }

    void descend(float endAltitude = 0)
    {
        // 1 m/s from 10m above the arming reference.
        for (float altitude = 1000; altitude > endAltitude; altitude -= 1) {
            step(altitude);
        }
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    }
};

TEST_F(LandingDetectionTest, DescendThenSettleDisarmsBeforeTimeout)
{
    descend();
    hold(0, 1000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    hold(0, 4000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
}

TEST_F(LandingDetectionTest, DisabledRetainsTimeout)
{
    failsafeConfigMutable()->failsafe_ld_en = false;
    descend();
    hold(0, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, NoDescentCannotDisarm)
{
    failsafeConfigMutable()->failsafe_ld_alt = 0;
    hold(1000, 8000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, AltitudeOutsideToleranceCannotDisarm)
{
    descend(500);
    hold(500, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, ZeroToleranceDisablesOnlyAltitudeGate)
{
    failsafeConfigMutable()->failsafe_ld_alt = 0;
    descend(500);
    gyro.gyroADCf[X] = 40;
    hold(500, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    gyro.gyroADCf[X] = 0;
    hold(500, 2000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, FiftyMetreToleranceAccepted)
{
    failsafeConfigMutable()->failsafe_ld_alt = 5000;
    descend(500);
    hold(500, 5000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, AccelerationMotionPreventsDisarm)
{
    descend();
    acc.accADC[Z] = 3000;
    hold(0, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, StaleBaroCannotDisarm)
{
    descend();
    for (unsigned t = 0; t < 5000; t += 10) {
        step(0, false);
    }
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, StaleAccelerometerCannotDisarm)
{
    descend();
    for (unsigned t = 0; t < 5000; t += 10) {
        step(0, true, false);
    }
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, MissingBarometerCannotDisarm)
{
    descend();
    sensorsClear(SENSOR_BARO);
    hold(0, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, ConfirmationDurationHonoured)
{
    failsafeConfigMutable()->failsafe_ld_ms = 5000;
    descend();
    hold(0, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    hold(0, 3000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, MotionRestartsConfirmation)
{
    failsafeConfigMutable()->failsafe_ld_ms = 5000;
    descend();
    hold(0, 4500);
    gyro.gyroADCf[Z] = 50;
    step(0);
    gyro.gyroADCf[Z] = 0;
    hold(0, 4000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
    hold(0, 2000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, ExitAndReenterRequiresNewDescent)
{
    descend();
    deactivateBoxFailsafe();
    failsafeOnRxResume();
    step(0);
    hold(0, 100);
    activateBoxFailsafe();
    failsafeOnValidDataFailed();
    hold(0, 6000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, RearmingCapturesNewAltitudeReference)
{
    deactivateBoxFailsafe();
    failsafeOnRxResume();
    step(2000);
    DISABLE_ARMING_FLAG(ARMED);
    hold(2000, 3000);
    ENABLE_ARMING_FLAG(ARMED);
    failsafeOnArm();
    hold(3000, 1000);
    activateBoxFailsafe();
    failsafeOnValidDataFailed();
    step(3000);
    for (float altitude = 3000; altitude > 2000; altitude -= 1) {
        step(altitude);
    }
    hold(2000, 5000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, MissingArmingReferenceBlocksAltitudeGate)
{
    // A second arming call without a disarmed baseline must invalidate the old one.
    failsafeOnArm();
    descend();
    hold(0, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, ZeroToleranceDoesNotRequireArmingReference)
{
    failsafeOnArm();
    failsafeConfigMutable()->failsafe_ld_alt = 0;
    descend();
    hold(0, 5000);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, BelowReferenceOutsideToleranceCannotDisarm)
{
    descend(-500);
    hold(-500, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, MissingAccelerometerCannotDisarm)
{
    descend();
    sensorsClear(SENSOR_ACC);
    hold(0, 5000);
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_MW_DISARM));
}

TEST_F(LandingDetectionTest, TimeoutStillDisarmsWhenAltitudeGateFails)
{
    hold(1000, 200100);
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_MW_DISARM));
}

// STUBS

extern "C" {
baro_t baro;
acc_t acc;
gyro_t gyro;
bool isBaroReady(void) { return testBaroReady; }
bool baroIsCalibrated(void) { return testBaroCalibrated; }
float getBaroAltitude(void) { return baro.altitude; }
bool accIsCalibrationComplete(void) { return true; }
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float rcCommand[4];
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;
bool isUsingSticksToArm = true;

PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

uint32_t micros(void)
{
    return millis() * 1000;
}

throttleStatus_e calculateThrottleStatus()
{
    return throttleStatus;
}

void delay(uint32_t) {}

bool featureIsEnabled(uint32_t mask)
{
    return (mask & testFeatureMask);
}

void disarm(flightLogDisarmReason_e)
{
    callCounts[COUNTER_MW_DISARM]++;
}

void beeper(beeperMode_e mode)
{
    UNUSED(mode);
}

uint16_t getCurrentMinthrottle(void)
{
    return testMinThrottle;
}

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}

bool areSticksActive(uint8_t stickPercentLimit)
{
    UNUSED(stickPercentLimit);
    return false;
}

void beeperConfirmationBeeps(uint8_t beepCount) { UNUSED(beepCount); }

bool crashRecoveryModeActive(void) { return false; }
void pinioBoxTaskControl(void) {}
}
