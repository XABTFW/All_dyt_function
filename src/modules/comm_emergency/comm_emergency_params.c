/**
 * Enable communication emergency policy
 *
 * @boolean
 * @group Communication Emergency
 */
PARAM_DEFINE_INT32(CEM_EN, 1);

/**
 * Communication-loss hold time
 *
 * Time to hold before returning to the takeoff point in any airborne mode. A battery capacity
 * below CEM_BAT_THR can trigger an earlier return or landing decision.
 *
 * @unit s
 * @min 5
 * @max 600
 * @decimal 1
 * @increment 5
 * @group Communication Emergency
 */
PARAM_DEFINE_FLOAT(CEM_WAIT, 30.f);

/**
 * Communication-loss timeout action
 *
 * Action after CEM_WAIT expires while the communication link is still lost.
 * The low-battery decision during the waiting period remains energy-aware and
 * can independently select Return or Land.
 *
 * @value 0 Return
 * @value 1 Land
 * @min 0
 * @max 1
 * @group Communication Emergency
 */
PARAM_DEFINE_INT32(CEM_TO_ACT, 0);

/**
 * Communication emergency battery threshold
 *
 * Below this remaining-capacity threshold, return feasibility is checked.
 * The vehicle returns when its estimated remaining flight time exceeds the
 * safe RTL time estimate; otherwise it lands at its current position.
 *
 * @unit norm
 * @min 0.05
 * @max 0.95
 * @decimal 2
 * @increment 0.05
 * @group Communication Emergency
 */
PARAM_DEFINE_FLOAT(CEM_BAT_THR, 0.40f);
