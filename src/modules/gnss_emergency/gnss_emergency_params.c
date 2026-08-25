/**
 * Enable GNSS interference emergency policy
 *
 * @boolean
 * @group GNSS Emergency
 */
PARAM_DEFINE_INT32(GEM_EN, 1);

/**
 * GNSS recovery confirmation time
 *
 * GNSS fix and local/global navigation validity must remain healthy for this
 * time before an interrupted mission is resumed.
 *
 * @unit s
 * @min 1
 * @max 30
 * @decimal 1
 * @increment 1
 * @group GNSS Emergency
 */
PARAM_DEFINE_FLOAT(GEM_REC_T, 3.f);
