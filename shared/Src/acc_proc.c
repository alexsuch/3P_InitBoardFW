#if LIS2DH12_ACC_ENABLE
#include <LIS2DH12.h>
#endif
#include <string.h>
#include "init_brd.h"
#include "acc_proc.h"
#include "prj_config.h"

static accProcStatus_t accProcStatus;

static app_cbk_fn acc_sys_cbk = NULL;
static int16_t *x_axis;
static int16_t *y_axis;
static int16_t *z_axis;

#if ACC_NO_DIVIDE_ENABLE
const uint64_t accSqrtThreshold = ACC_SQRT_TRESHOLD * ACC_BUFF_SIZE * ACC_BUFF_SIZE;
#else
const uint32_t accSqrtThreshold = ACC_SQRT_TRESHOLD;
#endif



#if ACC_NO_DIVIDE_ENABLE
static uint64_t tmp32;
#else
static uint32_t tmp32;
static int16_t x_tmp;
static int16_t y_tmp;
static int16_t z_tmp;
#endif

static int16_t x_tmp;
static int16_t y_tmp;
static int16_t z_tmp;

static uint8_t prev_idx;

#if NET_DETECTION_ENABLE
const int32_t accNetThreshold = ACC_NET_TRESHOLD;
static int32_t accNetThreshold1 = ACC_NET_TRESHOLD;
static uint8_t oldest_idx;
static int32_t x_new, y_new, z_new, x_old, y_old, z_old, norm_new, norm_old, delta;
#endif

/*-------------------------- Movement calculations block -----------------------*/

#define FIXED_SCALE (1LL << 16) // Масштаб: 2^16 = 65536 точність 4-5 знаків
#define FLOAT_TO_FIXED(x) ((int64_t)((x) * FIXED_SCALE))
//#define FIXED_TO_FLOAT(x) ((float)(x) / FIXED_SCALE)
#define FIXED_MUL(a, b) ((int64_t)(((int64_t)(a) * (int64_t)(b)) >> 16))
#define FIXED_DIV(a, b) ((int64_t)(((int64_t)(a) << 16) / (b)))

// Граничне значення для фільтрації шуму
#define NOISE_THRESHOLD 20LL

// Граничне значення для детекції руху (наприклад 1.1 радіана/сек = FLOAT_TO_FIXED(1.1) = 72089
#define MOVEMENT_THRESHOLD 72089LL

//Налаштування акселерометра
// 1000 - 1G
// 8000 - 8G максимальне значення
/*
 * Максимальне значення прискорення по одній осі
 * Якщо значення прискорення хоча б на одній з осей буде більше порогу
 * вимірювання буде ігноруватись.
 * !!! Вирівнювання значень по порогу буде змінювати кут вектора що може
 * спричинити детекцію перевертання, тому значення ігноруються
 */
#define ACC_AXIS_LIMIT 8500LL

//*l - last value,   *c - current value
int16_t xl, yl, zl, xc, yc, zc;
int8_t firstMeasure = 1;
static int64_t diff, dot, cosThetaFixed, thetaFixed, magnitude_l, magnitude_c;

// Кутова швидкість (рад/с)
static int64_t angularVelocityFixed;

//For ACOS
#define COEF_1 (-1227LL) //FLOAT_TO_FIXED(-0.0187293f);
#define COEF_2 (4866LL) //FLOAT_TO_FIXED(0.0742610f);
#define COEF_3 (13901LL) //FLOAT_TO_FIXED(0.2121144f);
#define COEF_4 (102939LL) //FLOAT_TO_FIXED(1.5707288f);
#define COEF_5 (205887LL) //FLOAT_TO_FIXED(3.14159265358979f);

int64_t constrain_int64(int64_t v, int64_t min, int64_t max) {
    if (v < min) {
        return min;
    } else if (v > max) {
        return max;
    }
    return v;
}

int64_t vectorMagnitudeScaled(int16_t x, int16_t y, int16_t z) {
    return (int64_t) x * x + (int64_t) y * y + (int64_t) z * z;
}

int64_t dotProduct(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2) {
    return (int64_t) x1 * x2 + (int64_t) y1 * y2 + (int64_t) z1 * z2;
}

int64_t sqrtBitwiseUint64(int64_t a) {
    if (a == 0 || a < 0) return 0;
    int64_t result = 0;
    int64_t bit = ((int64_t) 1) << 32;

    while (bit > a) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (a >= result + bit) {
            a -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }

    return result;
}

int64_t absInt64(int64_t v) {
    return (v < 0) ? -v : v;
}

int64_t acosFastFixed(int64_t x) {
    int64_t negate = (x < 0) ? 1 : 0;
    x = (x < 0) ? -x : x;
    int64_t ret = COEF_1;
    ret = FIXED_MUL(ret, x);
    ret = ret + COEF_2;
    ret = FIXED_MUL(ret, x);
    ret = ret - COEF_3;
    ret = FIXED_MUL(ret, x);
    ret = ret + COEF_4;
    // sqrt(1.0 - x) обчислюється через фіксовану точність
    int64_t sqrt_one_minus_x = sqrtBitwiseUint64((FLOAT_TO_FIXED(1.0f) - x) * FIXED_SCALE);
    ret = FIXED_MUL(ret, sqrt_one_minus_x);
    ret = ret - FIXED_MUL(2 * negate, ret);
    ret = negate * COEF_5 + ret;
    return ret;
}

uint8_t calcVectorSpeed(int16_t x, int16_t y, int16_t z, uint32_t currentTimeMillis) {
    if (firstMeasure == 1) {
        xl = x;
        yl = y;
        zl = z;
        magnitude_l = sqrtBitwiseUint64(vectorMagnitudeScaled(x, y, z));
        firstMeasure = 0;
        return 0;
    }

    if (x > ACC_AXIS_LIMIT || y > ACC_AXIS_LIMIT || z > ACC_AXIS_LIMIT) {
        //починаємо вимірювання спочатку оскільки це аномальне значення вектора
        firstMeasure = 1;
        return 0;
    }

    int8_t detected = 0;
    xc = x;
    yc = y;
    zc = z;

    magnitude_c = sqrtBitwiseUint64(vectorMagnitudeScaled(x, y, z));

    //last measure not correct
    if (magnitude_l == 0) {
        magnitude_l = magnitude_c;
#if TEST_DETECTION
        Serial.println("Not correct last measure");
#endif
        return 0;
    }

    if (magnitude_c == 0) {
#if TEST_DETECTION
        Serial.println("Not correct current measure");
#endif
        return 0;
    }
    diff = (int64_t) magnitude_c - (int64_t) magnitude_l;

    // Перевірка на рух (ігноруємо значення нижче порогу шуму)
    if (absInt64(diff) > NOISE_THRESHOLD) {

        // Обчислення скалярного добутку
        dot = dotProduct(xl, yl, zl, xc, yc, zc);
        cosThetaFixed = FIXED_DIV(dot, magnitude_c * magnitude_l);
        cosThetaFixed = constrain_int64(cosThetaFixed, -FIXED_SCALE, FIXED_SCALE);

        thetaFixed = acosFastFixed(cosThetaFixed);

        angularVelocityFixed = FIXED_DIV(thetaFixed, currentTimeMillis * FIXED_SCALE / 1000);

#if TEST_DETECTION
        Serial.printf("v= %u\n", angularVelocityFixed);
#endif
        // Перевірка на рух за швидкістю і модулем вектора
#if MOVE_THRESHOLD_NOT_CONFIG
        if (absInt64(angularVelocityFixed) > MOVEMENT_THRESHOLD)
#else
            if (absInt64(angularVelocityFixed) > accProcStatus.move_threshold)
#endif /* MOVE_THRESHOLD_CONFIG */
        {
            detected = 1;
            // BOOOM!
#if TEST_DETECTION
            Serial.println("BOOM!");
#endif
        } else {
            // small movement or noise
        }
    }

    magnitude_l = magnitude_c;
    xl = xc;
    yl = yc;
    zl = zc;

    return detected;
}
/*--------------------------Hit detection block -------------------------------*/

#if NET_DETECTION_ENABLE
static int32_t approx_norm(int32_t x, int32_t y, int32_t z)
{
    x = (x < 0) ? -x : x;
    y = (y < 0) ? -y : y;
    z = (z < 0) ? -z : z;

    int32_t max, mid;

    if (x >= y && x >= z) {
        max = x;
        mid = (y > z) ? y : z;
    } else if (y >= x && y >= z) {
        max = y;
        mid = (x > z) ? x : z;
    } else {
        max = z;
        mid = (x > y) ? x : y;
    }

    return max + (mid >> 1);  // max + mid / 2
}
#endif /* NET_DETECTION_ENABLE */

static void App_AccCbk (system_evt_t evt, uint32_t usr_data)
{
	if (evt == SYSTEM_EVT_ERROR)
	{
		if (acc_sys_cbk != NULL)
		{
			/* Pass the error to the higher layer */
			acc_sys_cbk(SYSTEM_EVT_ERROR, 0u);
		}
	}
	else if (evt == SYSTEM_EVT_READY)
	{
		if (usr_data == ACC_EVT_INIT_OK)
		{
			if (acc_sys_cbk != NULL)
			{
				/* Pass the init event to the higher layer */
				acc_sys_cbk(SYSTEM_EVT_READY, usr_data);
			}
		}
		else if (usr_data == ACC_EVT_HIT_INIT_OK)
		{
			/* Start hit processing */
			accProcStatus.hit_detection_enabled  = true;
		}
		else if (usr_data == ACC_EVT_MOVE_INIT_OK)
		{
			/* Start movement detect processing */
			accProcStatus.move_detection_enabled = true;
		}
		else if (usr_data == ACC_EVT_DATA_READY)
		{
			/* Data is ready for processing */
			if (
					(accProcStatus.hit_detection_enabled != false) ||
                    (accProcStatus.move_detection_enabled != false)
				)
			{
				/* Process the data */
				accProcStatus.data_ready = true;
			}
		}
		else
		{
			//Empty handler
		}
	}
}

static void AccProc_Processing(void)
{
	if (accProcStatus.data_ready != false)
	{
		if (accProcStatus.init_skip_counts == 0)
		{
			if (accProcStatus.hit_detection_enabled != false)
			{
			    // 1. Вилучаємо старе значення з суми
			    accProcStatus.x_sum -= accProcStatus.x_buff[accProcStatus.idx];
			    accProcStatus.y_sum -= accProcStatus.y_buff[accProcStatus.idx];
			    accProcStatus.z_sum -= accProcStatus.z_buff[accProcStatus.idx];

			    // 2. Записуємо нові значення
			    accProcStatus.x_buff[accProcStatus.idx] = *x_axis;
			    accProcStatus.y_buff[accProcStatus.idx] = *y_axis;
			    accProcStatus.z_buff[accProcStatus.idx] = *z_axis;

			    // 3. Додаємо нове значення до суми
			    accProcStatus.x_sum += *x_axis;
			    accProcStatus.y_sum += *y_axis;
			    accProcStatus.z_sum += *z_axis;

			    // 4. Збільшуємо лічильник валідності (тільки до MAX)
			    if (accProcStatus.validCount < ACC_BUFF_SIZE)
			        accProcStatus.validCount++;

			    // 5. Розрахунок тільки після заповнення буфера
			    if (accProcStatus.validCount >= ACC_BUFF_SIZE)
			    {
#if ACC_NO_DIVIDE_ENABLE
			        // Без ділення — сума квадратів, порівнюється з масштабованим threshold
			        tmp32 = accProcStatus.x_sum * accProcStatus.x_sum +
			                accProcStatus.y_sum * accProcStatus.y_sum +
			                accProcStatus.z_sum * accProcStatus.z_sum;
#else
			        // Ділення — обчислення середнього
			        int32_t x_tmp = accProcStatus.x_sum / (int32_t)ACC_BUFF_SIZE;
			        int32_t y_tmp = accProcStatus.y_sum / (int32_t)ACC_BUFF_SIZE;
			        int32_t z_tmp = accProcStatus.z_sum / (int32_t)ACC_BUFF_SIZE;

			        tmp32 = x_tmp * x_tmp + y_tmp * y_tmp + z_tmp * z_tmp;
#endif

			        // 6. Порівняння з порогом
			        if (tmp32 > accSqrtThreshold)
			        {
			            if (!accProcStatus.hitDetected)
			            {
			                accProcStatus.hitDetected = true;
			                 if (acc_sys_cbk != NULL)  acc_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_HIT_DETECTED);
			            }
			        }
			        else
			        {
			            accProcStatus.hitDetected = false;
			        }
			    }

			    // 7. Перехід до наступного індексу в циклічному буфері
			    accProcStatus.idx = (accProcStatus.idx + 1) % ACC_BUFF_SIZE;

#if NET_DETECTION_ENABLE

			    oldest_idx = (accProcStatus.net_idx + 1) % ACC_NET_BUFF_SIZE; // Why not +1

			    // Оновлення суми + збереження
			    accProcStatus.net_x_sum_buff[accProcStatus.net_idx] =
			        accProcStatus.net_x_sum += *x_axis - accProcStatus.net_x_raw_buff[oldest_idx];
			    accProcStatus.net_y_sum_buff[accProcStatus.net_idx] =
			        accProcStatus.net_y_sum += *y_axis - accProcStatus.net_y_raw_buff[oldest_idx];
			    accProcStatus.net_z_sum_buff[accProcStatus.net_idx] =
			        accProcStatus.net_z_sum += *z_axis - accProcStatus.net_z_raw_buff[oldest_idx];

			    // Зберігаємо сирі значення
			    accProcStatus.net_x_raw_buff[accProcStatus.net_idx] = *x_axis;
			    accProcStatus.net_y_raw_buff[accProcStatus.net_idx] = *y_axis;
			    accProcStatus.net_z_raw_buff[accProcStatus.net_idx] = *z_axis;

			    if (accProcStatus.net_validCount < ACC_BUFF_SIZE)
			    {
			        accProcStatus.net_validCount++;
			    }

			    // Перевірка через ACC_NET_DECIMATION_RATE
			    accProcStatus.net_check_counter++;
			    if (accProcStatus.net_check_counter >= ACC_NET_DECIMATION_RATE)
			    {
			        accProcStatus.net_check_counter = 0;

			        x_new = accProcStatus.net_x_sum_buff[accProcStatus.net_idx];
			        y_new = accProcStatus.net_y_sum_buff[accProcStatus.net_idx];
			        z_new = accProcStatus.net_z_sum_buff[accProcStatus.net_idx];

			        x_old = accProcStatus.net_x_sum_buff[oldest_idx];
			        y_old = accProcStatus.net_y_sum_buff[oldest_idx];
			        z_old = accProcStatus.net_z_sum_buff[oldest_idx];

			        // Do approximation with 10-15% error instead of sqrt
			        norm_new = approx_norm(x_new, y_new, z_new);
			        norm_old = approx_norm(x_old, y_old, z_old);
			        delta = norm_new - norm_old;

			        if (delta > accNetThreshold)
			        {
			            if (!accProcStatus.hitDetected)
			            {
			                accProcStatus.hitDetected = true;
			                //if (acc_sys_cbk != NULL)  acc_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_HIT_DETECTED);
			            }
			        }
			    }

			    // goto next index
			    accProcStatus.net_idx = oldest_idx;

#endif /* NET_DETECTION_ENABLE */
			}
			else if (accProcStatus.move_detection_enabled != false)
			{
				if (++accProcStatus.move_detection_retry_cnt % 2 == 0u)
				{
					accProcStatus.move_detection_retry_cnt = 0u;
					/* Run movement detection logic every 200ms */
					if (calcVectorSpeed(*x_axis, *y_axis, *z_axis, 200u) != false)
					{
						if (acc_sys_cbk != NULL)
						{
							/* Movement detected */
							acc_sys_cbk(SYSTEM_EVT_READY, ACC_EVT_MOVE_DETECTED);
						}
					}
				}
			}
			/* reset a flag */
			accProcStatus.data_ready = false;
		}
		else
		{
			/* Skip a few first measurements */
			accProcStatus.init_skip_counts--;
			accProcStatus.data_ready = false;
		}
	}
}

void AccProc_Stop (void)
{
	/* Reset all data and SM */
	memset(&accProcStatus, 0u, sizeof(accProcStatus_t));

	/* Reset local variables */
	firstMeasure = 1u;

	/* Skip first measurements results */
	accProcStatus.init_skip_counts = MOVE_DETECT_INIT_DELAY_SKIP_CNT;

#if	LIS2DH12_ACC_ENABLE
	/* Deinit Accelerometer functionality */
	Lis2dh12_Deinit();
#endif
}

void AccProc_Reset (app_cbk_fn sys_cbk)
{
	memset(&accProcStatus, 0u, sizeof(accProcStatus_t));
	acc_sys_cbk = sys_cbk;

#if	LIS2DH12_ACC_ENABLE
	/* Init Accelerometer functionality */
	Lis2dh12_Reset(App_AccCbk, &x_axis, &y_axis, &z_axis);
#endif
}

void AccProc_HitDetectionStart (void)
{
	if ((accProcStatus.hit_detection_enabled == false) && (acc_sys_cbk != NULL))
	{
		/* Reset all data and SM */
		AccProc_Stop();
#if	LIS2DH12_ACC_ENABLE
		/* Set hit mode parameters */
		Lis2dh12_GotoHitMode();
#endif
	}
}

void AccProc_MoveDetectionStart (uint32_t move_threshold)
{
	if ((accProcStatus.move_detection_enabled == false) && (acc_sys_cbk != NULL))
	{
		/* Reset all data and SM */
		AccProc_Stop();
		/* Save movement threshold */
		accProcStatus.move_threshold = move_threshold;
#if	LIS2DH12_ACC_ENABLE
		/* Goto move detection mode */
		Lis2dh12_GotoMoveMode();
#endif
	}
}

void AccProc_Task ()
{
	AccProc_Processing();

#if LIS2DH12_ACC_ENABLE
	/* Accelerometer tasks*/
	Lis2dh12_Task();
#endif
}
