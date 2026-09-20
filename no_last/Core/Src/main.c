/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* =====================================================================
 *  РАЗМЕРЫ БУФЕРОВ
 * ===================================================================== */
#define SCOPE_BUFFER_SIZE      500    /** Отсчётов в полном кадре осциллографа */
#define SCOPE_DECIMATION_STEP  10     /** Отправляем каждую N-ю точку в UART */
#define UART_BUFFER_SIZE       32     /** Максимальная длина строки в send_tagged/parser */

/* =====================================================================
 *  МАСКИ ФЛАГОВ
 * ===================================================================== */

/* События от прерываний (statusFlag) */
#define FLAG_OS_READY   (1 << 0)   /** ADC1 заполнил OS_buffer */
#define FLAG_UART_READY (1 << 1)   /** Принята полная команда по UART */
#define FLAG_V_READY    (1 << 2)   /** ADC2 сделал замер для вольтметра */
#define FLAG_A_READY    (1 << 3)   /** ADC3 сделал замер для амперметра */

/* Состояния ВКЛ/ВЫКЛ приборов (onFlag) */
#define FLAG_OS_ON      (1 << 0)   /** Осциллограф активен */
#define FLAG_V_ON       (1 << 1)   /** Вольтметр активен */
#define FLAG_GEN_ON     (1 << 2)   /** Генератор сигналов активен */
#define FLAG_A_ON       (1 << 3)   /** Амперметр активен */
#define FLAG_LC_ON      (1 << 4)   /** LC-метр активен (блокирует OS и GEN) */

/* =====================================================================
 *  КАЛИБРОВОЧНЫЕ КОНСТАНТЫ
 * ===================================================================== */
/** Коэффициенты пересчёта кодов АЦП в мВ для диапазонов вольтметра */
#define MODE_V_0_VALUE 3300      /** Диапазон 1:1 → 3.31 В полный ход */
#define MODE_V_1_VALUE 19800     /** Диапазон 1:6 → 19.8 В полный ход */

#define MAX_ANALOG_VALUE 3300    /** Соответствие 4095 → 3310 мВ на входе */
#define MAX_ADC_VALUE    4095    /** 12-битный АЦП: максимум кода */

/* =====================================================================
 *  GPIO ПЕРЕКЛЮЧАТЕЛЕЙ ДИАПАЗОНОВ ВОЛЬТМЕТРА
 * ===================================================================== */
#define PORT_S1 GPIOF
#define PORT_S2 GPIOF
#define PORT_S3 GPIOF

#define PIN_S1  GPIO_PIN_3
#define PIN_S2  GPIO_PIN_5
#define PIN_S3  GPIO_PIN_10

/* =====================================================================
 *  ГЕНЕРАТОР СИГНАЛОВ
 * ===================================================================== */
#define MAX_WAVE_POINTS    256          /** Размер таблицы (гладкость + память) */
#define MIN_WAVE_POINTS    16           /** Меньше — форма теряет читаемость */
#define TIM_CLK            84000000UL   /** Тактовая TIM6, Гц */
#define DAC_MAX_UPDATE_HZ 1000000UL    /** Комфортный предел обновления ЦАП, Гц */

/**
 * @brief  Сопротивление шунта амперметра
 * @note   СМОТРИ БАГ №2 НИЖЕ — текущее значение не согласовано с формулой в A_metr
 */
#define A_RESISTOR_VALUE 3300

/* =====================================================================
 *  LC-МЕТР (резонансный метод)
 * ===================================================================== */
#define LC_F_START      1000UL       /** Нижняя граница развёртки, Гц */
#define LC_F_STOP       10000UL      /** Верхняя граница (потолок осциллографа), Гц */
#define LC_COARSE_STEP  500UL        /** Шаг грубого прохода, Гц */
#define LC_FINE_STEP    50UL         /** Шаг тонкого прохода, Гц */
#define LC_FINE_SPAN    500UL        /** Полуширина окна тонкого прохода, Гц */
#define LC_SCOPE_FS     100000UL     /** Дискретизация осциллографа на замере, Гц */
#define LC_CAP_F        100e-9f      /** Эталонная ёмкость контура, Ф */
#define LC_AVG_N        4            /** Число оборотов DMA для усреднения амплитуды */

/* =====================================================================
 *  ENUM-Ы
 * ===================================================================== */

/**
 * @brief  Диапазоны измерения вольтметра (переключаются через GPIO S1/S2/S3)
 */
typedef enum {
    V_RANGE_1_1 = 0,    /** 1:1 — до ~3.3 В */
    V_RANGE_1_6 = 1,    /** 1:6 — до ~20 В */
    COUNT_V_RANGES      /** Служебное: количество диапазонов (для bounds check) */
} VoltageRange;

/**
 * @brief  Формы сигналов генератора
 */
typedef enum {
    W_SINE = 0,         /** Синус */
    W_SQR,              /** Меандр */
    W_TRI,              /** Треугольник */
    W_SAW,              /** Пила */
    W_COUNT             /** Служебное: количество форм (для bounds check) */
} WaveForm;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
/* =====================================================================
 *  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ СОСТОЯНИЯ
 * ===================================================================== */

/**
 * @brief  Коэффициенты для преобразования кодов АЦП → мВ на разных диапазонах
 * @note   Индексируется через VoltageRange; коэффициенты в "мВ на 4095" (LSB)
 */
const uint32_t voltage_coeffs[COUNT_V_RANGES] = {
    MODE_V_0_VALUE,
    MODE_V_1_VALUE
};

VoltageRange V_range = V_RANGE_1_1;

/** Значение АЦП2 для вольтметра (обновляется из ISR ADC2) */
volatile uint32_t V_value = 0;

/** Значение АЦП3 для амперметра (обновляется из ISR ADC3) */
volatile uint32_t A_value = 0;

/**
 * @brief  Таблицы форм сигналов для ЦАП
 * @note   waves[W_XXX][i] — i-я точка формы XXX, 12-битное значение (0..4095).
 *         gen_points — текущая длина активного периода (<= MAX_WAVE_POINTS).
 */
uint16_t waves[W_COUNT][MAX_WAVE_POINTS];
uint32_t gen_points;

/**
 * @brief  Круговой буфер осциллографа (заполняется DMA от ADC1)
 * @note   volatile обязателен: DMA пишет независимо от CPU.
 *         На время отправки в UART (Os_metr) TIM3 останавливается,
 *         чтобы буфер не перезаписывался под рукой.
 */
volatile uint32_t OS_buffer[SCOPE_BUFFER_SIZE];

/**
 * @brief  Флаги событий от прерываний (ISR пишет, main loop читает и сбрасывает)
 * @note   volatile обязателен — ISR обновляет из другого контекста.
 *         Маски: FLAG_OS_READY, FLAG_UART_READY, FLAG_V_READY, FLAG_A_READY
 */
volatile uint8_t statusFlag = 0;

/**
 * @brief  Флаги ВКЛ/ВЫКЛ приборов
 * @note   Пишется и читается только из main loop, поэтому volatile не требуется.
 *         Маски: FLAG_OS_ON, FLAG_V_ON, FLAG_GEN_ON, FLAG_A_ON, FLAG_LC_ON.
 *         Взаимная блокировка: OS и LC и GEN не могут быть включены одновременно.
 */
uint8_t onFlag = 0;

/** Буфер для сборки исходящих UART-строк (используется в send_tagged) */
char uart_buf[UART_BUFFER_SIZE];

/**
 * @brief  Буфер приёма UART-команд
 * @note   volatile обязателен: ISR (HAL_UART_RxCpltCallback) пишет буфер независимо
 */
volatile char rx_buffer[UART_BUFFER_SIZE];
volatile uint8_t rx_index = 0;    /** Текущий индекс записи в rx_buffer */
uint8_t rx_byte;         /** Последний принятый байт (ISR → main) */

/**
 * @brief  Состояние машины состояний LC-развёртки
 * @note   Все переменные — internal (static), доступ только через LC_start/LC_metr
 */
static uint8_t  lc_pass;       /** 0 = грубый проход, 1 = тонкий */
static uint8_t  lc_phase;      /** 0 = смыв старых данных, 1 = чистый буфер */
static uint32_t lc_f;          /** Текущая частота генератора */
static uint32_t lc_best_a;     /** Минимальная амплитуда в текущем проходе */
static uint32_t lc_best_f;     /** Частота, на которой был найден минимум */
static uint32_t lc_coarse_f;   /** Грубый минимум (центр окна тонкого прохода) */
static uint32_t lc_acc;        /** Аккумулятор сумм амплитуд для усреднения */
static uint8_t  lc_n;          /** Счётчик оборотов для усреднения (0..LC_AVG_N-1) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void gen_set_freq(uint32_t hz);
void gen_fill_tables(void);
void gen_set_wave(WaveForm w);

void Os_metr(void);

void V_metr(void);
void V_set_range(uint8_t mode);

void A_metr(void);

int uint_to_string(uint32_t num, char *str);
void parser(void);
void On(void);

void send_bin_byte(UART_HandleTypeDef *huart, uint8_t val);
void send_tagged(char tag, uint32_t value);

void LC_metr(void);
void LC_start(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */


    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)OS_buffer, SCOPE_BUFFER_SIZE);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&V_value, 1);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&A_value, 1);

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    gen_fill_tables();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  	  if (statusFlag & FLAG_OS_READY) {
	  		  if (onFlag & FLAG_LC_ON) {
	  			  LC_metr();
	  		  } else if (onFlag & FLAG_OS_ON) {
	  			  Os_metr();
	  		  }
	  		  statusFlag &= ~FLAG_OS_READY;
	  	  }



	  	  if (statusFlag & FLAG_UART_READY) {
	  		  statusFlag &= ~FLAG_UART_READY;
	  		  parser();
	  	  }

	  	  if (statusFlag & FLAG_V_READY) {
	  		  statusFlag &= ~FLAG_V_READY;
	  		  if (onFlag & FLAG_V_ON) V_metr();
	  	  }

	  	  if (statusFlag & FLAG_A_READY) {
	  		  statusFlag &= ~FLAG_A_READY;
	  		  if (onFlag & FLAG_A_ON) A_metr();
	  	  }
	  	  __WFI();


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */
  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */
  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */
  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 15624;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 15624;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */
  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */
  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF3 PF5 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin PB8
                           PB9 */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Колбэк завершения преобразования АЦП
 * @note   Фиксирует готовность данных для соответствующих измерительных модулей:
 *          - ADC1: Осциллограф
 *          - ADC2: Вольтметр
 *          - ADC3: Амперметр
 * @param  hadc Указатель на структуру хэндлера АЦП, вызвавшего прерывание.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) statusFlag |= FLAG_OS_READY;
    else if (hadc->Instance == ADC2) statusFlag |= FLAG_V_READY;
    else if (hadc->Instance == ADC3) statusFlag |= FLAG_A_READY;
}

/**
 * @brief  Колбэк посимвольного приема данных из UART
 * @note   Сборка пакета идет в линейный буфер rx_buffer до появления символов '\r' или '\n'.
 * @warning Обработка флага FLAG_UART_READY должна сбрасывать его в основном цикле,
 *          иначе новые данные начнут перезаписывать буфер.
 * @param  huart Указатель на структуру хэндлера UART.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {

        // определение коньца строки по спец символам
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                statusFlag |= FLAG_UART_READY;
                rx_buffer[rx_index] = '\0';
                rx_index = 0;               // Сброс индекса для подготовки к следующему пакету
            }
        }
        // Накопление символов в буфере
        else {
            // Защита от переполнения буфера (оставляем 1 байт под '\0')
            if (rx_index < (sizeof(rx_buffer) - 1)) {
                rx_buffer[rx_index] = rx_byte;
                rx_index++;
            }

        }

        // Перезапуск прерывания на прием следующего байта
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}
/**
 * @brief  Расчет напряжения на осциллографе и отправка пакета данных в UART.
 * @note   Перед отправкой таймер TIM3 временно останавливается. Это предотвращает
 *         перезапись текущего буфера новыми данными от DMA во время трансляции.
 * @note   Данные прореживаются с шагом SCOPE_DECIMATION_STEP для снижения нагрузки на UART.
 */
void Os_metr(void) {
    HAL_TIM_Base_Stop(&htim3);

    for (int i = 0; i < SCOPE_BUFFER_SIZE; i += SCOPE_DECIMATION_STEP) {
        // Перевод сырых значений АЦП в милливольты
        uint32_t voltage_mv = (OS_buffer[i] * MAX_ANALOG_VALUE) / MAX_ADC_VALUE;

        send_tagged('O', voltage_mv);
    }

    HAL_TIM_Base_Start(&htim3);
}

/**
 * @brief  Заполнение таблиц форм сигнала генератора специальных сигналов.
 * @note   Генерирует таблицы отсчетов для 4 типов сигналов:
 *          - W_SINE: Синусоида
 *          - W_SQR:  Меандр (скважность 50%)
 *          - W_TRI:  Треугольный сигнал
 *          - W_SAW:  Пилообразный сигнал
 * @note   Расчеты завязаны на 12-битное разрешение встроенного ЦАП (диапазон значений 0..4095).
 */
void gen_fill_tables(void) {
    for (int i = 0; i < gen_points; i++) {
        float ph = (float)i / gen_points;   // Нормированная фаза (0.0..1.0) внутри одного периода

        // Синус: сдвиг на +1.0 и масштабирование в диапазон 0..4095
        waves[W_SINE][i] = (uint16_t)((sinf(2.0f * 3.14159265f * ph) + 1.0f) * 2047.5f);

        // Меандр: 50% скважность
        waves[W_SQR][i]  = (ph < 0.5f) ? 4095 : 0;

        // Треугольник: линейный рост до 0.5, затем линейный спад до 1.0
        waves[W_TRI][i]  = (uint16_t)(((1.0f - 4.0f * fabsf(ph - 0.5f)) + 1.0f) * 2047.5f);

        // Пила: линейный рост от 0 до 4095
        waves[W_SAW][i]  = (uint16_t)(ph * 4095.0f);
    }
}

/**
 * @brief  Динамическая установка частоты генератора
 * @note   Алгоритм адаптирует количество точек сигнала под целевую частоту
 *         и рассчитывает делители таймера TIM6 так, чтобы избежать переполнения 16-битного ARR.
 * @param  hz Требуемая частота выходного сигнала в Гц.
 */
void gen_set_freq(uint32_t hz) {
    // Защита от деления на ноль
    if (hz == 0) {
        hz = 1;
    }

    // Ограничение сверху: предотвращаем превышение максимальной скорости ЦАП
    // Максимальная частота сигнала ограничена скоростью обновления ЦАП и мин. кол-вом точек
    const uint32_t MAX_SIGNAL_HZ = DAC_MAX_UPDATE_HZ / MIN_WAVE_POINTS;
    if (hz > MAX_SIGNAL_HZ) {
        hz = MAX_SIGNAL_HZ;
    }

    // Расчет количества точек на период
    // Чем выше частота, тем меньше точек помещается в период при фиксированной скорости ЦАП
    gen_points = DAC_MAX_UPDATE_HZ / hz;

    // Ограничиваем количество точек разумными пределами для качества и памяти
    if (gen_points > MAX_WAVE_POINTS) {
        gen_points = MAX_WAVE_POINTS;
    }


    // Расчет параметров таймера
    uint32_t fs    = hz * gen_points;        // Реальная частота дискретизации (<= DAC_MAX_UPDATE_HZ)
    uint32_t ticks = TIM_CLK / fs;           // Тиков таймера на один отсчет ЦАП

    // TIM6 — 16-битный. Разбиваем большой счетчик на PSC (предделитель) и ARR (период)
    // PSC выбирается так, чтобы ARR поместился в 16 бит (<= 65535)
    uint32_t psc = ticks / 65536UL;

    TIM6->PSC = psc;
    // ARR = (общее число тиков / (psc+1)) - 1.
    // Вычитаем 1, т.к. счет идет от 0 до ARR (всего ARR+1 состояний)
    TIM6->ARR = (ticks / (psc + 1)) - 1;

    // Пересчитываем таблицу волновых форм, так как изменилось количество точек
    gen_fill_tables();
}


/**
 * @brief  Установка новой формы волны
 * @note   Останавливает текущий DMA и запускает его с новым буфером.
 * @param  w - объект enum (W_SINE, W_SQR, W_TRI, W_SAW)
 */
void gen_set_wave(WaveForm w) {

    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1,
    					(uint32_t*)waves[w],
						gen_points,
						DAC_ALIGN_12B_R);
}

/**
 * @brief  Установка новой частоты генератора (специально для режима LC-метра)
 * @note   Использует строго фиксированное количество точек (MAX_WAVE_POINTS).
 *         Это критично для быстродействия: исключает тяжелый пересчет
 *         таблицы отсчетов при каждом шаге сканирования частоты.
 * @note   Предназначена ТОЛЬКО для режима LC-метра.
 *         Для режима автономного генератора используйте адаптивный алгоритм.
 * @param  hz Требуемая частота синусоиды в Гц.
 */
static void gen_set_freq_fixed(uint32_t hz) {
    gen_points = MAX_WAVE_POINTS;

    uint32_t fs    = hz * gen_points;
    uint32_t ticks = TIM_CLK / fs;
    uint32_t psc   = ticks / 65536UL;

    // Обновляем регистры таймера.
    /* Таблица отсчетов НЕ пересчитывается, так как при фиксированном
       gen_points форма синусоиды математически идентична на любой частоте.*/
    TIM6->PSC = psc;
    TIM6->ARR = (ticks / (psc + 1)) - 1;

}

/**
 * @brief  Вычисление размаха синусоиды
 * @note   Алгоритм: берём минимумы/максимумы пар соседних точек,
 *         что отбрасывает одиночные иголки (они не повторяются в двух сэмплах подряд).
 *
 * @retval Размах сигнала в кодах АЦП (0..4095)
 */
static uint32_t lc_amplitude(void) {
    uint32_t robust_high = 0;
    uint32_t robust_low = 4095;

    for (uint32_t i = 0; i < SCOPE_BUFFER_SIZE - 1; i++) {
        uint32_t sample_curr = OS_buffer[i];
        uint32_t sample_next = OS_buffer[i + 1];

        // минимум пары — срезает высокую иголку
        uint32_t pair_min = (sample_curr < sample_next) ? sample_curr : sample_next;

        // максимум пары — срезает низкую иголку
        uint32_t pair_max = (sample_curr > sample_next) ? sample_curr : sample_next;

        // ищем самый высокий "устойчивый" пик и самую низкую "устойчивую" впадину
        if (pair_min > robust_high) robust_high = pair_min;
        if (pair_max < robust_low)  robust_low = pair_max;
    }

    return robust_high - robust_low;
}
/**
 * @brief  Вычисление индуктивности по резонансной частоте
 * @note   Работает в паре с lc_amplitude() и LC-развёрткой:
 *         после нахождения резонансной частоты считает индуктивность,
 *         используя известную эталонную ёмкость LC_CAP_F.
 * @note   Формула: L [мГн] = 1000 / (4π²f²C)
 * @param  f_hz Резонансная частота LC-контура, Гц
 * @retval Индуктивность катушки в миллиГенри (округлено до целого)
 */
static uint32_t lc_compute_mH(uint32_t f_hz) {
    float f = (float)f_hz;
    float l = 1000.0f / (4.0f * 3.14159265f * 3.14159265f * f * f * LC_CAP_F);

    // +0.5 перед отбрасыванием дробной части = математическое округление
    return (uint32_t)(l + 0.5f);
}


/**
 * @brief  Инициализация LC-метра перед развёрткой.
 * @note   Переводит систему в монопольный режим LC-измерения:
 *         - Отключает осциллограф и генератор (взаимная блокировка ресурсов)
 *         - Фиксирует частоту дискретизации АЦП на LC_SCOPE_FS
 *         - Сбрасывает состояние машины состояний
 *         - Запускает генератор на стартовой частоте
 * @note   После вызова main loop должен вызывать LC_metr() по событию FLAG_OS_READY.
 */
void LC_start(void) {
    /* LC монопольно забирает TIM3 (АЦП) и TIM6 (ЦАП): освобождаем общие ресурсы
       во избежание конфликтов периферии */
    onFlag &= ~FLAG_OS_ON;
    onFlag &= ~FLAG_GEN_ON;

    // Рассчитываем период таймера (ARR) исходя из базовой частоты тактирования шины TIM3.
    // Минус 1 компенсирует счет таймера от нуля.
    uint32_t new_arr = (1000000UL / LC_SCOPE_FS) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr);

    lc_pass   = 0;
    lc_f      = LC_F_START;
    lc_best_a = UINT32_MAX;   // Гарантирует, что первое же измерение перезапишет минимум
    lc_phase  = 0;            // Первый оборот после старта — смывной (срез переходных процессов)

    // Используем фиксированное максимальное количество точек для сохранения гладкости волны.
    // Это позволяет не перенастраивать DMA «на лету» во время развертки, исключая сбои тактирования.
    gen_set_freq_fixed(lc_f);
    gen_fill_tables();
    gen_set_wave(W_SINE);     // Синусоида обеспечивает наиболее чистый резонанс в контуре

    onFlag |= FLAG_LC_ON;
}

/**
 * @brief  Машина состояний LC-развёртки
 * @note   Вызывается из main loop по событию FLAG_OS_READY (каждые 5 мс при 100 кГц).
 *         Реализует двухпроходный поиск резонанса:
 *         - Coarse pass: сканирование диапазона с шагом LC_COARSE_STEP
 *         - Fine pass: уточнение вокруг минимума с шагом LC_FINE_STEP
 * @note   Фазовая модель:
 *         - lc_phase = 0: пропуск оборота DMA (flush transient)
 *         - lc_phase = 1: накопление LC_AVG_N измерений для усреднения
 *         - Завершение: сброс FLAG_LC_ON, отправка F и L
 * @note   На каждом шаге (кроме завершения) обновляет частоту генератора
 *         и переводит lc_phase = 0 для следующего flush-оборота
 */
void LC_metr(void) {
    if (!(onFlag & FLAG_LC_ON)) return;

    /* Смывной оборот: в буфере ещё смесь старого и нового сигнала
       после смены частоты — пропускаем без измерения */
    if (lc_phase == 0) {
        lc_phase = 1;
        lc_acc = 0;
        lc_n = 0;
        return;
    }

    // Копим LC_AVG_N оборотов и усредняем — давит шум и артефакты
    lc_acc += lc_amplitude();
    if (++lc_n < LC_AVG_N) return;

    uint32_t a = lc_acc / LC_AVG_N;

    // Последовательный резонанс = провал амплитуды, ищем МИНИМУМ
    if (a < lc_best_a) { lc_best_a = a; lc_best_f = lc_f; }

    if (lc_pass == 0) {
        if (lc_f + LC_COARSE_STEP <= LC_F_STOP) {
            lc_f += LC_COARSE_STEP;
        } else {
            /* Грубый проход завершён: запоминаем минимум и переходим
               к тонкому сканированию в окне ±LC_FINE_SPAN */
            lc_pass     = 1;
            lc_coarse_f = lc_best_f;
            /* UINT32_MAX гарантирует, что первый замер тонкого прохода
               сразу обновит минимум */
            lc_best_a   = UINT32_MAX;
            // Кламп снизу: не уйти ниже LC_F_START при отступе назад
            lc_f = (lc_coarse_f > LC_FINE_SPAN) ? lc_coarse_f - LC_FINE_SPAN
                                                : LC_F_START;
        }
    } else {
        if (lc_f + LC_FINE_STEP <= lc_coarse_f + LC_FINE_SPAN) {
            lc_f += LC_FINE_STEP;
        } else {
            // Развёртка завершена: выключаем LC, отдаём результат
            onFlag &= ~FLAG_LC_ON;
            send_tagged('F', lc_best_f);

            send_tagged('L', lc_compute_mH(lc_best_f));
            return;
        }
    }

    // Частота изменилась → следующий оборот смывной (переходные процессы)
    gen_set_freq_fixed(lc_f);
    lc_phase = 0;
}

/**
 * @brief  Расчет силы тока на амперметре и отправка данных в UART.
 * @note   Математика целочисленная. Для сохранения точности тока
 */
void A_metr(void){
	uint32_t shunt_mv = (A_value * MAX_ANALOG_VALUE) / MAX_ADC_VALUE;

	//Умножение на 1000 так как номинал резистора в миллиомах
	uint32_t current_ma = (shunt_mv * 1000)/A_RESISTOR_VALUE;
	send_tagged('a', shunt_mv);
    send_tagged('A', current_ma);
}



/**
 * @brief  Расчет напряжения на вольтметре и отправка данных в UART.
 */
void V_metr(void) {
    if (V_range >= COUNT_V_RANGES) V_range = V_RANGE_1_1;

    uint32_t voltage_mv = (V_value * voltage_coeffs[V_range]) / MAX_ADC_VALUE;
    send_tagged('v', V_value);
    send_tagged('V', voltage_mv);
}

/**
 * @brief  Установка аппаратного диапазона измерения вольтметра.
 * @note   Входные диапазоны аттенюатора:
 *          - V_RANGE_1_1: Без делителя (0 - 3.3 В)
 *          - V_RANGE_1_6: С делителем напряжения (0 - 19.8 В)
 * @param  mode Числовое значение целевого диапазона, приводимое к VoltageRange.
 */
void V_set_range(uint8_t mode) {
    // Защита от передачи некорректного индекса во избежание сбоя мультиплексора
    if (mode >= COUNT_V_RANGES) return;
    V_range = (VoltageRange)mode;

    /* Значения перечисления VoltageRange специально спроектированы так, чтобы их двоичный
       код (биты 0, 1, 2) напрямую соответствовал логическим уровням на управляющих пинах S1, S2, S3. */

    // Младшие 16 бит регистра BSRR устанавливают пин в 1, старшие 16 бит (сдвиг << 16) сбрасывают в 0.
    PORT_S1->BSRR = (V_range & 1) ? PIN_S1 : ((uint32_t)PIN_S1 << 16);
    PORT_S2->BSRR = (V_range & 2) ? PIN_S2 : ((uint32_t)PIN_S2 << 16);
    PORT_S3->BSRR = (V_range & 4) ? PIN_S3 : ((uint32_t)PIN_S3 << 16);
}


/**
 * @brief  Обновление состояния аппаратных таймеров в соответствии с флагами приборов.
 * @note   Эта функция синхронизирует логическое состояние оборудования с физическими
 *         выходами МК. Должна вызываться каждый раз после изменения `onFlag`.
 *
 * @details Взаимосвязь таймеров и модулей:
 *          - TIM2: Вольтметр
 *          - TIM3: Индуктивометр ИЛИ Осциллограф (общие цепи, гасится только при отключении обоих).
 *          - TIM6: Индуктивометр ИЛИ Генератор (аппаратная связь модулей, гасится только при отключении обоих).
 *          - TIM8: Амперметр.
 *
 * @see    parser
 */
void On(void) {

	//Модули зависят от триггера(TRGO) своих таймеров

    // Вольтметр
    if (onFlag & FLAG_V_ON) {
        HAL_TIM_Base_Start(&htim2);
    } else {
        HAL_TIM_Base_Stop(&htim2);
    }

    // TIM3 обслуживает и индуктивометр, и осциллограф.
    // Гасим таймер, только если выключены оба прибора.
    if ((onFlag & FLAG_LC_ON) || (onFlag & FLAG_OS_ON)) {
        HAL_TIM_Base_Start(&htim3);
    } else {
        HAL_TIM_Base_Stop(&htim3);
    }

    // TIM6 обслуживает генератор и индуктивометр

    if ((onFlag & FLAG_LC_ON) || (onFlag & FLAG_GEN_ON)) {
        HAL_TIM_Base_Start(&htim6);
    } else {
        HAL_TIM_Base_Stop(&htim6);
    }

    // Амперметр
    if (onFlag & FLAG_A_ON) {
        HAL_TIM_Base_Start(&htim8);
    } else {
        HAL_TIM_Base_Stop(&htim8);
    }

}

/**
 * @brief  Парсер входящих текстовых команд из UART.
 * @note   Принимает текстовые команды, обновляет флаги состояния onFlag
 *         и применяет настройки к периферии (таймеры, АЦП, ЦАП).
 *
 * @format Формат команд:
 *         1. Без параметров:  "<КОМАНДА> <ON|OFF>\r\n"
 *         2. С параметром:   "<КОМАНДА> <ЧИСЛО>\r\n"
 *
 * @example Примеры пакетов:
 *          "OS ON\r"       -> Включить осциллограф
 *          "OSFREQ 5000\n" -> Установить частоту осциллографа 5 кГц
 */
void parser(void) {
    char cmd[16];
    char sw[8];
    uint32_t param = 0;

    // Оставляем 1 байт под терминирующий нуль для защиты от переполнения буфера
    memset(cmd, 0, sizeof(cmd));
    memset(sw, 0, sizeof(sw));

    int i = 0, j = 0;

    // Выделение имени команды
    while (rx_buffer[i] != ' ' && rx_buffer[i] != '\0' && rx_buffer[i] != '\n' && rx_buffer[i] != '\r') {
        if (j < (sizeof(cmd) - 1)) cmd[j++] = rx_buffer[i];
        i++;
    }
    cmd[j] = '\0';

    // Пропуск разделителей
    while (rx_buffer[i] == ' ' || rx_buffer[i] == '\t') i++;

    j = 0;
    int is_number = 0;

    // Парсинг аргумента: либо число (param), либо строка-переключатель (sw)
    if (rx_buffer[i] >= '0' && rx_buffer[i] <= '9') {
        is_number = 1;
        while (rx_buffer[i] >= '0' && rx_buffer[i] <= '9') {
            param = param * 10 + (rx_buffer[i] - '0');
            i++;
        }
    } else {
        while (rx_buffer[i] != ' ' && rx_buffer[i] != '\0' && rx_buffer[i] != '\n' && rx_buffer[i] != '\r') {
            if (j < (sizeof(sw) - 1)) sw[j++] = rx_buffer[i];
            i++;
        }
        sw[j] = '\0';
    }



    // Включение осциллографа монопольно отключает LC-метр (аппаратное ограничение)
    if (strcmp(cmd, "OS") == 0) {
        if (strcmp(sw, "ON") == 0) {
            onFlag |= FLAG_OS_ON;
            onFlag &= ~FLAG_LC_ON;
        }
        else if (strcmp(sw, "OFF") == 0) {
            onFlag &= ~FLAG_OS_ON;
        }
    }
    // Генератор и LC-метр также используют общие цепи и взаимоисключают друг друга
    else if (strcmp(cmd, "GEN") == 0) {
        if (strcmp(sw, "ON") == 0) {
            onFlag |= FLAG_GEN_ON;
            onFlag &= ~FLAG_LC_ON;
        }
        else if (strcmp(sw, "OFF") == 0) {
            onFlag &= ~FLAG_GEN_ON;
        }
    }
    // LC-метр при старте сам захватывает и настраивает цепи генератора и осциллографа
    else if (strcmp(cmd, "L") == 0) {
        if (strcmp(sw, "ON") == 0) {
            LC_start();
        }
        else if (strcmp(sw, "OFF") == 0) {
            onFlag &= ~FLAG_LC_ON;
        }
    }
    // Вольтметр
    else if (strcmp(cmd, "V") == 0) {
        if (strcmp(sw, "ON") == 0) onFlag |= FLAG_V_ON;
        else if (strcmp(sw, "OFF") == 0) onFlag &= ~FLAG_V_ON;
    }
    // Амперметр
    else if (strcmp(cmd, "A") == 0) {
        if (strcmp(sw, "ON") == 0) onFlag |= FLAG_A_ON;
        else if (strcmp(sw, "OFF") == 0) onFlag &= ~FLAG_A_ON;
    }
    // Изменение частоты осциллографа
    else if (strcmp(cmd, "OSFREQ") == 0 && is_number) {
        // Ограничение 1 МГц обусловлено пределом стабильной дискретизации встроенного АЦП
        if (param > 0 && param <= 1000000 && !(onFlag & FLAG_LC_ON)) {
            uint32_t new_arr = (1000000 / param) - 1;
            HAL_UART_Transmit(&huart3, (uint8_t*)&param, sizeof(param), 100);
            __HAL_TIM_SET_AUTORELOAD(&htim3, new_arr);
        }
    }
    // Выбор частоты работы генератора специальных сигналов
    else if (strcmp(cmd, "GENFREQ") == 0 && is_number) {

        /* Максимальная частота ограничена 62.5 кГц
           так как при меньшем количестве точек форма сигнала полностью теряется.*/
        if (param > 0 && param <= 62500) {
            gen_set_freq(param);
        }
    }

    // Форма сигнала генератора
    else if (strcmp(cmd, "SETWAVEFORM") == 0 && is_number) {
        if (param <= W_COUNT) {
            gen_set_wave((WaveForm)param);
        }
    }
    // Диапазон вольтметра
    else if (strcmp(cmd, "VRANGE") == 0 && is_number) {
        V_set_range(param);
        send_bin_byte(&huart3, V_range);
    }

    // Применяем обновленный массив флагов к аппаратным таймерам
    On();
}
/**
 * @brief  Преобразование unsigned 32-битного числа в ASCII-строку (без ведущего нуля)
 * @param  num Число для преобразования
 * @param  str Выходной буфер (должен вмещать до 11 байт: 10 цифр + '\0')
 * @retval Длина записанной строки (без учёта '\0')
 * @note   Используется внутри send_tagged() вместо itoa/stdio,
 *         чтобы не тянуть в прошивку тяжёлый libc
 */
int uint_to_string(uint32_t num, char *str) {
    int i = 0;

    /* Отдельный разбор нуля: основной цикл while(num > 0) для нуля просто не выполнится */
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return i;
    }

    /* Извлекаем цифры с конца (младшие разряды первыми) —
       поэтому строка получается перевёрнутой */
    while (num > 0) {
        str[i++] = (num % 10) + '0';
        num /= 10;
    }
    str[i] = '\0';

    /* Разворот на месте: меняем местами символы от краёв к центру,
       превращая "4321" в "1234" */
    int start = 0, end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }

    return i;
}

/**
 * @brief  Отправка байта в виде 8 ASCII-символов '0'/'1' (старший бит первым)
 * @param  huart Handle UART-порта
 * @param  val Байт для отправки
 * @note   Используется для отладочной выдачи onFlag/V_range — удобно
 *         смотреть состояние флагов прямо в терминале
 */
void send_bin_byte(UART_HandleTypeDef *huart, uint8_t val) {
    char buf[9];

    /* i = индекс бита (7→0, т.е. от старшего к младшему),
       7-i = индекс в буфере, чтобы старший бит попал в buf[0] */
    for (int i = 7; i >= 0; i--) {
        buf[7 - i] = (val & (1 << i)) ? '1' : '0';
    }

    buf[8] = '\0';
    HAL_UART_Transmit(huart, (uint8_t*)buf, 8, HAL_MAX_DELAY);
}

/**
 * @brief  Отправка тегированного значения: "T value\r\n"
 * @param  tag Односимвольный тег канала (O, V, A, F, L и т.д.)
 * @param  value Числовое значение, которое будет преобразовано в десятичную строку
 * @note   Формат строки: "<tag> <value>\r\n" — такой формат легко парсить
 *         на стороне SerialPlot/ПК по первому символу
 * @note   Использует глобальный uart_buf — не потокобезопасна
 */
void send_tagged(char tag, uint32_t value) {
    /* Сначала получаем строку с числом в начале uart_buf */
    int len = uint_to_string(value, uart_buf);

    /* Сдвиг строки на 2 позиции вправо (освобождаем место для "T "),
       идём с конца к началу, чтобы не затирать не перенесённые данные */
    for (int j = len; j >= 0; j--) {
        uart_buf[j + 2] = uart_buf[j];
    }

    uart_buf[0] = tag;
    uart_buf[1] = ' ';

    /* После сдвига '\0' стоит в uart_buf[len+2] — заменяем его на \r
       и добавляем \n */
    uart_buf[len + 2] = '\r';
    uart_buf[len + 3] = '\n';
    uart_buf[len + 4] = '\0';

    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, len + 4, 100);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
