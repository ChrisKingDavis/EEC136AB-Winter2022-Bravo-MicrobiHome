#include "project.h"
#include "stdio.h"
void WDTIsr(void);
void handle_error(void);
#define WDT_IGNOREBITS      (2u)
static cy_stc_scb_i2c_master_xfer_config_t register_setting;

static uint8 rbuff[2];
static uint8 wbuff[2];

static void WaitForOperation() //Stock WaitForOperation method
{
  while(0 != (ACC1_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY)){}
    {
        CyDelayUs(1);
    }
}

static void WriteRegister(uint8 reg_addr, uint8 data) //Stock WriteRegister method
{
    wbuff[0] = reg_addr;
    wbuff[1] = data;
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    
    ACC1_MasterWrite(&register_setting);
    WaitForOperation();
}

static uint8 ReadRegister(uint8 reg_addr) //Stock ReadRegister method
{
    wbuff[0] = reg_addr;
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;
    
    ACC1_MasterWrite(&register_setting);
    WaitForOperation();
    
    register_setting.buffer = rbuff;
    register_setting.xferPending = false;
    
    ACC1_MasterRead(&register_setting);
    WaitForOperation();
    
    return rbuff[0];
}

int main(void)
{
    __enable_irq();
    setvbuf(stdin,NULL,_IONBF,0);
    register_setting.slaveAddress = (0x37);
    Cy_WDT_Enable();
//start conversions?
    //uint16 tempC = 0;
    float32 tempF = 0;
    //uint16 count = 0;
    //uint16 IntLevel = Cy_MCWDT_GetInterruptStatus(MCWDT_STRUCT0);
    //Cy_SysInt_Init(&INT1_cfg, IntHandler);
    //NVIC_ClearPendingIRQ(INT1_cfg.intrSrc);
    //NVIC_EnableIRQ(INT1_cfg.intrSrc);
    //Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 
    //__enable_irq();
    //__disable_irq();

 //   MCWDT_Start();
    for(;;)
    {
        
        //tempC = ReadRegister(0x00); 
        tempF = ((1.8*ReadRegister(0x00))+32);
        Cy_SysPm_SystemEnterUlp(); // Enter System Ultra-Low Power
        UART_Start();
        ACC1_Start();
        ADC_1_Start();
        ADC_1_StartConvert();
        Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS); 
        //Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS); 
        printf("Soil Sensor: %d\r\n", ADC_1_GetResult16(0));
        printf("Light Sensor: %d\r\n", ADC_1_GetResult16(1));
        printf("Temp Sensor: %f\r\n", tempF);
        //printf("Int Sensor: %i\r\n", Cy_WDT_GetCount());
        //printf("Int Sensor: %i\r\n", Cy_MCWDT_GetInterruptStatus(MCWDT_STRUCT0));
        if(ADC_1_GetResult16(0) >= 3000){
         Cy_GPIO_Set(Red_LED_PORT, Red_LED_NUM); /* toggle the led */   
        } else {
            Cy_GPIO_Clr(Red_LED_PORT, Red_LED_NUM); /* toggle the led */   
        }
        if(ADC_1_GetResult16(1) >= 3000){
         Cy_GPIO_Set(Green_LED_PORT, Green_LED_NUM); /* toggle the led */   
        } else {
            Cy_GPIO_Clr(Green_LED_PORT, Green_LED_NUM); /* toggle the led */   
        }
        if(tempF >= 64.9){
           Cy_GPIO_Set(Yellow_LED_PORT, Yellow_LED_NUM); /* toggle the led */   
        } else {
           Cy_GPIO_Clr(Yellow_LED_PORT, Yellow_LED_NUM); /* toggle the led */   
        }
        //Timing:
         /*if(count >= 20) {
            //Convert and Read
            UART_Start();
            ACC1_Start();
            ADC_1_Start();
            ADC_1_StartConvert();
            Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS); 
            //Delay
            CyDelay(100);
            //Reset count and resume
            count = 0;
        } else {
            count++;
            //ADC_1_Stop();
            //Cy_SAR_StopConvert(SAR); 
            //Cy_SysPm_CpuEnterDeepSleep();
        } */
        CyDelay(10);
    }
}