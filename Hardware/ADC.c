#include "stm32f10x.h"
extern uint16_t ADC_ConvertedValue[2];
void Init_adc(void)
{        
        ADC_InitTypeDef ADC_InitStructure;        
        DMA_InitTypeDef DMA_InitStructure;         
        GPIO_InitTypeDef GPIO_InitStructure;            
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//����DMAʱ�� 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//����ADC1ʱ��         
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//����������                     
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
				GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
				GPIO_Init(GPIOC,&GPIO_InitStructure);
        //DMA����
        DMA_DeInit(DMA1_Channel1);//DMA1ͨ��1����                
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40012400+0x4c;//�����ַ                        
        DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue;//�ڴ��ַ        
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//dma���䷽����                        
        DMA_InitStructure.DMA_BufferSize = 2;//����DMA�ڴ���ʱ�������ĳ���                
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//����DMA���������ģʽ��һ������                        
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//����DMA���ڴ����ģʽ                
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���������ֳ�        
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        //�ڴ������ֳ�        
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;        //����DMA�Ĵ���ģʽ���������ϵ�ѭ��ģʽ        
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;//����DMA�����ȼ���                        
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//����DMA��2��memory�еı����������        
        DMA_Init(DMA1_Channel1, &DMA_InitStructure);                        
        DMA_Cmd(DMA1_Channel1, ENABLE);//ʹ��ͨ��1         
        //ADC����        
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//��������ģʽ                        
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ɨ�跽ʽ        
        ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��                
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;        //�ⲿ������ֹ                
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�����Ҷ���                        
        ADC_InitStructure.ADC_NbrOfChannel = 2;//����ת����ͨ����        
        ADC_Init(ADC1, &ADC_InitStructure);                
        //����ģʽͨ������        
				ADC_RegularChannelConfig(ADC1,ADC_Channel_10,1,ADC_SampleTime_239Cycles5);
				ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_239Cycles5);                                       
        ADC_DMACmd(ADC1, ENABLE);//ʹ��ADC1��DMA                        
        ADC_Cmd(ADC1, ENABLE);//ʹ��ADC1                         
        ADC_ResetCalibration(ADC1);//ʹ��ADC1��λУ׼�Ĵ���                
        while(ADC_GetResetCalibrationStatus(ADC1));//���У׼�Ĵ����Ƿ�λ���                
        ADC_StartCalibration(ADC1);//��ʼУ׼                        
        while(ADC_GetCalibrationStatus(ADC1));//����Ƿ�У׼���                         
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);//����ADC1�����ת��
} 
