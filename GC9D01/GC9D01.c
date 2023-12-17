/*

  ******************************************************************************
  * @file 			( фаил ):   GC9D01.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	 author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

#include <GC9D01.h>


uint16_t GC9D01_X_Start = GC9D01_XSTART;	
uint16_t GC9D01_Y_Start = GC9D01_YSTART;

uint16_t GC9D01_Width = 0;
uint16_t GC9D01_Height = 0;

// массив буфер кадра
uint16_t buff_frame[GC9D01_WIDTH*GC9D01_HEIGHT] = { 0x0000, };


static void GC9D01_Unselect(void);
static void GC9D01_Select(void);
static void GC9D01_SendCmd(uint8_t Cmd);
static void GC9D01_SendData(uint8_t Data );
static void GC9D01_SendDataMASS(uint8_t* buff, size_t buff_size);
static void GC9D01_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static void GC9D01_RamWrite(uint16_t *pBuff, uint32_t Len);
static void GC9D01_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd);
static void GC9D01_RowSet(uint16_t RowStart, uint16_t RowEnd);
static void SwapInt16Values(int16_t *pValue1, int16_t *pValue2);
static void GC9D01_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

	
//##############################################################################
	  
	  
//==============================================================================
	  
	  
	  
//==============================================================================
// Процедура инициализации дисплея
//==============================================================================
void GC9D01_Init(void){
	
	GC9D01_Width = GC9D01_WIDTH;
	GC9D01_Height = GC9D01_HEIGHT;
	
  GC9D01_Select();

  GC9D01_HardReset(); 
	
	HAL_Delay(200);	
	
	//--- init ------------
	GC9D01_SendCmd(0xFE);
  GC9D01_SendCmd(0xEF);

	// Включение внутреннего регистра 80~8Fh -----
  GC9D01_SendCmd(0x80);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x81);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x82);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x83);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x84);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x85);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x86);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x87);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x88);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x89);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x8A);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x8B);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x8C);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x8D);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x8E);
  GC9D01_SendData(0xFF);

  GC9D01_SendCmd(0x8F);
  GC9D01_SendData(0xFF);
	//--------------------------------------
	
	// Установить режим переворота
  GC9D01_SendCmd(0xEC);
  GC9D01_SendData(0x11);

	// Размер VGL
  GC9D01_SendCmd(0x7E);
  GC9D01_SendData(0x7a);

	// Изменить частоту кадров
  GC9D01_SendCmd(0x74);
  GC9D01_SendData(0x02);
  GC9D01_SendData(0x0E);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x28);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
	
	// Внутренняя регулировка напряжения
  GC9D01_SendCmd(0x98);
  GC9D01_SendData(0x3E);
  GC9D01_SendCmd(0x99);
  GC9D01_SendData(0x3E);

	// Внутренние настройки porch
  GC9D01_SendCmd(0xB5);		//  Blanking Porch Control (B5h) VFP=14 VBP=14 HBP=Off
  GC9D01_SendData(0x0E);
  GC9D01_SendData(0x0E);

	// начало времени gip
  GC9D01_SendCmd(0x60);
  GC9D01_SendData(0x38);
  GC9D01_SendData(0x09);
  GC9D01_SendData(0x6D);
  GC9D01_SendData(0x67);


  GC9D01_SendCmd(0x63);
  GC9D01_SendData(0x38);
  GC9D01_SendData(0xAD);
  GC9D01_SendData(0x6D);
  GC9D01_SendData(0x67);
  GC9D01_SendData(0x05);


  GC9D01_SendCmd(0x64);
  GC9D01_SendData(0x38);
  GC9D01_SendData(0x0B);
  GC9D01_SendData(0x70);
  GC9D01_SendData(0xAB);
  GC9D01_SendData(0x6D);
  GC9D01_SendData(0x67);


  GC9D01_SendCmd(0x66);
  GC9D01_SendData(0x38);
  GC9D01_SendData(0x0F);
  GC9D01_SendData(0x70);
  GC9D01_SendData(0xAF);
  GC9D01_SendData(0x6d);
  GC9D01_SendData(0x67);

  GC9D01_SendCmd(0x6A);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);

  GC9D01_SendCmd(0x68);
  GC9D01_SendData(0x3B);
  GC9D01_SendData(0x08);
  GC9D01_SendData(0x04);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x04);
  GC9D01_SendData(0x64);
  GC9D01_SendData(0x67);



  GC9D01_SendCmd(0x6C);
  GC9D01_SendData(0x22);
  GC9D01_SendData(0x02);
  GC9D01_SendData(0x22);
  GC9D01_SendData(0x02);
  GC9D01_SendData(0x22);
  GC9D01_SendData(0x22);
  GC9D01_SendData(0x50);

  GC9D01_SendCmd(0x6E);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x07);
  GC9D01_SendData(0x01);
  GC9D01_SendData(0x13);
  GC9D01_SendData(0x11);

  GC9D01_SendData(0x0B);
  GC9D01_SendData(0x09);
  GC9D01_SendData(0x16);
  GC9D01_SendData(0x15);
  GC9D01_SendData(0x1D);
  GC9D01_SendData(0x1E);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);

  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x1E);

  GC9D01_SendData(0x1D);
  GC9D01_SendData(0x15);
  GC9D01_SendData(0x16);
  GC9D01_SendData(0x0A);
  GC9D01_SendData(0x0C);

  GC9D01_SendData(0x12);
  GC9D01_SendData(0x14);
  GC9D01_SendData(0x02);
  GC9D01_SendData(0x08);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x00);
	// окончание времени gip


	// Начинается настройка внутреннего напряжения
  GC9D01_SendCmd(0xA9);
  GC9D01_SendData(0x1B);

  GC9D01_SendCmd(0xA8);
  GC9D01_SendData(0x6B);

  GC9D01_SendCmd(0xA8);
  GC9D01_SendData(0x6D);

  GC9D01_SendCmd(0xA7);
  GC9D01_SendData(0x40);

  GC9D01_SendCmd(0xAD);
  GC9D01_SendData(0x47);


  GC9D01_SendCmd(0xAF);
  GC9D01_SendData(0x73);

  GC9D01_SendCmd(0xAF);
  GC9D01_SendData(0x73);

  GC9D01_SendCmd(0xAC);
  GC9D01_SendData(0x44);

  GC9D01_SendCmd(0xA3);
  GC9D01_SendData(0x6C);

		
  GC9D01_SendCmd(0xCB);
  GC9D01_SendData(0x00);

  GC9D01_SendCmd(0xCD);
  GC9D01_SendData(0x22);


  GC9D01_SendCmd(0xC2);
  GC9D01_SendData(0x10);

  GC9D01_SendCmd(0xC5);
  GC9D01_SendData(0x00);

  GC9D01_SendCmd(0xC6);
  GC9D01_SendData(0x0E);

  GC9D01_SendCmd(0xC7);
  GC9D01_SendData(0x1f);

  GC9D01_SendCmd(0xC8);
  GC9D01_SendData(0x0E);
	// Настройка внутреннего напряжения завершена

	// Выберите режим одиночных ворот
  GC9D01_SendCmd(0xbf);		//  Dual-Single Gate Select (BFh) 0=>Single gate
  GC9D01_SendData(0x00);

	//Корректировки, связанные с SOU
  GC9D01_SendCmd(0xF9);
  GC9D01_SendData(0x20);

	//регулировка напряжения vreg
  GC9D01_SendCmd(0x9b);
  GC9D01_SendData(0x3b);
  GC9D01_SendCmd(0x93);
  GC9D01_SendData(0x33);
  GC9D01_SendData(0x7f);
  GC9D01_SendData(0x00);

	//регулировка VGH/VGL CLK 70，71h
  GC9D01_SendCmd(0x70);
  GC9D01_SendData(0x0E);
  GC9D01_SendData(0x0f);
  GC9D01_SendData(0x03);
  GC9D01_SendData(0x0e);
  GC9D01_SendData(0x0f);
  GC9D01_SendData(0x03);

  GC9D01_SendCmd(0x71);
  GC9D01_SendData(0x0e);
  GC9D01_SendData(0x16);
  GC9D01_SendData(0x03);

	// Внутренняя регулировка напряжения
  GC9D01_SendCmd(0x91);
  GC9D01_SendData(0x0e);
  GC9D01_SendData(0x09);

	// регулировка напряжения vreg
  GC9D01_SendCmd(0xc3);
  GC9D01_SendData(0x2c);
  GC9D01_SendCmd(0xc4);
  GC9D01_SendData(0x1a);

	// gamma F1~F3h
  GC9D01_SendCmd(0xf0);		// SET_GAMMA1 (F0h)
  GC9D01_SendData(0x51);
  GC9D01_SendData(0x13);
  GC9D01_SendData(0x0c);
  GC9D01_SendData(0x06);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x2f);

  GC9D01_SendCmd(0xf2);		// SET_GAMMA3 (F2h)
  GC9D01_SendData(0x51);
  GC9D01_SendData(0x13);
  GC9D01_SendData(0x0c);
  GC9D01_SendData(0x06);
  GC9D01_SendData(0x00);
  GC9D01_SendData(0x33);

  GC9D01_SendCmd(0xf1);		// SET_GAMMA2 (F1h)
  GC9D01_SendData(0x3c);
  GC9D01_SendData(0x94);
  GC9D01_SendData(0x4f);
  GC9D01_SendData(0x33);
  GC9D01_SendData(0x34);
  GC9D01_SendData(0xCf);

  GC9D01_SendCmd(0xf3);		// SET_GAMMA4 (F3h)
  GC9D01_SendData(0x4d);
  GC9D01_SendData(0x94);
  GC9D01_SendData(0x4f);
  GC9D01_SendData(0x33);
  GC9D01_SendData(0x34);
  GC9D01_SendData(0xCf);
	
	// Scan direction
	GC9D01_SendCmd(GC9D01_DisplayFunctionControl);
	GC9D01_SendData(0x00);
	GC9D01_SendData(0x10); 
		
	// def rotation
  GC9D01_SendCmd(GC9D01_MADCTL);
  GC9D01_SendData(GC9D01_ROTATION);
	
	// ColorMode
	GC9D01_SendCmd(GC9D01_COLMOD);
	GC9D01_SendData(ColorMode_MCU_16bit & 0x77);
	
  GC9D01_SendCmd(0x11);	// Sleep Out Mode (11h) and delay(200)
		
  HAL_Delay(200);
		
  GC9D01_SendCmd(0x29);	// Display ON (29h) and delay(20)
  GC9D01_SendCmd(0x2C);	// Memory Write (2Ch) D=0

  GC9D01_Unselect();
	
	GC9D01_ClearFrameBuffer();

}
//==============================================================================


//==============================================================================
// Процедура управления SPI
//==============================================================================
static void GC9D01_Select(void) {
	
    #ifdef CS_PORT
	
			//-- если захотим переделать под HAL ------------------	
			#ifdef GC9D01_SPI_HAL
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
			#endif
			//-----------------------------------------------------
			
			//-- если захотим переделать под CMSIS  ---------------
			#ifdef GC9D01_SPI_CMSIS
				CS_GPIO_Port->BSRR = ( CS_Pin << 16 );
			#endif
			//-----------------------------------------------------
	#endif
	
}
//==============================================================================


//==============================================================================
// Процедура управления SPI
//==============================================================================
static void GC9D01_Unselect(void) {
	
    #ifdef CS_PORT
	
			//-- если захотим переделать под HAL ------------------	
			#ifdef GC9D01_SPI_HAL
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
			#endif
			//-----------------------------------------------------
			
			//-- если захотим переделать под CMSIS  ---------------
			#ifdef GC9D01_SPI_CMSIS
					 CS_GPIO_Port->BSRR = CS_Pin;
			#endif
			//-----------------------------------------------------
	
	#endif
	
}
//==============================================================================


//==============================================================================
// Процедура вывода цветного изображения на дисплей
//==============================================================================
void GC9D01_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
	
    if((x >= GC9D01_Width) || (y >= GC9D01_Height)){
		return;
	}
	
    if((x + w - 1) >= GC9D01_Width){
		return;
	}
	
    if((y + h - 1) >= GC9D01_Height){
		return;
	}
	
	// заполняем буфер кадра
	for( uint16_t i = 0; i < h; i++ ){
		for( uint16_t j = 0; j < w; j++ ){
			buff_frame[( y + i ) * GC9D01_Width + x + j] = *data;
			data++;
		}
	}
}
//==============================================================================


//==============================================================================
// Процедура аппаратного сброса дисплея (ножкой RESET)
//==============================================================================
void GC9D01_HardReset(void){

	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);	
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	
}
//==============================================================================


//==============================================================================
// Процедура отправки команды в дисплей
//==============================================================================
__inline static void GC9D01_SendCmd(uint8_t Cmd){	
		
	//-- если захотим переделать под HAL ------------------	
	#ifdef GC9D01_SPI_HAL
	
		 // pin DC LOW
		 HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
					 
		 HAL_SPI_Transmit(&GC9D01_SPI_HAL, &Cmd, 1, HAL_MAX_DELAY);
		 while(HAL_SPI_GetState(&GC9D01_SPI_HAL) != HAL_SPI_STATE_READY){};
				
		 // pin DC HIGH
		 HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
		 
	#endif
	//-----------------------------------------------------
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef GC9D01_SPI_CMSIS
		
		// pin DC LOW
		DC_GPIO_Port->BSRR = ( DC_Pin << 16 );
	
		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9D01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9D01_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};	
			
			// заполняем буфер передатчика 1 байт информации--------------
			*((__IO uint8_t *)&GC9D01_SPI_CMSIS->DR) = Cmd;
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9D01_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
				
			//Ждем, пока SPI освободится от предыдущей передачи
			//while((GC9D01_SPI_CMSIS->SR&SPI_SR_BSY)){};	

			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			// Enable SPI
			if((GC9D01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(GC9D01_SPI_CMSIS->SR & SPI_SR_TXP)){};		
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&GC9D01_SPI_CMSIS->TXDR )  = Cmd;
				
			// Ждать завершения передачи---------------
			while (!( GC9D01_SPI_CMSIS -> SR & SPI_SR_TXC )){};
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
		// pin DC HIGH
		DC_GPIO_Port->BSRR = DC_Pin;
	
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура отправки данных (параметров) в дисплей 1 BYTE
//==============================================================================
__inline static void GC9D01_SendData(uint8_t Data ){
	
	//-- если захотим переделать под HAL ------------------
	#ifdef GC9D01_SPI_HAL
	
		HAL_SPI_Transmit(&GC9D01_SPI_HAL, &Data, 1, HAL_MAX_DELAY);
		while(HAL_SPI_GetState(&GC9D01_SPI_HAL) != HAL_SPI_STATE_READY){};
		
	#endif
	//-----------------------------------------------------
	
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef GC9D01_SPI_CMSIS
		
		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9D01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}

			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9D01_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&GC9D01_SPI_CMSIS->DR) = Data;

			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9D01_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};

			// Ждем, пока не освободится буфер передатчика
			//while((GC9D01_SPI_CMSIS->SR&SPI_SR_BSY)){};	
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9D01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}

			SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(GC9D01_SPI_CMSIS->SR & SPI_SR_TXP)){};		
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&GC9D01_SPI_CMSIS->TXDR )  = Data;
				
			// Ждать завершения передачи---------------
			while (!( GC9D01_SPI_CMSIS -> SR & SPI_SR_TXC )){};
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура отправки данных (параметров) в дисплей MASS
//==============================================================================
__inline static void GC9D01_SendDataMASS(uint8_t* buff, size_t buff_size){
	
	//-- если захотим переделать под HAL ------------------
	#ifdef GC9D01_SPI_HAL
		
		if( buff_size <= 0xFFFF ){
			HAL_SPI_Transmit(&GC9D01_SPI_HAL, buff, buff_size, HAL_MAX_DELAY);
		}
		else{
			while( buff_size > 0xFFFF ){
				HAL_SPI_Transmit(&GC9D01_SPI_HAL, buff, 0xFFFF, HAL_MAX_DELAY);
				buff_size-=0xFFFF;
				buff+=0xFFFF;
			}
			HAL_SPI_Transmit(&GC9D01_SPI_HAL, buff, buff_size, HAL_MAX_DELAY);
		}
		
		while(HAL_SPI_GetState(&GC9D01_SPI_HAL) != HAL_SPI_STATE_READY){};

	#endif
	//-----------------------------------------------------
	
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef GC9D01_SPI_CMSIS	

		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9D01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			while( buff_size ){
				
			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9D01_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
					
				// передаем 1 байт информации--------------
				*((__IO uint8_t *)&GC9D01_SPI_CMSIS->DR) = *buff++;

				buff_size--;
			}
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (GC9D01_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
				
			// Ждем, пока не освободится буфер передатчика
			// while((GC9D01_SPI_CMSIS->SR&SPI_SR_BSY)){};
				
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((GC9D01_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}

			SET_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// GC9D01_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(GC9D01_SPI_CMSIS->SR & SPI_SR_TXP)){};		
			
			while( buff_size ){
		
				// передаем 1 байт информации--------------
				*((__IO uint8_t *)&GC9D01_SPI_CMSIS->TXDR )  = *buff++;
				
				// Ждать завершения передачи---------------
				while (!( GC9D01_SPI_CMSIS -> SR & SPI_SR_TXC )){};

				buff_size--;

			}
			
			// Disable SPI	
			//CLEAR_BIT(GC9D01_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура включения режима сна
//==============================================================================
void GC9D01_SleepModeEnter( void ){
	
	GC9D01_Select(); 
	
	GC9D01_SendCmd(GC9D01_SLPIN);
	
	GC9D01_Unselect();
	
	HAL_Delay(250);
}
//==============================================================================


//==============================================================================
// Процедура отключения режима сна
//==============================================================================
void GC9D01_SleepModeExit( void ){
	
	GC9D01_Select(); 
	
	GC9D01_SendCmd(GC9D01_SLPOUT);
	
	GC9D01_Unselect();
	
	HAL_Delay(250);
}
//==============================================================================


//==============================================================================
// Процедура включения/отключения режима частичного заполнения экрана
//==============================================================================
void GC9D01_InversionMode(uint8_t Mode){
	
  GC9D01_Select(); 
	
  if (Mode){
    GC9D01_SendCmd(GC9D01_INVON);
  }
  else{
    GC9D01_SendCmd(GC9D01_INVOFF);
  }
  
  GC9D01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура закрашивает экран цветом color
//==============================================================================
void GC9D01_FillScreen(uint16_t color){
	
  GC9D01_FillRect(0, 0,  GC9D01_Width, GC9D01_Height, color);
}
//==============================================================================


//==============================================================================
// Процедура очистки экрана - закрашивает экран цветом черный
//==============================================================================
void GC9D01_Clear(void){
	
  GC9D01_FillRect(0, 0,  GC9D01_Width, GC9D01_Height, 0);
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольника цветом color
//==============================================================================
void GC9D01_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
	
  if ((x >= GC9D01_Width) || (y >= GC9D01_Height)){
	  return;
  }
  
  if ((x + w) > GC9D01_Width){	  
	  w = GC9D01_Width - x;
  }
  
  if ((y + h) > GC9D01_Height){
	  h = GC9D01_Height - y;
  }
  
	// заполняем буфер кадра
	for( uint16_t i = 0; i < h; i++ ){
		for( uint16_t j = 0; j < w; j++ ){
			buff_frame[( y + i ) * GC9D01_Width + x + j] = ((color & 0xFF)<<8) | (color >> 8 );
		}
	}
	
}
//==============================================================================


//==============================================================================
// Процедура установка границ экрана для заполнения
//==============================================================================
static void GC9D01_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
	
	GC9D01_Select();
	
	GC9D01_ColumnSet(x0, x1);
	GC9D01_RowSet(y0, y1);
	
	// write to RAM
	GC9D01_SendCmd(GC9D01_RAMWR);

	GC9D01_Unselect();
	
}
//==============================================================================


//==============================================================================
// Процедура записи данных в дисплей
//==============================================================================
static void GC9D01_RamWrite(uint16_t *pBuff, uint32_t Len){
	
  GC9D01_Select();
	
  uint8_t buff[2];
  buff[0] = *pBuff >> 8;
  buff[1] = *pBuff & 0xFF;

  while (Len--){
	  GC9D01_SendDataMASS( buff, 2);
  } 

  GC9D01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура установки начального и конечного адресов колонок
//==============================================================================
static void GC9D01_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd){
	
  if (ColumnStart > ColumnEnd){
    return;
  }
  
  if (ColumnEnd > GC9D01_Width){
    return;
  }
	
  ColumnStart += GC9D01_X_Start;
  ColumnEnd += GC9D01_X_Start;
  
  GC9D01_SendCmd(GC9D01_CASET);
  GC9D01_SendData(ColumnStart >> 8);  
  GC9D01_SendData(ColumnStart & 0xFF);  
  GC9D01_SendData(ColumnEnd >> 8);  
  GC9D01_SendData(ColumnEnd & 0xFF);  
  
}
//==============================================================================


//==============================================================================
// Процедура установки начального и конечного адресов строк
//==============================================================================
static void GC9D01_RowSet(uint16_t RowStart, uint16_t RowEnd){
	
  if (RowStart > RowEnd){
    return;
  }
  
  if (RowEnd > GC9D01_Height){
    return;
  }
  
  RowStart += GC9D01_Y_Start;
  RowEnd += GC9D01_Y_Start;
 
  GC9D01_SendCmd(GC9D01_RASET);
  GC9D01_SendData(RowStart >> 8);  
  GC9D01_SendData(RowStart & 0xFF);  
  GC9D01_SendData(RowEnd >> 8);  
  GC9D01_SendData(RowEnd & 0xFF);  

}
//==============================================================================


//==============================================================================
// Процедура управления подсветкой (ШИМ)
//==============================================================================
void GC9D01_SetBL(uint8_t Value){
	
//  if (Value > 100)
//    Value = 100;

//	tmr2_PWM_set(ST77xx_PWM_TMR2_Chan, Value);

}
//==============================================================================


//==============================================================================
// Процедура включения/отключения питания дисплея
//==============================================================================
void GC9D01_DisplayPower(uint8_t On){
	
  GC9D01_Select(); 
	
  if (On){
    GC9D01_SendCmd(GC9D01_DISPON);
  }
  else{
    GC9D01_SendCmd(GC9D01_DISPOFF);
  }
  
  GC9D01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольника ( пустотелый )
//==============================================================================
void GC9D01_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	
  GC9D01_DrawLine(x1, y1, x1, y2, color);
  GC9D01_DrawLine(x2, y1, x2, y2, color);
  GC9D01_DrawLine(x1, y1, x2, y1, color);
  GC9D01_DrawLine(x1, y2, x2, y2, color);
	
}
//==============================================================================


//==============================================================================
// Процедура вспомогательная для --- Процедура рисования прямоугольника ( заполненый )
//==============================================================================
static void SwapInt16Values(int16_t *pValue1, int16_t *pValue2){
	
  int16_t TempValue = *pValue1;
  *pValue1 = *pValue2;
  *pValue2 = TempValue;
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольника ( заполненый )
//==============================================================================
void GC9D01_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor) {
	
  if (x1 > x2){
    SwapInt16Values(&x1, &x2);
  }
  
  if (y1 > y2){
    SwapInt16Values(&y1, &y2);
  }
  
  GC9D01_FillRect(x1, y1, x2 - x1, y2 - y1, fillcolor);
}
//==============================================================================


//==============================================================================
// Процедура вспомогательная для --- Процедура рисования линии
//==============================================================================
static void GC9D01_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	
  const int16_t deltaX = abs(x2 - x1);
  const int16_t deltaY = abs(y2 - y1);
  const int16_t signX = x1 < x2 ? 1 : -1;
  const int16_t signY = y1 < y2 ? 1 : -1;

  int16_t error = deltaX - deltaY;

  GC9D01_DrawPixel(x2, y2, color);

  while (x1 != x2 || y1 != y2) {
	  
    GC9D01_DrawPixel(x1, y1, color);
    const int16_t error2 = error * 2;
 
    if (error2 > -deltaY) {
		
      error -= deltaY;
      x1 += signX;
    }
    if (error2 < deltaX){
		
      error += deltaX;
      y1 += signY;
    }
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования линии
//==============================================================================
void GC9D01_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

  if (x1 == x2){

    if (y1 > y2){
      GC9D01_FillRect(x1, y2, 1, y1 - y2 + 1, color);
	}
    else{
      GC9D01_FillRect(x1, y1, 1, y2 - y1 + 1, color);
	}
	
    return;
  }
  
  if (y1 == y2){
    
    if (x1 > x2){
      GC9D01_FillRect(x2, y1, x1 - x2 + 1, 1, color);
	}
    else{
      GC9D01_FillRect(x1, y1, x2 - x1 + 1, 1, color);
	}
	
    return;
  }
  
  GC9D01_DrawLine_Slow(x1, y1, x2, y2, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования линии с указаным углом и длиной
//==============================================================================
void GC9D01_DrawLineWithAngle(int16_t x, int16_t y, uint16_t length, double angle_degrees, uint16_t color) {
    // Преобразование угла в радианы
    double angle_radians = (360.0 - angle_degrees) * PI / 180.0;

    // Вычисление конечных координат
    int16_t x2 = x + length * cos(angle_radians) + 0.5;
    int16_t y2 = y + length * sin(angle_radians) + 0.5;

    // Используем существующую функцию для рисования линии
    GC9D01_DrawLine(x, y, x2, y2, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования треугольника ( пустотелый )
//==============================================================================
void GC9D01_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color){
	/* Draw lines */
	GC9D01_DrawLine(x1, y1, x2, y2, color);
	GC9D01_DrawLine(x2, y2, x3, y3, color);
	GC9D01_DrawLine(x3, y3, x1, y1, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования треугольника ( заполненый )
//==============================================================================
void GC9D01_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color){
	
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;
	
	deltax = abs(x2 - x1);
	deltay = abs(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} 
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} 
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} 
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		GC9D01_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}
//==============================================================================


//==============================================================================
// Процедура окрашивает 1 пиксель дисплея
//==============================================================================
void GC9D01_DrawPixel(int16_t x, int16_t y, uint16_t color){
	
  if ((x < 0) ||(x >= GC9D01_Width) || (y < 0) || (y >= GC9D01_Height)){
    return;
  }
	
	// заполняем буфер кадра
	buff_frame[y * GC9D01_Width + x] = ((color & 0xFF)<<8) | (color >> 8 );
}
//==============================================================================


//==============================================================================
// Процедура рисования круг ( заполненый )
//==============================================================================
void GC9D01_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor) {
	
  int x = 0;
  int y = radius;
  int delta = 1 - 2 * radius;
  int error = 0;

  while (y >= 0){
	  
    GC9D01_DrawLine(x0 + x, y0 - y, x0 + x, y0 + y, fillcolor);
    GC9D01_DrawLine(x0 - x, y0 - y, x0 - x, y0 + y, fillcolor);
    error = 2 * (delta + y) - 1;

    if (delta < 0 && error <= 0) {
		
      ++x;
      delta += 2 * x + 1;
      continue;
    }
	
    error = 2 * (delta - x) - 1;
		
    if (delta > 0 && error > 0) {
		
      --y;
      delta += 1 - 2 * y;
      continue;
    }
	
    ++x;
    delta += 2 * (x - y);
    --y;
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования круг ( пустотелый )
//==============================================================================
void GC9D01_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	
  int x = 0;
  int y = radius;
  int delta = 1 - 2 * radius;
  int error = 0;

  while (y >= 0){
	  
    GC9D01_DrawPixel(x0 + x, y0 + y, color);
    GC9D01_DrawPixel(x0 + x, y0 - y, color);
    GC9D01_DrawPixel(x0 - x, y0 + y, color);
    GC9D01_DrawPixel(x0 - x, y0 - y, color);
    error = 2 * (delta + y) - 1;

    if (delta < 0 && error <= 0) {
		
      ++x;
      delta += 2 * x + 1;
      continue;
    }
	
    error = 2 * (delta - x) - 1;
		
    if (delta > 0 && error > 0) {
		
      --y;
      delta += 1 - 2 * y;
      continue;
    }
	
    ++x;
    delta += 2 * (x - y);
    --y;
  }
}
//==============================================================================


//==============================================================================
// рисуем элипс
//==============================================================================
void GC9D01_DrawEllipse(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color) {
    int x, y;
    for (float angle = 0; angle <= 360; angle += 0.1) {
        x = x0 + radiusX * cos(angle * PI / 180);
        y = y0 + radiusY * sin(angle * PI / 180);
        GC9D01_DrawPixel(x, y, color);
    }
}
//==============================================================================


//==============================================================================
// рисуем элипс под указаным углом наклона
//==============================================================================
void GC9D01_DrawEllipseWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color) {
    float cosAngle = cos((360.0 - angle_degrees) * PI / 180);
    float sinAngle = sin((360.0 - angle_degrees) * PI / 180);

    for (int16_t t = 0; t <= 360; t++) {
        float radians = t * PI / 180.0;
        int16_t x = radiusX * cos(radians);
        int16_t y = radiusY * sin(radians);

        int16_t xTransformed = x0 + cosAngle * x - sinAngle * y;
        int16_t yTransformed = y0 + sinAngle * x + cosAngle * y;

        GC9D01_DrawPixel(xTransformed, yTransformed, color);
    }
}
//==============================================================================


//==============================================================================
// рисуем элипс закрашенный
//==============================================================================
void GC9D01_DrawEllipseFilled(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color) {
	int x, y;

	for (y = -radiusY; y <= radiusY; y++) {
			for (x = -radiusX; x <= radiusX; x++) {
					if ((x * x * radiusY * radiusY + y * y * radiusX * radiusX) <= (radiusX * radiusX * radiusY * radiusY)) {
							GC9D01_DrawPixel(x0 + x, y0 + y, color);
					}
			}
	}
}
//==============================================================================


//==============================================================================
// рисуем элипс закрашенный под указаным углом наклона
//==============================================================================
void GC9D01_DrawEllipseFilledWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color) {
   float cosAngle = cos((360.0 - angle_degrees) * PI / 180.0);
    float sinAngle = sin((360.0 - angle_degrees) * PI / 180.0);

    for (int16_t y = -radiusY; y <= radiusY; y++) {
        for (int16_t x = -radiusX; x <= radiusX; x++) {
          float xTransformed = cosAngle * x - sinAngle * y;
          float yTransformed = sinAngle * x + cosAngle * y;

					if ((x * x * radiusY * radiusY + y * y * radiusX * radiusX) <= (radiusX * radiusX * radiusY * radiusY)){
             GC9D01_DrawPixel(x0 + xTransformed, y0  + yTransformed, color);
          }
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования символа ( 1 буква или знак )
//==============================================================================
void GC9D01_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch){
	
	uint32_t i, b, j;
	
	uint32_t X = x, Y = y;
	
	uint8_t xx, yy;
	
	if( multiplier < 1 ){
		multiplier = 1;
	}

	/* Check available space in LCD */
	if (GC9D01_Width >= ( x + Font->FontWidth) || GC9D01_Height >= ( y + Font->FontHeight)){

	
			/* Go through font */
			for (i = 0; i < Font->FontHeight; i++) {		
				
				if( ch < 127 ){			
					b = Font->data[(ch - 32) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch > 191 ){
					// +96 это так как латинские символы и знаки в шрифтах занимают 96 позиций
					// и если в шрифте который содержит сперва латиницу и спец символы и потом 
					// только кирилицу то нужно добавлять 95 если шрифт 
					// содержит только кирилицу то +96 не нужно
					b = Font->data[((ch - 192) + 96) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 168 ){	// 168 символ по ASCII - Ё
					// 160 эллемент ( символ Ё ) 
					b = Font->data[( 160 ) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 184 ){	// 184 символ по ASCII - ё
					// 161 эллемент  ( символ ё ) 
					b = Font->data[( 161 ) * Font->FontHeight + i];
				}
				//-------------------------------------------------------------------
				
				//----  Украинская раскладка ----------------------------------------------------
				else if( (uint8_t) ch == 170 ){	// 168 символ по ASCII - Є
					// 162 эллемент ( символ Є )
					b = Font->data[( 162 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 175 ){	// 184 символ по ASCII - Ї
					// 163 эллемент  ( символ Ї )
					b = Font->data[( 163 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 178 ){	// 168 символ по ASCII - І
					// 164 эллемент ( символ І )
					b = Font->data[( 164 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 179 ){	// 184 символ по ASCII - і
					// 165 эллемент  ( символ і )
					b = Font->data[( 165 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 186 ){	// 184 символ по ASCII - є
					// 166 эллемент  ( символ є )
					b = Font->data[( 166 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 191 ){	// 168 символ по ASCII - ї
					// 167 эллемент ( символ ї )
					b = Font->data[( 167 ) * Font->FontHeight + i];
				}
				//-----------------------------------------------------------------------------
			
				for (j = 0; j < Font->FontWidth; j++) {
					
					if ((b << j) & 0x8000) {
						
						for (yy = 0; yy < multiplier; yy++){
							for (xx = 0; xx < multiplier; xx++){
									GC9D01_DrawPixel(X+xx, Y+yy, TextColor);
							}
						}
						
					} 
					else if( TransparentBg ){
						
						for (yy = 0; yy < multiplier; yy++){
							for (xx = 0; xx < multiplier; xx++){
									GC9D01_DrawPixel(X+xx, Y+yy, BgColor);
							}
						}
						
					}
					X = X + multiplier;
				}
				X = x;
				Y = Y + multiplier;
			}
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования строки
//==============================================================================
void GC9D01_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str){	
	
	if( multiplier < 1 ){
		multiplier = 1;
	}
	
	unsigned char buff_char;
	
	uint16_t len = strlen(str);
	
	while (len--) {
		
		//---------------------------------------------------------------------
		// проверка на кириллицу UTF-8, если латиница то пропускаем if
		// Расширенные символы ASCII Win-1251 кириллица (код символа 128-255)
		// проверяем первый байт из двух ( так как UTF-8 ето два байта )
		// если он больше либо равен 0xC0 ( первый байт в кириллеце будет равен 0xD0 либо 0xD1 именно в алфавите )
		if ( (uint8_t)*str >= 0xC0 ){	// код 0xC0 соответствует символу кириллица 'A' по ASCII Win-1251
			
			// проверяем какой именно байт первый 0xD0 либо 0xD1---------------------------------------------
			switch ((uint8_t)*str) {
				case 0xD0: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x90 && (uint8_t)*str <= 0xBF){ buff_char = (*str) + 0x30; }	// байт символов А...Я а...п  делаем здвиг на +48
					else if ((uint8_t)*str == 0x81) { buff_char = 0xA8; break; }		// байт символа Ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x84) { buff_char = 0xAA; break; }		// байт символа Є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x86) { buff_char = 0xB2; break; }		// байт символа І ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x87) { buff_char = 0xAF; break; }		// байт символа Ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
				case 0xD1: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x80 && (uint8_t)*str <= 0x8F){ buff_char = (*str) + 0x70; }	// байт символов п...я	елаем здвиг на +112
					else if ((uint8_t)*str == 0x91) { buff_char = 0xB8; break; }		// байт символа ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x94) { buff_char = 0xBA; break; }		// байт символа є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x96) { buff_char = 0xB3; break; }		// байт символа і ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x97) { buff_char = 0xBF; break; }		// байт символа ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
			}
			//------------------------------------------------------------------------------------------------
			// уменьшаем еще переменную так как израсходывали 2 байта для кириллицы
			len--;
			
			GC9D01_DrawChar(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, buff_char);
		}
		//---------------------------------------------------------------------
		else{
			GC9D01_DrawChar(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, *str);
		}
		
		x = x + (Font->FontWidth * multiplier);
		/* Increase string pointer */
		str++;
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования символа с указаным углом ( 1 буква или знак )
//==============================================================================
void GC9D01_DrawCharWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, unsigned char ch){
	
	uint32_t i, b, j;
	
	uint32_t X = x, Y = y;
	
	uint8_t xx, yy;
	
	// Преобразуем угол в радианы
	double radians = (360.0 - angle_degrees) * PI / 180.0;

	// Вычисляем матрицу поворота
	double cosTheta = cos(radians);
	double sinTheta = sin(radians);

	// Переменные для преобразованных координат
	double newX, newY;
	
	if( multiplier < 1 ){
		multiplier = 1;
	}

	/* Check available space in LCD */
	if (GC9D01_Width >= ( x + Font->FontWidth) || GC9D01_Height >= ( y + Font->FontHeight)){

			/* Go through font */
			for (i = 0; i < Font->FontHeight; i++) {		
				
				if( ch < 127 ){			
					b = Font->data[(ch - 32) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch > 191 ){
					// +96 это так как латинские символы и знаки в шрифтах занимают 96 позиций
					// и если в шрифте который содержит сперва латиницу и спец символы и потом 
					// только кирилицу то нужно добавлять 95 если шрифт 
					// содержит только кирилицу то +96 не нужно
					b = Font->data[((ch - 192) + 96) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 168 ){	// 168 символ по ASCII - Ё
					// 160 эллемент ( символ Ё ) 
					b = Font->data[( 160 ) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 184 ){	// 184 символ по ASCII - ё
					// 161 эллемент  ( символ ё ) 
					b = Font->data[( 161 ) * Font->FontHeight + i];
				}
				//-------------------------------------------------------------------
				
				//----  Украинская раскладка ----------------------------------------------------
				else if( (uint8_t) ch == 170 ){	// 168 символ по ASCII - Є
					// 162 эллемент ( символ Є )
					b = Font->data[( 162 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 175 ){	// 184 символ по ASCII - Ї
					// 163 эллемент  ( символ Ї )
					b = Font->data[( 163 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 178 ){	// 168 символ по ASCII - І
					// 164 эллемент ( символ І )
					b = Font->data[( 164 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 179 ){	// 184 символ по ASCII - і
					// 165 эллемент  ( символ і )
					b = Font->data[( 165 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 186 ){	// 184 символ по ASCII - є
					// 166 эллемент  ( символ є )
					b = Font->data[( 166 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 191 ){	// 168 символ по ASCII - ї
					// 167 эллемент ( символ ї )
					b = Font->data[( 167 ) * Font->FontHeight + i];
				}
				//-----------------------------------------------------------------------------
			
				for (j = 0; j < Font->FontWidth; j++) {
					if ((b << j) & 0x8000) {
							// Применяем поворот к координатам
							newX = cosTheta * (X - x) - sinTheta * (Y - y) + x;
							newY = sinTheta * (X - x) + cosTheta * (Y - y) + y;

							for (yy = 0; yy < multiplier; yy++) {
									for (xx = 0; xx < multiplier; xx++) {
											GC9D01_DrawPixel(newX + xx, newY + yy, TextColor);
									}
							}
					} else if (TransparentBg) {
							// Аналогично для фона
							newX = cosTheta * (X - x) - sinTheta * (Y - y) + x + 0.5;
							newY = sinTheta * (X - x) + cosTheta * (Y - y) + y + 0.5;

							for (yy = 0; yy < multiplier; yy++) {
									for (xx = 0; xx < multiplier; xx++) {
											GC9D01_DrawPixel(newX + xx, newY + yy, BgColor);
									}
							}
					}
					X = X + multiplier;
				}
				X = x;
				Y = Y + multiplier;
			}
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования строки с указаным углом
//==============================================================================
void GC9D01_printWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, char *str){	
	
	if( multiplier < 1 ){
		multiplier = 1;
	}
	
	unsigned char buff_char;
	
	uint16_t len = strlen(str);
	
	while (len--) {
		
		//---------------------------------------------------------------------
		// проверка на кириллицу UTF-8, если латиница то пропускаем if
		// Расширенные символы ASCII Win-1251 кириллица (код символа 128-255)
		// проверяем первый байт из двух ( так как UTF-8 ето два байта )
		// если он больше либо равен 0xC0 ( первый байт в кириллеце будет равен 0xD0 либо 0xD1 именно в алфавите )
		if ( (uint8_t)*str >= 0xC0 ){	// код 0xC0 соответствует символу кириллица 'A' по ASCII Win-1251
			
			// проверяем какой именно байт первый 0xD0 либо 0xD1---------------------------------------------
			switch ((uint8_t)*str) {
				case 0xD0: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x90 && (uint8_t)*str <= 0xBF){ buff_char = (*str) + 0x30; }	// байт символов А...Я а...п  делаем здвиг на +48
					else if ((uint8_t)*str == 0x81) { buff_char = 0xA8; break; }		// байт символа Ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x84) { buff_char = 0xAA; break; }		// байт символа Є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x86) { buff_char = 0xB2; break; }		// байт символа І ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x87) { buff_char = 0xAF; break; }		// байт символа Ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
				case 0xD1: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x80 && (uint8_t)*str <= 0x8F){ buff_char = (*str) + 0x70; }	// байт символов п...я	елаем здвиг на +112
					else if ((uint8_t)*str == 0x91) { buff_char = 0xB8; break; }		// байт символа ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x94) { buff_char = 0xBA; break; }		// байт символа є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x96) { buff_char = 0xB3; break; }		// байт символа і ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x97) { buff_char = 0xBF; break; }		// байт символа ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
			}
			//------------------------------------------------------------------------------------------------
			// уменьшаем еще переменную так как израсходывали 2 байта для кириллицы
			len--;
			
			GC9D01_DrawCharWithAngle(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, angle_degrees, buff_char);
		}
		//---------------------------------------------------------------------
		else{
			GC9D01_DrawCharWithAngle(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, angle_degrees, *str);
		}
		// Смещаем начальные координаты с каждым символом с учетом угла
    x += (Font->FontWidth * multiplier * cos((360.0 - angle_degrees) * PI / 180.0) + 0.5);
    y += (Font->FontWidth * multiplier * sin((360.0 - angle_degrees) * PI / 180.0) + 0.5);

		/* Increase string pointer */
		str++;
	}
}
//==============================================================================


//==============================================================================
// Процедура ротации ( положение ) дисплея
//==============================================================================
// па умолчанию 1 режим ( всего 1, 2, 3, 4 )
void GC9D01_rotation( uint8_t rotation ){
	
	GC9D01_Select();
	
	GC9D01_SendCmd(GC9D01_MADCTL);

	// длайвер расчитан на экран 160 х 160 (  максимальный размер )
	// для подгона под любой другой нужно отнимать разницу пикселей

	  switch (rotation) {
		
		case 1:
			//== 0.99" 40 x 160 GC9D01 =================================================
			#ifdef GC9D01_IS_40X160
				GC9D01_SendData( GC9D01_MADCTL_MV | GC9D01_MADCTL_ML | GC9D01_MADCTL_RGB );
				GC9D01_Width = 160;
				GC9D01_Height = 40;
				GC9D01_X_Start = -60;
				GC9D01_Y_Start = 60;
				GC9D01_FillScreen(0);
			#endif
			//==========================================================================

		 break;
		
		case 2:
			//== 0.99" 40 x 160 GC9D01 =================================================
			#ifdef GC9D01_IS_40X160
				GC9D01_SendData( GC9D01_MADCTL_RGB );
				GC9D01_Width = 40;
				GC9D01_Height = 160;
				GC9D01_X_Start = 0;
				GC9D01_Y_Start = 0;
				GC9D01_FillScreen(0);
			#endif
			//==========================================================================
	
		 break;
		
	   case 3:
		   //== 0.99" 40 x 160 GC9D01 =================================================
			#ifdef GC9D01_IS_40X160
				GC9D01_SendData( GC9D01_MADCTL_MV | GC9D01_MADCTL_MX | GC9D01_MADCTL_MY | GC9D01_MADCTL_ML | GC9D01_MADCTL_RGB );
				GC9D01_Width = 160;
				GC9D01_Height = 40;
				GC9D01_X_Start = -60;
				GC9D01_Y_Start = 60;
				GC9D01_FillScreen(0);
			#endif
			//==========================================================================
			
		 break;
	   
	   case 4:
		   //== 0.99" 40 x 160 GC9D01 =================================================
			#ifdef GC9D01_IS_40X160
				GC9D01_SendData(GC9D01_MADCTL_MX | GC9D01_MADCTL_MY | GC9D01_MADCTL_RGB);
				GC9D01_Width = 40;
				GC9D01_Height = 160;
				GC9D01_X_Start = 0;
				GC9D01_Y_Start = 0;
				GC9D01_FillScreen(0);
			#endif
			//==========================================================================
	   
		 break;
	   
	   default:
		 break;
	  }
	  
	  GC9D01_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура рисования иконки монохромной
//==============================================================================
void GC9D01_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color){

    int16_t byteWidth = (w + 7) / 8; 	// Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++){
		
        for(int16_t i=0; i<w; i++){
			
            if(i & 7){
               byte <<= 1;
            }
            else{
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
			
            if(byte & 0x80){
							GC9D01_DrawPixel(x+i, y, color);
						}
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования иконки монохромной с указаным углом
//==============================================================================
void GC9D01_DrawBitmapWithAngle(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color, double angle_degrees) {
    // Преобразование угла в радианы
    double angle_radians = (360.0 - angle_degrees) * PI / 180.0;

    // Вычисление матрицы поворота
    double cosTheta = cos(angle_radians);
    double sinTheta = sin(angle_radians);

    // Ширина и высота повернутого изображения
    int16_t rotatedW = round(fabs(w * cosTheta) + fabs(h * sinTheta));
    int16_t rotatedH = round(fabs(h * cosTheta) + fabs(w * sinTheta));

    // Вычисление центральных координат повернутого изображения
    int16_t centerX = x + w / 2;
    int16_t centerY = y + h / 2;

    // Проходим по каждому пикселю изображения и рисуем его повернутым
    for (int16_t j = 0; j < h; j++) {
        for (int16_t i = 0; i < w; i++) {
            // Вычисление смещения от центра
            int16_t offsetX = i - w / 2;
            int16_t offsetY = j - h / 2;

            // Применение матрицы поворота
            int16_t rotatedX = round(centerX + offsetX * cosTheta - offsetY * sinTheta);
            int16_t rotatedY = round(centerY + offsetX * sinTheta + offsetY * cosTheta);

            // Проверка находится ли пиксель в пределах экрана
            if (rotatedX >= 0 && rotatedX < GC9D01_Width && rotatedY >= 0 && rotatedY < GC9D01_Height) {
                // Получение цвета пикселя из исходного изображения
                uint8_t byteWidth = (w + 7) / 8;
                uint8_t byte = (*(const unsigned char*)(&bitmap[j * byteWidth + i / 8]));
                if (byte & (0x80 >> (i & 7))) {
                    // Рисование пикселя на экране
                    GC9D01_DrawPixel(rotatedX, rotatedY, color);
                }
            }
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольник с закругленніми краями ( заполненый )
//==============================================================================
void GC9D01_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color) {
	
	int16_t max_radius = ((width < height) ? width : height) / 2; // 1/2 minor axis
  if (cornerRadius > max_radius){
    cornerRadius = max_radius;
	}
	
  GC9D01_DrawRectangleFilled(x + cornerRadius, y, x + cornerRadius + width - 2 * cornerRadius, y + height, color);
  // draw four corners
  GC9D01_DrawFillCircleHelper(x + width - cornerRadius - 1, y + cornerRadius, cornerRadius, 1, height - 2 * cornerRadius - 1, color);
  GC9D01_DrawFillCircleHelper(x + cornerRadius, y + cornerRadius, cornerRadius, 2, height - 2 * cornerRadius - 1, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования половины окружности ( правая или левая ) ( заполненый )
//==============================================================================
void GC9D01_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  int16_t px = x;
  int16_t py = y;

  delta++; // Avoid some +1's in the loop

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    if (x < (y + 1)) {
      if (corners & 1){
        GC9D01_DrawLine(x0 + x, y0 - y, x0 + x, y0 - y - 1 + 2 * y + delta, color);
			}
      if (corners & 2){
        GC9D01_DrawLine(x0 - x, y0 - y, x0 - x, y0 - y - 1 + 2 * y + delta, color);
			}
    }
    if (y != py) {
      if (corners & 1){
        GC9D01_DrawLine(x0 + py, y0 - px, x0 + py, y0 - px - 1 + 2 * px + delta, color);
			}
      if (corners & 2){
        GC9D01_DrawLine(x0 - py, y0 - px, x0 - py, y0 - px - 1 + 2 * px + delta, color);
			}
			py = y;
    }
    px = x;
  }
}
//==============================================================================																		

//==============================================================================
// Процедура рисования четверти окружности (закругление, дуга) ( ширина 1 пиксель)
//==============================================================================
void GC9D01_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color)
{
    int16_t f = 1 - radius ;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * radius;
    int16_t x = 0;
    int16_t y = radius;

    while (x <= y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
				
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (quadrantMask & 0x4) {
            GC9D01_DrawPixel(x0 + x, y0 + y, color);
            GC9D01_DrawPixel(x0 + y, y0 + x, color);;
        }
        if (quadrantMask & 0x2) {
			GC9D01_DrawPixel(x0 + x, y0 - y, color);
            GC9D01_DrawPixel(x0 + y, y0 - x, color);
        }
        if (quadrantMask & 0x8) {
			GC9D01_DrawPixel(x0 - y, y0 + x, color);
            GC9D01_DrawPixel(x0 - x, y0 + y, color);
        }
        if (quadrantMask & 0x1) {
            GC9D01_DrawPixel(x0 - y, y0 - x, color);
            GC9D01_DrawPixel(x0 - x, y0 - y, color);
        }
    }
}
//==============================================================================		

//==============================================================================
// Процедура рисования прямоугольник с закругленніми краями ( пустотелый )
//==============================================================================
void GC9D01_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color) {
	
	int16_t max_radius = ((width < height) ? width : height) / 2; // 1/2 minor axis
  if (cornerRadius > max_radius){
    cornerRadius = max_radius;
	}
	
  GC9D01_DrawLine(x + cornerRadius, y, x + cornerRadius + width -1 - 2 * cornerRadius, y, color);         // Top
  GC9D01_DrawLine(x + cornerRadius, y + height - 1, x + cornerRadius + width - 1 - 2 * cornerRadius, y + height - 1, color); // Bottom
  GC9D01_DrawLine(x, y + cornerRadius, x, y + cornerRadius + height - 1 - 2 * cornerRadius, color);         // Left
  GC9D01_DrawLine(x + width - 1, y + cornerRadius, x + width - 1, y + cornerRadius + height - 1 - 2 * cornerRadius, color); // Right
	
  // draw four corners
	GC9D01_DrawCircleHelper(x + cornerRadius, y + cornerRadius, cornerRadius, 1, color);
  GC9D01_DrawCircleHelper(x + width - cornerRadius - 1, y + cornerRadius, cornerRadius, 2, color);
	GC9D01_DrawCircleHelper(x + width - cornerRadius - 1, y + height - cornerRadius - 1, cornerRadius, 4, color);
  GC9D01_DrawCircleHelper(x + cornerRadius, y + height - cornerRadius - 1, cornerRadius, 8, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования линия толстая ( последний параметр толщина )
//==============================================================================
void GC9D01_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick) {
	const int16_t deltaX = abs(x2 - x1);
	const int16_t deltaY = abs(y2 - y1);
	const int16_t signX = x1 < x2 ? 1 : -1;
	const int16_t signY = y1 < y2 ? 1 : -1;

	int16_t error = deltaX - deltaY;

	if (thick > 1){
		GC9D01_DrawCircleFilled(x2, y2, thick >> 1, color);
	}
	else{
		GC9D01_DrawPixel(x2, y2, color);
	}

	while (x1 != x2 || y1 != y2) {
		if (thick > 1){
			GC9D01_DrawCircleFilled(x1, y1, thick >> 1, color);
		}
		else{
			GC9D01_DrawPixel(x1, y1, color);
		}

		const int16_t error2 = error * 2;
		if (error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		}
		if (error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		}
	}
}
//==============================================================================		


//==============================================================================
// линия толстая нужной длины и указаным углом поворота (0-360) ( последний параметр толшина )
//==============================================================================
void GC9D01_DrawLineThickWithAngle(int16_t x, int16_t y, int16_t length, double angle_degrees, uint16_t color, uint8_t thick) {
    double angleRad = (360.0 - angle_degrees) * PI / 180.0;
    int16_t x2 = x + (int16_t)(cos(angleRad) * length) + 0.5;
    int16_t y2 = y + (int16_t)(sin(angleRad) * length) + 0.5;

    GC9D01_DrawLineThick(x, y, x2, y2, color, thick);
}
//==============================================================================


//==============================================================================
// Процедура рисования дуга толстая ( часть круга )
//==============================================================================
void GC9D01_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick) {
	
    int16_t xLast = -1, yLast = -1;

    if (startAngle > endAngle) {
        // Рисование первой части дуги от startAngle до 360 градусов
        for (int16_t angle = startAngle; angle <= 360; angle += 2) {
            float angleRad = (float)(360 - angle) * PI / 180;
            int x = cos(angleRad) * radius + x0;
            int y = sin(angleRad) * radius + y0;

            if (xLast != -1 && yLast != -1) {
                if (thick > 1) {
                    GC9D01_DrawLineThick(xLast, yLast, x, y, color, thick);
                } else {
                    GC9D01_DrawLine(xLast, yLast, x, y, color);
                }
            }

            xLast = x;
            yLast = y;
        }

        // Рисование второй части дуги от 0 до endAngle
        for (int16_t angle = 0; angle <= endAngle; angle += 2) {
            float angleRad = (float)(360 - angle) * PI / 180;
            int x = cos(angleRad) * radius + x0;
            int y = sin(angleRad) * radius + y0;

            if (xLast != -1 && yLast != -1) {
                if (thick > 1) {
                    GC9D01_DrawLineThick(xLast, yLast, x, y, color, thick);
                } else {
                    GC9D01_DrawLine(xLast, yLast, x, y, color);
                }
            }

            xLast = x;
            yLast = y;
        }
    } else {
        // Рисование дуги от startAngle до endAngle
        for (int16_t angle = startAngle; angle <= endAngle; angle += 2) {
            float angleRad = (float)(360 - angle) * PI / 180;
            int x = cos(angleRad) * radius + x0;
            int y = sin(angleRad) * radius + y0;

            if (xLast != -1 && yLast != -1) {
                if (thick > 1) {
                    GC9D01_DrawLineThick(xLast, yLast, x, y, color, thick);
                } else {
                    GC9D01_DrawLine(xLast, yLast, x, y, color);
                }
            }

            xLast = x;
            yLast = y;
        }
    }
}
//==============================================================================



//==============================================================================
// Процедура вывода буффера кадра на дисплей
//==============================================================================
void GC9D01_Update(void){
	
		GC9D01_SetWindow(0, 0, GC9D01_Width-1, GC9D01_Height-1);
	
		GC9D01_Select();
	
		GC9D01_SendDataMASS((uint8_t*)buff_frame, sizeof(uint16_t)*GC9D01_Width*GC9D01_Height);
	
		GC9D01_Unselect();
}
//==============================================================================

//==============================================================================
// Процедура очистка только буфера кадра  ( при етом сам экран не очищаеться )
//==============================================================================
void GC9D01_ClearFrameBuffer(void){
	memset((uint8_t*)buff_frame, 0x00, GC9D01_Width*GC9D01_Height*sizeof(uint16_t) );
}
//==============================================================================





//#########################################################################################################################
//#########################################################################################################################


/************************ (C) COPYRIGHT GKP *****END OF FILE****/
