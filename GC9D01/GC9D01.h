/*
  ******************************************************************************
  * @file 			( фаил ):   GC9D01.h
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	 author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
 */
 
 
#ifndef _GC9D01_H
#define _GC9D01_H


/* C++ detection */
#ifdef __cplusplus
extern C {
#endif

// Обязательно нужен #include "main.h" 
// чтоб отдельно не подключать файлы связанные с МК и стандартными библиотеками

#include "main.h"
#include "fonts.h"

#include "stdlib.h"
#include "string.h"
#include "math.h"



//#######  SETUP  ##############################################################################################
		
		//>>>>>>>>>  данная библиотека работает только с буфером кадра ( нужно много оперативки ) <<<<<<<<<<<<<<
		
		//==== выбераем через что будем отправлять через HAL или CMSIS(быстрее) ==================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
			// указываем порт SPI для CMSIS ( быстро )-------
			// так как у разных МК разные регистры то в функциях корректируем под свой МК
			// на данный момент есть реализация на серию F1 F4 H7 для выбора серии в функциях
			//	void GC9D01_SendCmd(uint8_t Cmd);
			//	void GC9D01_SendData(uint8_t Data );
			//	void GC9D01_SendDataMASS(uint8_t* buff, size_t buff_size);	
			// комментируем и раскомментируем то что нам нужно, также там же редактируем под свой МК если не работает
			//#define 	GC9D01_SPI_CMSIS 	SPI2
			//-----------------------------------------------
			
			// указываем порт SPI для HAL ( медлено )--------
			#define 	GC9D01_SPI_HAL 		hspi1
			//-----------------------------------------------
			
		//============================================================================
					
		//=== указываем порты ( если в кубе назвали их DC RES CS то тогда нечего указывать не нужно )
		#if defined (DC_GPIO_Port)
		#else
			#define DC_GPIO_Port	GPIOC
			#define DC_Pin			GPIO_PIN_5
		#endif
		
		#if defined (RST_GPIO_Port)
		#else
			#define RST_GPIO_Port   GPIOB
			#define RST_Pin			GPIO_PIN_14
		#endif
		
		//--  Cесли используем порт CS для выбора устройства тогда раскомментировать ------------
		// если у нас одно устройство лучше пин CS притянуть к земле( или на порту подать GND )
		
		#define CS_PORT
		
		//----------------------------------------------------------------------------------------
		#ifdef CS_PORT
			#if defined (CS_GPIO_Port)
			#else
				#define CS_GPIO_Port    GPIOB
				#define CS_Pin			GPIO_PIN_12
			#endif
		#endif
		
		//=============================================================================
		
		//==  выбираем дисплей: =======================================================
		//-- нужное оставляем другое коментируем ( важно должно быть только один выбран )---------
		
		#define	GC9D01_IS_40X160		// 0.99" 40 x 160 GC9D01 		
		
		//=============================================================================
		
		
//##############################################################################################################

#ifdef GC9D01_SPI_HAL
	extern SPI_HandleTypeDef GC9D01_SPI_HAL;
#endif

extern uint16_t GC9D01_Width, GC9D01_Height;

extern uint16_t GC9D01_X_Start;
extern uint16_t GC9D01_Y_Start;

#define RGB565(r, g, b)         (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define PI 	3.14159265

//--- готовые цвета ------------------------------
#define   	GC9D01_BLACK   			0x0000
#define   	GC9D01_BLUE    			0x001F
#define   	GC9D01_RED     			0xF800
#define   	GC9D01_GREEN   			0x07E0
#define 		GC9D01_CYAN    			0x07FF
#define 		GC9D01_MAGENTA 			0xF81F
#define 		GC9D01_YELLOW  			0xFFE0
#define 		GC9D01_WHITE   			0xFFFF
//------------------------------------------------


#define GC9D01_MADCTL_MY  				0x80
#define GC9D01_MADCTL_MX  				0x40
#define GC9D01_MADCTL_MV  				0x20
#define GC9D01_MADCTL_ML  				0x10
#define GC9D01_MADCTL_RGB 				0x00
#define GC9D01_MADCTL_BGR 				0x08
#define GC9D01_MADCTL_MH  				0x04
//-------------------------------------------------


#define GC9D01_SWRESET 						0x01
#define GC9D01_SLPIN   						0x10
#define GC9D01_SLPOUT  						0x11
#define GC9D01_NORON   						0x13
#define GC9D01_INVOFF  						0x20
#define GC9D01_INVON   						0x21
#define GC9D01_DISPOFF 						0x28
#define GC9D01_DISPON  						0x29
#define GC9D01_CASET   						0x2A
#define GC9D01_RASET   						0x2B
#define GC9D01_RAMWR   						0x2C
#define GC9D01_COLMOD  						0x3A
#define GC9D01_MADCTL  						0x36
#define GC9D01_RAMWR_CONT       	0x3C
//-----------------------------------------------


//==============================================================================
// Значения, передаваемые аргументом с командой GC9D01_COLMOD
#define ColorMode_RGB_16bit  			0x50
#define ColorMode_RGB_18bit  			0x60
#define ColorMode_MCU_12bit  			0x03
#define ColorMode_MCU_16bit  			0x05
#define ColorMode_MCU_18bit  			0x06


#define GC9D01_DisplayFunctionControl    	0xB6



// длайвер расчитан на экран 160 х 160 (  максимальный размер )

//###  параметры дисплея 0.99" 40 x 160 GC9D01 ###################################
// 0.99" 40 x 160 GC9D01  display, default orientation
#ifdef GC9D01_IS_40X160
	
	#define GC9D01_WIDTH  			160
	#define GC9D01_HEIGHT 			40
	#define GC9D01_XSTART 			-60
	#define GC9D01_YSTART 			60
	#define GC9D01_ROTATION 		(GC9D01_MADCTL_MV | GC9D01_MADCTL_ML | GC9D01_MADCTL_RGB)
	
#endif
	
	
//##############################################################################

void GC9D01_Init(void);
void GC9D01_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);	
void GC9D01_HardReset(void);
void GC9D01_SleepModeEnter( void );
void GC9D01_SleepModeExit( void );
void GC9D01_ColorModeSet(uint8_t ColorMode);
void GC9D01_MemAccessModeSet(uint8_t Rotation, uint8_t VertMirror, uint8_t HorizMirror, uint8_t IsBGR);
void GC9D01_InversionMode(uint8_t Mode);
void GC9D01_FillScreen(uint16_t color);
void GC9D01_Clear(void);
void GC9D01_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void GC9D01_SetBL(uint8_t Value);
void GC9D01_DisplayPower(uint8_t On);
void GC9D01_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void GC9D01_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor);
void GC9D01_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void GC9D01_DrawLineWithAngle(int16_t x, int16_t y, uint16_t length, double angle_degrees, uint16_t color);
void GC9D01_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void GC9D01_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void GC9D01_DrawPixel(int16_t x, int16_t y, uint16_t color);
void GC9D01_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor);
void GC9D01_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
void GC9D01_DrawEllipse(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void GC9D01_DrawEllipseFilled(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color);
void GC9D01_DrawEllipseFilledWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void GC9D01_DrawEllipseWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color);
void GC9D01_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch);
void GC9D01_DrawCharWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, unsigned char ch);
void GC9D01_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str);
void GC9D01_printWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, char *str);
void GC9D01_rotation( uint8_t rotation );
void GC9D01_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);
void GC9D01_DrawBitmapWithAngle(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color, double angle_degrees);
void GC9D01_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color);
void GC9D01_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void GC9D01_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void GC9D01_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color);
void GC9D01_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick);
void GC9D01_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick);
void GC9D01_DrawLineThickWithAngle(int16_t x, int16_t y, int16_t length, double angle_degrees, uint16_t color, uint8_t thick);

void GC9D01_Update(void);
void GC9D01_ClearFrameBuffer(void);



/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif	/*	_GC9D01_H */

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
