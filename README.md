# g36pqwd-display-
U8g2, FreeRTOS

/* modiff main.c
begin */
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	/* STM32 supports HW SPI, Remove unused cases like U8X8_MSG_DELAY_XXX & U8X8_MSG_GPIO_XXX */
	switch(msg)
	{
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		/* Insert codes for initialization */
		break;
	case U8X8_MSG_DELAY_MILLI:
		/* ms Delay */
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_CS:
		/* Insert codes for SS pin control */
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_DC:
		/* Insert codes for DC pin control */
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_RESET:
		/* Insert codes for RST pin control */
		HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, arg_int);
		break;
	}
	return 1;
}
uint8_t u8x8_byte_stm32_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch(msg) {
	case U8X8_MSG_BYTE_SEND:
		/* Insert codes to transmit data SPI_SR_BSY */
		while(!(SPI1->SR & SPI_SR_TXE)){};
		if(HAL_SPI_Transmit_DMA(&hspi1, arg_ptr, arg_int) != HAL_OK){
			  Error_Handler();
		}
		while (SPI1->SR & SPI_SR_BSY){};
		break;
	case U8X8_MSG_BYTE_INIT:
		/* Insert codes to begin SPI transmission */
		break;
	case U8X8_MSG_BYTE_SET_DC:
		/* Control DC pin, U8X8_MSG_GPIO_DC will be called */
		while(!(SPI1->SR & SPI_SR_TXE)){};
		while (SPI1->SR & SPI_SR_BSY){};
		u8x8_gpio_SetDC(u8x8, arg_int);
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		/* Select slave, U8X8_MSG_GPIO_CS will be called */
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		/* Insert codes to end SPI transmission */
		while(!(SPI1->SR & SPI_SR_TXE)){};
		while (SPI1->SR & SPI_SR_BSY){};
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
		break;
	default:
		return 0;
	}
	return 1;
}
/* modiff main.c
end */


/* modiff U8g2
u8x8_d_uc1638.c beging*/

uint8_t u8x8_d_uc1638_240x64(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  /* call common procedure first and handle messages there */
  if ( u8x8_d_uc1638_common(u8x8, msg, arg_int, arg_ptr) == 0 )
  {
    /* msg not handled, then try here */
    switch(msg)
    {
      case U8X8_MSG_DISPLAY_SETUP_MEMORY:
	u8x8_d_helper_display_setup_memory(u8x8, &u8x8_uc1638_240x64_display_info);
	break;
      case U8X8_MSG_DISPLAY_INIT:
	u8x8_d_helper_display_init(u8x8);
	u8x8_cad_SendSequence(u8x8, u8x8_d_uc1638_240x64_init_seq);
	break;
      default:
	return 0;		/* msg unknown */
    }
  }
  return 1;
}

static const uint8_t u8x8_d_uc1638_240x64_init_seq[] = {
  U8X8_START_TRANSFER(),             	/* enable chip, delay is part of the transfer start */

  U8X8_CA(0x0e1, 0x0e2),		/* software reset */    /* UC1638*/
  U8X8_DLY(10),					/* 5 ms */

 // U8X8_C(0xa2),					/* line rate */
 // U8X8_C(0x2d),            		/* charge pump */
 // U8X8_C(0x24),            		/* set temp comp*/
//  U8X8_C(0x0c2),  					/*	mirror y and mirror x */
//
// U8X8_CA(0x04,0x00),				//set column Address
//
  U8X8_C(0xeb),
  U8X8_CA(0x81,255),
//
// U8X8_CA(0xb8,0x00),			//MTP
//
//  U8X8_C(0xc4),					//set lcd mapping control
//  	  U8X8_C(0xa3),					//set line rate  20klps
  U8X8_C(0x95),					/* set 1 bit per pixel, pattern 0*/
  U8X8_C(0x086),
  U8X8_CA(0xf1,159),
  U8X8_C(0x88),            		/*	 set auto increment, low bits are AC2 AC1 AC0 */  /* WAS 89 */

  U8X8_C(0xc2),

  U8X8_CA(0x31,0x91),
  U8X8_CA(0xc9,0xad),

  U8X8_END_TRANSFER(),             	/* disable chip */
  U8X8_END()             			/* end of sequence */
};

static const u8x8_display_info_t u8x8_uc1638_240x64_display_info =
{
  /* chip_enable_level = */ 0,	/* low active CS for this display */
  /* chip_disable_level = */ 1,

  /* post_chip_enable_wait_ns = */ 10,	/* */
  /* pre_chip_disable_wait_ns = */ 20,	/* */
  /* reset_pulse_width_ms = */ 5, 	/* */
  /* post_reset_wait_ms = */ 150,
  /* sda_setup_time_ns = */ 25,		/* */
  /* sck_pulse_width_ns = */ 65,	/* */
  /* sck_clock_hz = */ 2000000UL,	/* since Arduino 1.6.0, the SPI bus speed in Hz. Should be  1000000000/sck_pulse_width_ns */
  /* spi_mode = */ 0,		/* active high, rising edge */
  /* i2c_bus_clock_100kHz = */ 4,
  /* data_setup_time_ns = */ 30,	/*  */
  /* write_pulse_width_ns = */ 35,	/*  */
  /* tile_width = */ 30,		/* width of 30*8=240 pixel */
  /* tile_height = */ 8,
  /* default_x_offset = */ 0,	/* lower nibble: x offset, upper nibble: y offset */
  /* flipmode_x_offset = */0,	/* lower nibble: x offset, upper nibble: y offset */
  /* pixel_width = */ 240,
  /* pixel_height = */ 64
};

/* modiff U8g2
u8x8_d_uc1638.c end*/


/* modiff U8g2
u8x8_cad.c begin */
/*
  convert to bytes by using 
    dc = 0 for commands 
    dc = 1 for args and data
*/
uint8_t u8x8_cad_011(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_CAD_SEND_CMD:
      u8x8_byte_SetDC(u8x8, 0);
      u8x8_byte_SendByte(u8x8, arg_int);
 /*!!!*/u8x8_byte_SetDC(u8x8, 1);
      break;
    case U8X8_MSG_CAD_SEND_ARG:
      u8x8_byte_SetDC(u8x8, 1);
      u8x8_byte_SendByte(u8x8, arg_int);
      break;
    case U8X8_MSG_CAD_SEND_DATA:
      u8x8_byte_SetDC(u8x8, 1);
      //u8x8_byte_SendBytes(u8x8, arg_int, arg_ptr);
      //break;
      /* fall through */
    case U8X8_MSG_CAD_INIT:
    case U8X8_MSG_CAD_START_TRANSFER:
    case U8X8_MSG_CAD_END_TRANSFER:
      return u8x8->byte_cb(u8x8, msg, arg_int, arg_ptr);
    default:
      return 0;
  }
  return 1;
}
/* modiff U8g2
u8x8_cad.c end */

