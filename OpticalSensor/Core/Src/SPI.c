#include "SPI.h"
#include "stm32.h"
#include "main.h"

void PMW3901_init() {
	write_reg(0x3A, 0x5A);
	delay_ms(5);

	read_reg(0x00);
	read_reg(0x5F);

	read_reg(0x02);
	delay_us(50);
	read_reg(0x03);
	delay_us(50);
	read_reg(0x04);
	delay_us(50);
	read_reg(0x05);
	delay_us(50);
	read_reg(0x06);

	delay_ms(1);

	write_reg(0x7F, 0x00);
	write_reg(0x61, 0xAD);
	write_reg(0x7F, 0x03);
	write_reg(0x40, 0x00);
	write_reg(0x7F, 0x05);
	write_reg(0x41, 0xB3);
	write_reg(0x43, 0xF1);
	write_reg(0x45, 0x14);
	write_reg(0x5B, 0x32);
	write_reg(0x5F, 0x34);
	write_reg(0x7B, 0x08);
	write_reg(0x7F, 0x06);
	write_reg(0x44, 0x1B);
	write_reg(0x40, 0xBF);
	write_reg(0x4E, 0x3F);
}

void PMW3901_read_deltas(int16_t *delta_x, int16_t *delta_y, uint8_t *squal, uint8_t *shutter_u) {
	static uint8_t x_l, x_h, y_l, y_h;

	read_reg(0x02);
	delay_us(50);
	x_l = read_reg(0x03);
	delay_us(50);
	x_h = read_reg(0x04);
	delay_us(50);
	y_l = read_reg(0x05);
	delay_us(50);
	y_h = read_reg(0x06);
	delay_us(50);
	*squal = read_reg(0x07);
	delay_us(50);
	*shutter_u = read_reg(0x0C);

	*delta_x = ((int16_t)x_h << 8) | x_l;
	*delta_y = ((int16_t)y_h << 8) | y_l;
}

void write_reg(uint8_t addr, uint8_t val) {
	addr |= 0x80u;

	GPIOA->BSRR |= 1 << 20;
	delay_us(50);

	SPI_write(addr);

	delay_us(50);

	SPI_write(val);

	while((SPI1->SR >> 7) & 1);

	delay_us(50);
	GPIOA->BSRR |= 1 << 4;

	delay_us(200);
}

uint8_t read_reg(uint8_t addr) {
	uint8_t ret;

	addr &= ~0x80u;

	GPIOA->BSRR |= 1 << 20;
	delay_us(50);

	SPI_write(addr);

	delay_us(50);

	while (!((SPI1->SR) & (1 << 1)));
	while (((SPI1->SR) & (1 << 7)));

	uint16_t temp = *(volatile uint8_t *)(&SPI1->DR);
	temp = SPI1->SR;

	SPI_write(0);

	while (!((SPI1->SR)  & (1 << 0)));

	ret = *(volatile uint8_t *)(&SPI1->DR);

	delay_us(100);
	GPIOA->BSRR |= 1 << 4;

	return ret;
}

void SPI_init() {
	RCC->APB2ENR |= 1 << 12;

	//PB3 - AF - SPI1_SCK
	GPIOB->AFR[0] |= 0b0101 << 12;
	GPIOB->MODER |= 1 << 7;
	GPIOB->MODER &= ~(1 << 6);

	//PB4 - AF - SPI1_MISO
	GPIOB->AFR[0] |= 0b0101 << 16;
	GPIOB->MODER |= 1 << 9;
	GPIOB->MODER &= ~(1 << 8);

	//PB5 - AF - SPI1_MOSI
	GPIOB->AFR[0] |= 0b0101 << 20;
	GPIOB->MODER |= 1 << 11;
	GPIOB->MODER &= ~(1 << 10);

	//PA4 - AF - SPI1_CS_L
//	GPIOA->AFR[0] |= 0b0101 << 16;
//	GPIOA->MODER |= 1 << 9;
//	GPIOA->MODER &= ~(1 << 8);

	// PA4 - GPO
	GPIOA->MODER &= ~(1 << 9);
	GPIOA->MODER |= 1 << 8;

	GPIOA->PUPDR &= ~(1 << 9);
	GPIOA->PUPDR |= 1 << 8;

	SPI1->CR1 = 0;

	//Baud rate: /128
	SPI1->CR1 |= 0b111 << 3;

	//SPI Master
	SPI1->CR1 |= 1 << 2;

	//SW CS control
	SPI1->CR1 |= 1 << 9;

	//CS enable
	SPI1->CR2 |= 1 << 2;

	//8-bit data
//	SPI1->CR2 |= 0b0011 << 8;

	//8-bit RX threshold
	SPI1->CR2 |= 1 << 12;

	//SPI enable
	SPI1->CR1 |= 1 << 6;
}

void SPI_write(uint8_t data) {
	while(((SPI1->SR >> 1) & 1) == 0);
//	SPI1->DR = data;
	*(volatile uint8_t *)(&SPI1->DR) = data;
}

uint8_t SPI_read() {
	return SPI1->DR;
}
