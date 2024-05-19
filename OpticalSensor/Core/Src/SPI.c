#include "SPI.h"
#include "main.h"

void SPI_init(float freq) {
	//PB3 - AF - SPI1_SCK
	GPIOB->MODER |= 1 << 7;
	GPIOB->MODER &= ~(1 << 6);

	//PB4 - AF - SPI1_MISO
	GPIOB->MODER |= 1 << 9;
	GPIOB->MODER &= ~(1 << 8);

	//PB5 - AF - SPI1_MOSI
	GPIOB->MODER |= 1 << 11;
	GPIOB->MODER &= ~(1 << 10);

	//PA4 - AF - SPI1_CS_L
	GPIOB->MODER |= 1 << 9;
	GPIOB->MODER &= ~(1 << 8);

	SPI1->CR1 = 0;

	//Baud rate: /128
	SPI1->CR1 |= 0b110 << 6;

	//SPI Master
	SPI1->CR1 |= 1 << 2;

	SPI1->CR2 |= 1 << 2;

	SPI1->CR1 |= 1 << 6;
}

void SPI_write(uint8_t byte) {
	SPI1->DR = byte;
}

uint8_t SPI_read(uint8_t byte) {
	SPI1->DR = byte;
}