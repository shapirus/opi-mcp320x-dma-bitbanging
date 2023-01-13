all : spi-bitbanging
spi-bitbanging : spi-bitbanging.c Makefile
	gcc spi-bitbanging.c -o spi-bitbanging -Wpedantic -O2 -Wall -g
