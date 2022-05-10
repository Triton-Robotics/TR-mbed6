compile:
	mbed-tools compile -m NUCLEO_F446RE -t GCC_ARM

deploy:
	mbed-tools compile -m NUCLEO_F446RE -t GCC_ARM -f --sterm
