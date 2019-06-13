# ADC_PIC_16F1788_FIONES

Programa del PIC 16F1788 de la placa de control para la fuente de iones.

El PIC lee 9 canales analógicos (0 a 10V) y devuelve por el puerto serie
los datos datos de todos los canales mas un checksum.

Se incluye un programa de prueba para ESP32, este lee el puerto serie del PIC
y reenvia los datos por el Serial para ser visualizados en la PC.