# EmbarcaTechSensoresi2c
Projeto de utilização de sensores através de conexão i2c do programa de capacitação EmbarcaTech - TIC 37

# Vídeo Demonstração

https://youtu.be/vAz5BIqGXbY

# Hardware/Firmware
Projeto desenvolvido em uma placa de desenvolvimento BitDogLab, versão 6.3.
Desenvolvimento de firmware feito através do PicoSDK, versão 2.1.1, com a IDE Visual Studio Code.

# Instruções
O programa recebe dados dos sensores conectados nos terminais i2c da BitDogLab e os demonstra em gráficos em um servidor Web.<br><br>

O BMP280 envia sinais de temperatura e também de pressão, este sendo lido e utilizado para estimar uma altitude em relação ao nível do mar.<br>
O AHT20 (ou AHT10) é utilizado para medir temperatura e umidade do ar.<br><br>

O LED azul piscando indica que o programa está rodando e pronto para receber conexões TCP.
