# Firmware HEADEND v1.0

**Status:** Paused  
**People:** Arturo Armando Veras Olivos  
**Finalización:** 1

[Carpeta](https://drive.google.com/drive/folders/17ku6Cu4AZ0bmrrsLup_GOUawh5QAdTd1?usp=drive_link)
## Versión 1.3
[headend_20242602.hex ](https://drive.google.com/file/d/10mNuLc_gexjG9TbZrhT5K_cRUJCLGUcE/view?usp=drive_link)
Parches para el funcionamiento de la pantalla:
- Se cambian los colores de la pantalla
- Se mejora las alertas DC
- Se mejora el cambio de voltaje DC, se agrega el estado VIN_0V
## Versión 1.0
[headend_20242602.hex](https://drive.google.com/file/d/12gfd_B679W3XQUE-kX0PbZPM9oSn8pr-/view?usp=drive_link)
- 4 sensores de corriente max4080, uno para cada arterial
- 2 sensores de potencia, ~~uplink entrada~~ y downlink en la base. max4003
- ~~Voltage DC de arterial. analog input 12V / 24V / 48V.~~
- 4 salidas digitales para 4 switch RF.
- 4 salidas digitales para 4 switch DC.
- 5 entradas digitales para las alarmas.
- uart1 para la pantalla nexus
- uart2 para debug por rs485
- memoria eeprom por i2c
- 4 led en salidas digitales para mostrar estados.
- pantalla
- Enviar datos anteriores por pantalla. Switch rf en pantalla, switch DC
- comunicacion rs485 para drs sacar datos por serial}
- alarma voltaje en rojo
- Consultar por rs485 todos los parametros previamente mencionados.
- Alarma de corriente de 1 [A]

## Project tasks
1. [Establish development environment](https://www.notion.so/Se-establece-el-entorno-de-desarrollo-96eb48fca0cc42a898ba74b9292d0b7a?pvs=21)
2. [Configure readings for the 4 arterials on the screen](https://www.notion.so/Se-configura-lecturas-para-los-4-arteriales-en-la-pantalla-1f8588f9ba33483b860036284f232a89?pvs=21)
3. [Review requirements](https://www.notion.so/Revisar-requerimientos-0f4f89ef8060453f8164d90daa271e7b?pvs=21)
4. [Read digital inputs ](https://www.notion.so/Leer-entradas-digitales-40a4d96796fa4cfda6c0901d2764c71b?pvs=21)
5. [Read analog inputs](https://www.notion.so/Leer-entradas-anaolgas-e17fbdc50b764eb9b374c56f4d42299b?pvs=21)
6. Read Voltage [Read Voltage](https://www.notion.so/Leer-Voltaje-06505226a8774a31929aba65151ae023?pvs=21)
7. [Investigate why ADC is not reading](https://www.notion.so/Revisar-porqu-no-lee-los-ADC-b89f18818e384342b1c5871c47115e29?pvs=21)
8. [Decode current sensor](https://www.notion.so/Decodificar-sensor-de-corriente-0d351d1127734bb4b2a4801f824dffc2?pvs=21)
9. [Review UART1 communication for faster functionality](https://www.notion.so/Revisar-comunicaci-n-UART1-para-que-funcione-r-pido-5ce4a08cab614c65b32fd9694f6d7fab?pvs=21)
10. [Decode voltage sensor](https://www.notion.so/Decodificar-sensor-de-voltaje-0f93af65441c4ab6a7298918889b9055?pvs=21)
11. [Investigate delay in reading analog inputs after button press](https://www.notion.so/Porqu-las-entradas-analogas-se-leen-luego-de-presdionar-el-boton-tiene-algo-que-ver-la-interrupci-6a31c4b07c7a4d11bc9e1a31c31ffbc9?pvs=21)
12. [Send commands via uart2](https://www.notion.so/Env-o-de-comandos-por-uart2-cabbd2618f454b75963467125cbeb8ca?pvs=21)
13. [Alerts for digital input and levels](https://www.notion.so/Alertas-por-entrada-digital-y-por-niveles-96bcbc51cc9c4c8889a96f1692c4ccd3?pvs=21)
14. [Code Refactor](https://www.notion.so/Code-Refactor-c204a37c89274f6f8bfd5b0611f9847c?pvs=21)
15. [Review moving average](https://www.notion.so/Revisar-movinga-average-6f627efebfdc46b883e93d4ca7b2a0f7?pvs=21)
16. [Display only one voltage on the panel](https://www.notion.so/Dejar-solo-un-voltaje-en-el-panel-e2bae9c104ca40838f0814a8bd86579b?pvs=21)
17. [Adjust alert levels to 1 A for current](https://www.notion.so/Ajustar-nieveles-de-alertas-a-1-A-de-corriente-a72793f4e48448e28dd3a8eb85eb4266?pvs=21)
18. [Change font for voltages](https://www.notion.so/Cambiar-tipograf-a-para-voltajes-13728c5b542f4583994b38a7171f6ab9?pvs=21)
19. [Improve screen layout](https://www.notion.so/MEjora-de-layout-de-la-pantalla-c87be99b98d54e7e91d8c049ef6b7103?pvs=21)
20. [Perform measurements to adjust ADC values](https://www.notion.so/REalizar-mediciones-para-ajustar-valores-de-ADC-6dc1b1f0c99d471e835ccecd99a2e198?pvs=21)
21. [Try new layout](https://www.notion.so/Probar-nuevo-layout-1df81b0be2924133a50d18c250903443?pvs=21)
22. [Activate or deactivate voltage by arterial](https://www.notion.so/Activar-o-desactrivar-voltaje-por-arterial-bbc0aa9f53f5495d9f39837f219ff6e5?pvs=21)
23. [Alert for current ADC value](https://www.notion.so/Alerta-por-valor-ADC-de-corriente-30b080ae2b5b4ff58ca350709548dfc1?pvs=21)
24. [Add voltage by arterial](https://www.notion.so/Agregar-volataje-por-arterial-662d980a94d84f79ad4fab6d6c3f1d5c?pvs=21)
25. [Map real values and Downlink alerts](https://www.notion.so/Mapear-valores-reales-y-alertas-Downlunk-b2f60545436c422ab2209f1a4db97160?pvs=21)
26. [Clean up code](https://www.notion.so/Limpiar-el-c-digo-489d171bbe484f9f8f5ef324b109286b?pvs=21)
27. [Acknowledge lack of DC in each arterial, turn on DC button in red](https://www.notion.so/Akarma-de-que-no-hay-DC-en-cada-arterial-encender-boton-DC-en-rojo-0652266217464e2190a34f4a65941030?pvs=21)
28. [Add alarms when DC is not connected, turn on voltage in red](https://www.notion.so/Agregar-alarmas-cuando-dc-no-se-conecta-encender-voltaje-en-rojo-a16b4f370b8e4ca08a8fb9cf907a9841?pvs=21)
29. [Color Rojo y sin numero para cuando no hay potencia](https://www.notion.so/Color-Rojo-y-sin-numero-para-cuando-no-hay-potencia-e684521c1c1d4eaa963b1d03926e5efc?pvs=21)
30. [Mejora de alarmas en pantalla](https://www.notion.so/Mejora-de-alarmas-en-pantalla-ca2ab3a31186484a9e877cb41a5b377f?pvs=21)
