# Coleta dados do barômetro e calcula a altitude

import time
import navio.util
from  navio.ms5611 import MS5611

navio.util.check_apm()
baro = MS5611()
baro.initialize()

#--------------------------------------------------------

time.sleep(2)
baro.refreshPressure()
time.sleep(0.01) # Waiting for pressure data ready 10ms
baro.readPressure()
filtered =  baro.PRES
ref = (filtered/(1.165*9.8))
print("referencia: ",ref)
#--------------------------------------------------------

while(True):
	baro.refreshPressure()
	time.sleep(0.01) # Waiting for pressure data ready 10ms
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Waiting for temperature data ready 10ms
	baro.readTemperature()

	baro.calculatePressureAndTemperature()
	print ("Temperature(C): " ,(baro.TEMP))
	print ("Pressure(millibar): ",(baro.PRES))
	#-----------------------------------------------------
	filtered = filtered + 0.1*((baro.PRES) - filtered)
	alt = (ref - filtered/(1.165*9.8))/3,6
	print ("altura em cm", (alt//1))
	#-----------------------------------------------------
	#  p=dgh, onde p eh a pressao, d eh a densidade do ar 
	# (para 25graus = 1.1839 e para 30graus = 1.165), g eh 
	#  a gravidade, e h eh a altura
	#-----------------------------------------------------

	time.sleep(1)