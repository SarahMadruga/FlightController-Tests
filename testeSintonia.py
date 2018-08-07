# Teste de sintonia feito com a arquitetura completa

#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import time
import pwmconverter
import Adafruit_PCA9685
import math
import controle
import RPi.GPIO as GPIO
from MPU6050 import MPU6050
import matplotlib.pyplot as plt
import numpy as np


controle = controle.Controle()

""" instanciando as placas IMU e o Gerador PWM"""
pwm = Adafruit_PCA9685.PCA9685()
sensor = MPU6050(0x68)
sensor.set_accel_range(0x00)
sensor.set_gyro_range(0x00)

motor1 = []
motor2 = []
motor3 = []
motor4 = []

StartTime = time.time()

accel_data_filtered = sensor.get_accel_data()
accX_filtered = accel_data_filtered["x"]
accY_filtered = accel_data_filtered["y"]
accZ_filtered = accel_data_filtered["z"]

gyro_data_filtered = sensor.get_gyro_data()
p_atual_filtered = gyro_data_filtered["x"]
q_atual_filtered = gyro_data_filtered["y"]
r_atual_filtered = gyro_data_filtered["z"]

try:
	while True:
				
		accel_data = sensor.get_accel_data()
		gyro_data = sensor.get_gyro_data()
		
		accX = accel_data["x"] # ACELERACAO x em m/s2
		accY = accel_data["y"] # ACELERACAO y em m/s2
		accZ = accel_data["z"] # ACELERACAO z em m/s2
		
		#--------------------FILTRO---------------------------#
		accX_filtered = accX_filtered*0.7 + accX*0.3
		accY_filtered = accY_filtered*0.7 + accY*0.3
		accZ_filtered = accZ_filtered*0.7 + accZ*0.3
		#-----------------------------------------------------#

		p_atual = gyro_data["x"] #velocidades angulares P em graus/s
		q_atual = gyro_data["y"] #velocidades angulares Q em graus/s
		r_atual = gyro_data["z"] #velocidades angulares R em graus/s
		
		#--------------------FILTRO---------------------------#
		p_atual_filtered = p_atual_filtered*0.7 + p_atual*0.3
		q_atual_filtered = q_atual_filtered*0.7 + q_atual*0.3
		r_atual_filtered = r_atual_filtered*0.7 + r_atual*0.3
		#-----------------------------------------------------#	

		theta_atual_rad = math.atan2(accX,math.sqrt(accZ**2+accY**2)) #atan2() da o resultado em radianos
		phi_atual_rad = math.atan2(accY,math.sqrt(accZ**2+accX**2))
		psi_atual_rad = math.atan2(accY, accX) # psi eh o arcotangente entre o vx e vy

		theta_atual = math.degrees(theta_atual_rad) #conversao de rad pra graus
		phi_atual = math.degrees(phi_atual_rad)
		psi_atual = math.degrees(psi_atual_rad)

		controlealtitude = controle.controle_alt(0, phi_atual, 0, theta_atual, 0, psi_atual, 0, 0, p_atual, q_atual, r_atual)
		controlemotores = controle.motores_control(controlealtitude[0], controlealtitude[1], controlealtitude[2], controlealtitude[3])
	
		print(controlemotores[0], controlemotores[1], controlemotores[2], controlemotores[3])

		controlemotorespwm = pwmconverter.pwmconverter_all(controlemotores[0], controlemotores[1], controlemotores[2], controlemotores[3])
		
		motor1.append(controlemotorespwm[0]) #guardando valores dos comandos enviados aos motores, para plotar
		motor2.append(controlemotorespwm[1])
		motor3.append(controlemotorespwm[2])
		motor4.append(controlemotorespwm[3])
	
		print(controlemotorespwm[0], controlemotorespwm[1], controlemotorespwm[2], controlemotorespwm[3])
		print("--------------------------------------------------")
		pwmconverter.pcamotores(controlemotorespwm[0], controlemotorespwm[1], controlemotorespwm[2], controlemotorespwm[3])

except KeyboardInterrupt: #CTRL + C	
	controlemotorespwm = pwmconverter.pwmconverter_all(0, 0, 0, 0)
	pwmconverter.pcamotores(controlemotorespwm[0], controlemotorespwm[1], controlemotorespwm[2], controlemotorespwm[3])
	
	StopTime = time.time()
	RunTime = StopTime - StartTime
	PassoAmostragem = RunTime / len(motor1)
	t = np.arange(0, RunTime, PassoAmostragem) #evenly sampled time at PassoAmostragem s intervals
	
	print("\n Abortar missao")
	print("motores para ZERO")
	print("limpa o GPIO")
	GPIO.cleanup()
	
	motor1 = np.array(motor1)
	motor2 = np.array(motor2)
	motor3 = np.array(motor3)
	motor4 = np.array(motor4)
	
	plt.figure(1)

	plt.subplot(411)
	plt.plot(motor1)
	plt.ylabel(u'Cmd M1')

	plt.subplot(412)
	plt.plot(motor2)
	plt.ylabel(u'Cmd M2')

	plt.subplot(421)
	plt.plot(motor3)
	plt.ylabel(u'Cmd M3')
	
	plt.subplot(422)
	plt.plot(motor4)
	plt.ylabel(u'Cmd M4')
	
	plt.savefig('teste_sintonia.png')
	
	print("Tempo de amostragem em segundos: ", PassoAmostragem)