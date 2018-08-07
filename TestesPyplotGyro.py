# Código usado para coletar dados do MPU6050

#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import matplotlib.pyplot as plt
import numpy as np
import time
import math
from MPU6050 import MPU6050


""" instanciando as placas IMU """
sensor = MPU6050(0x68)
sensor.set_accel_range(0x00)
sensor.set_gyro_range(0x00)

P = []
Q = []
R = []

theta_graus = []
phi_graus = []
psi_graus = []

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

		accel_data = sensor.get_accel_data() #lendo dados acelerometro e giroscopio
		gyro_data = sensor.get_gyro_data()
	
		p_atual = gyro_data["x"] #velocidades angulares P em graus/s
		q_atual = gyro_data["y"] #velocidades angulares Q em graus/s
		r_atual = gyro_data["z"] #velocidades angulares R em graus/s

		#--------------------FILTRO---------------------------#

		p_atual_filtered = p_atual_filtered*0.7 + p_atual*0.3
		q_atual_filtered = q_atual_filtered*0.7 + q_atual*0.3
		r_atual_filtered = r_atual_filtered*0.7 + r_atual*0.3

		#-----------------------------------------------------#	

		P.append(p_atual) #guardando valores no array para plotar
		Q.append(q_atual)
		R.append(r_atual)

		print ("p_atual: ",p_atual, "q_atual: ", q_atual, "r_atual: ",r_atual)

		accX = accel_data["x"] # ACELERACAO x em m/s2
		accY = accel_data["y"] # ACELERACAO y em m/s2
		accZ = accel_data["z"] # ACELERACAO z em m/s2

		#--------------------FILTRO---------------------------#

		accX_filtered = accX_filtered*0.7 + accX*0.3
		accY_filtered = accY_filtered*0.7 + accY*0.3
		accZ_filtered = accZ_filtered*0.7 + accZ*0.3

		#-----------------------------------------------------#	

		theta_atual_rad = math.atan2(accX,math.sqrt(accZ**2+accY**2)) #atan2() da o resultado em radianos
		phi_atual_rad = math.atan2(accY,math.sqrt(accZ**2+accX**2))
		psi_atual_rad = math.atan2(accY, accX) # psi eh o arcotangente entre o vx e vy

		theta_atual = math.degrees(theta_atual_rad) #conversao de rad pra graus
		phi_atual = math.degrees(phi_atual_rad)
		psi_atual = math.degrees(psi_atual_rad)

		theta_graus.append(theta_atual) #guardando valores no array para plotar
		phi_graus.append(phi_atual)
		psi_graus.append(psi_atual)

		print ("theta_atual: ", theta_atual, "phi_atual: ", phi_atual, "psi_atual: ", psi_atual)

		
except KeyboardInterrupt: #CTRL + C

	StopTime = time.time()
	RunTime = StopTime - StartTime
	PassoAmostragem = RunTime / len(P)
	t = np.arange(0, RunTime, PassoAmostragem) #evenly sampled time at PassoAmostragem s intervals

#----------------------------------------------------------------------------------------------

	P = np.array(P) #phi rate
	Q = np.array(Q) #theta rate
	R = np.array(R) #psi rate

	plt.figure(1)

	plt.subplot(311)
	plt.plot(P)
	plt.ylabel(u'P data (graus/s)')

	plt.subplot(312)
	plt.plot(Q)
	plt.ylabel(u'Q data (graus/s)')

	plt.subplot(313)
	plt.plot(R)
	plt.ylabel(u'R data (graus/s)')
	plt.xlabel('Time (s)')

#----------------------------------------------------------------------------------------------
	
	phi_graus = np.array(phi_graus) #rotacao sobre o eixo x - ROLL
	theta_graus = np.array(theta_graus) #rotacao sobre o eixo y - PITCH
	psi_graus = np.array(psi_graus) #rotacao sobre o eixo z - YAW

	plt.figure(2)

	plt.subplot(311)
	plt.plot(phi_graus)
	plt.ylabel(u'Roll/Phi (graus)')

	plt.subplot(312)
	plt.plot(theta_graus)
	plt.ylabel(u'Pitch/Theta (graus)')

	plt.subplot(313)
	plt.plot(psi_graus)
	plt.ylabel(u'Yaw/Psi (graus)')
	plt.xlabel('Time (s)')
	
	plt.show()

	print("\nTempo de amostragem em segundos: ", PassoAmostragem)