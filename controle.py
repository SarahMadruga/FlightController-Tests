# -*- coding: utf-8 -*-

import math

"""Configuracao de eixos do drone, CONFIG deve assumir valores correspondentes a 'xis' = 0 (x) ou a 'plus' = 1 (+)"""
CONFIG = 1

#constantes
GRAVIDADE = 9.81

KP_ROLL_CONSTANT = 15   
KI_ROLL_CONSTANT = 0.05  
KD_ROLL_CONSTANT = 15    
PID_MAX_ROLL = 1000

KP_PITCH_CONSTANT = 15  
# Código usado para o controle PID

KI_PITCH_CONSTANT = 0.05 
KD_PITCH_CONSTANT = 15   
PID_MAX_PITCH = PID_MAX_ROLL

KP_YAW_CONSTANT = 3      
KI_YAW_CONSTANT = 0.02   
KD_YAW_CONSTANT = 0      
PID_MAX_YAW = 1000

KP_Z_CONSTANT = 2        
KI_Z_CONSTANT = 1.1      
KD_Z_CONSTANT = 3.3      
PID_MAX_Z = 1000

class Controle():

    def __init__(self):
        self.pid_i_mem_roll = 0
        self.erro_roll_anterior = 0
        self.pid_i_mem_pitch = 0
        self.erro_pitch_anterior = 0
        self.pid_i_mem_yaw = 0
        self.erro_yaw_anterior = 0
        self.pid_i_mem_z = 0
        self.erro_z_anterior = 0

    #Altitude (Z) Control
    def z_control(self, z_comando, z_atual):
        """Z control altitude"""
		
        erro_z = z_atual - z_comando
		
        self.pid_i_mem_z = self.pid_i_mem_z + (erro_z * KI_Z_CONSTANT)
        if self.pid_i_mem_z > PID_MAX_Z:
            self.pid_i_mem_z = PID_MAX_Z
        elif self.pid_i_mem_z < PID_MAX_Z * -1:
            self.pid_i_mem_z = PID_MAX_Z * -1
		
        z_correction = GRAVIDADE + KP_Z_CONSTANT * erro_z + self.pid_i_mem_z + KD_Z_CONSTANT * (erro_z - self.erro_z_anterior)
        if z_correction > PID_MAX_Z:
            z_correction = PID_MAX_Z
        elif z_correction < PID_MAX_Z * -1:
            z_correction = PID_MAX_Z * -1
		
        self.erro_z_anterior = erro_z
        return z_correction

    '''Controle de Postura: metodos abaixo'''

    #Roll (Phi) Control
    def phi_control(self, phi_comando, phi_atual, p_atual):
        """Controle Phi"""
		
        """Saturar angulo de roll para o drone nao perder o eixo"""
        if phi_comando > 12:
            phi_comando = 12

        elif phi_comando < -12:
            phi_comando = -12
			
        erro_roll = phi_atual - phi_comando
		
        self.pid_i_mem_roll = self.pid_i_mem_roll + (erro_roll * KI_ROLL_CONSTANT) #termo integrativo do PID, ja multiplicado por Ki
        if self.pid_i_mem_roll > PID_MAX_ROLL:
            self.pid_i_mem_roll = PID_MAX_ROLL
        elif self.pid_i_mem_roll < PID_MAX_ROLL * -1:
            self.pid_i_mem_roll = PID_MAX_ROLL * -1
		
        phi_correction = KP_ROLL_CONSTANT * erro_roll + self.pid_i_mem_roll + KD_ROLL_CONSTANT * (erro_roll - self.erro_roll_anterior)
        if phi_correction > PID_MAX_ROLL:
            phi_correction = PID_MAX_ROLL
        elif phi_correction < PID_MAX_ROLL * -1:
            phi_correction = PID_MAX_ROLL * -1
		
        self.erro_roll_anterior = erro_roll
        return phi_correction

    #Pitch (Theta) Control
    def theta_control(self, theta_comando, theta_atual, q_atual):
        """Theta Control"""
		
        """Saturar angulo de pitch para o drone nao perder o eixo"""
        if theta_comando > 12:
            theta_comando = 12

        elif theta_comando < -12:
            theta_comando = -12
			
        erro_pitch =  theta_atual - theta_comando
		
        self.pid_i_mem_pitch = self.pid_i_mem_pitch + (erro_pitch * KI_PITCH_CONSTANT)
        if self.pid_i_mem_pitch > PID_MAX_PITCH:
            self.pid_i_mem_pitch = PID_MAX_PITCH
        elif self.pid_i_mem_pitch < PID_MAX_PITCH * -1:
            self.pid_i_mem_pitch = PID_MAX_PITCH * -1
		
        theta_correction = KP_PITCH_CONSTANT * erro_pitch + self.pid_i_mem_pitch + KD_PITCH_CONSTANT * (erro_pitch - self.erro_pitch_anterior)
        if theta_correction > PID_MAX_PITCH:
            theta_correction = PID_MAX_PITCH
        elif theta_correction < PID_MAX_PITCH * -1:
            theta_correction = PID_MAX_PITCH * -1
		
        self.erro_pitch_anterior = erro_pitch
        return theta_correction

    #Yaw (Psi) Control
    def psi_control(self, psi_comando, psi_atual, r_atual):
        """Psi control"""
        erro_yaw =  psi_atual - psi_comando
		
        self.pid_i_mem_yaw = self.pid_i_mem_yaw + (erro_yaw * KI_YAW_CONSTANT)
        if self.pid_i_mem_yaw > PID_MAX_YAW:
            self.pid_i_mem_yaw = PID_MAX_YAW
        elif self.pid_i_mem_yaw < PID_MAX_YAW * -1:
            self.pid_i_mem_yaw = PID_MAX_YAW * -1
		
        psi_correction = KP_YAW_CONSTANT * erro_yaw + self.pid_i_mem_yaw + KD_YAW_CONSTANT * (erro_yaw - self.erro_yaw_anterior)
        if psi_correction > PID_MAX_YAW:
            psi_correction = PID_MAX_YAW
        elif psi_correction < PID_MAX_YAW * -1:
            psi_correction = PID_MAX_YAW * -1
			
        self.erro_yaw_anterior = erro_yaw
        return psi_correction

    #CHAMA CONTROLADOR DE ALTITUDE/NIVEL
    def controle_alt(self, phi_comando, phi_atual, theta_comando, theta_atual, psi_comando, psi_atual, z_comando, z_atual, p_atual, q_atual, r_atual):
        """Controle de altura"""
        zcontrol = self.z_control(z_comando, z_atual)
        phicontrol = self.phi_control(phi_comando, phi_atual, p_atual)
        thetacontrol = self.theta_control(theta_comando, theta_atual, q_atual)
        psicontrol = self.psi_control(psi_comando, psi_atual, r_atual)        
        return [zcontrol, thetacontrol, phicontrol, psicontrol]

    #Comandos para os motores
    def motores_control(self, z_correction, theta_correction, phi_correction, psi_correction):
        """Controle de correcao para cada motor"""
        if CONFIG == 0:
            mc1 = z_correction - theta_correction + phi_correction + psi_correction #Front Right, Clockwise (FR, CW)
            mc2 = z_correction - theta_correction - phi_correction - psi_correction #(FL, CCW)
            mc3 = z_correction + theta_correction - phi_correction + psi_correction #(BL, CW)
            mc4 = z_correction + theta_correction + phi_correction - psi_correction #(BR, CCW)

        elif CONFIG == 1:
            mc1 = z_correction + psi_correction - theta_correction  #Front Right, Clockwise (FR, CW)          
            mc3 = z_correction + psi_correction + theta_correction #(BL, CW)
            
            mc2 = z_correction - psi_correction - phi_correction  #(FL, CCW)
            mc4 = z_correction - psi_correction + phi_correction  #(BR, CCW)

        return [mc1, mc2, mc3, mc4]
