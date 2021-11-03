from vmaPID import PID
import time
import matplotlib.pyplot as plt
import numpy as np


class heater:
    def __init__(self):
        self.temp = 25
    def update(self, power, dt):
        if power is None:
            power = 0
        if power > 0 :
             
            #Variation of room temperature with power and time variable dt during heating
            self.temp += 2 * power * dt
        #Indicates heat loss in a room
        self.temp -= 200 * dt
        return self.temp


if __name__ == "__main__":
    power = 0
    P = 40
    I = 4
    D = 0.1
    pid = PID(P, I, D)

    pid.setSetPoint(0.0)
    pid.setSampleTime(0.01)
    pid.setOutputLimit(1500,0)
    pid.setWindup('Reset', 0.001) # Teria q testar melhor esse metodo
    pid.setWindup('Clamp', 20) 

    heater = heater()
    temp = heater.temp

    start_time = time.time()
    last_time = start_time

	#Visualize Output Results
    setpoint, temperatura, t, output = [], [], [], []

    # loop do controle, dura 1
    while time.time() - start_time < 1:
        current_time = time.time()

        dt = current_time - last_time

        power = pid.update(temp) # PID recebe estado atual e calcula resposta

        temp = heater.update(power, dt) # envia a potencia calculada ao heater simulado, q devolve nova temperatura atual
        pid._PID__saturation(1,1,1)

       #Visualize Output Results
        t += [current_time - start_time]
        temperatura += [temp]
        output += [power]
        setpoint += [pid.SetPoint]
		#Used for initial value assignment of variable temp
        if current_time - start_time > 0.1: # inserir o tempo para o passo ocorrer
            pid.SetPoint = 130

        if current_time - start_time > 0.3: # inserir o tempo para o passo ocorrer
            pid.SetPoint = 75
        if current_time - start_time > 0.5: # inserir o tempo para o passo ocorrer
            pid.SetPoint = 100

        last_time = current_time

    fig, ax1 = plt.subplots()

    ax1.set_xlabel('Tempo (s)')
    ax1.set_ylabel('Temperatura (C)')
    lns1 = ax1.plot(t, setpoint, label='Temperatura Desejada / Set Point')
    lns2 = ax1.plot(t, temperatura, label='Temperatura / State')
    plt.grid()
    
    import sys

    print('setpoint', setpoint[-1])
    print('temperatura', temperatura[-1])
    
    ax2 = ax1.twinx()
    ax2.set_ylabel('Potência (W)')
    lns3 = ax2.plot(t, output, label='Potência Aplicada / Control Effort', color = 'green')
    
    lns = lns1+lns2+lns3
    labs = [l.get_label() for l in lns]
    
    ax1.legend(lns, labs, loc=0)
    
    #plt.legend()
    
    plt.show()
