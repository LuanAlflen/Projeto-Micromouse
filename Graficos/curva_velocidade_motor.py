#! /usr/bin/python3
# -*- coding: utf-8 -*-
#_________________________________________________________
# Universidade Federal de Santa Catarina
# Departamento de Engenharias da Mobilidade
# Curso de Cálculo Numérico
# Prof. Alexandre Zabot
# https://zabot.paginas.ufsc.br
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

import matplotlib.pyplot as plt
import numpy as np
import math




x = [-1, -0.1, -0.09999, 0, 0.09999, 0.1, 1] #utilização do motor (em porcentagem)
y = [-49.2, -8.2, 0, 0, 0, 8.2, 49.2] #Velocidade (cm/s)

x2 = [-1, -0.1, 0, 0.1, 1] #utilização do motor (em porcentagem)
y2 = [-49.2, -8.2, 0, 8.2, 49.2] #Velocidade (cm/s)


# Configuração do primeiro gráfico (acima)
plt.subplot(2, 1, 1)  # 2 linhas, 1 coluna, gráfico 1
# plt.plot(x, y, "r-", label="Média")
plt.plot(x, y, "r-", label="Sem offset")
plt.xlabel('Utilização do motor (%)')
plt.ylabel('Velocidade (cm/s)')
# plt.title('Precisão média do melhor indivíduo')
plt.grid()
plt.legend()

plt.subplots_adjust(hspace=0.4)

# Configuração do segundo gráfico (abaixo)
plt.subplot(2, 1, 2)  # 2 linhas, 1 coluna, gráfico 2
# plt.plot(x, z, "b-", label="Variancia")
plt.plot(x2, y2, "b-", label="Com offset")
plt.xlabel('Utilização do motor (%)')
plt.ylabel('Velocidade (cm/s)')
# plt.title('Variancia do melhor indivíduo')
plt.grid()
plt.legend()

plt.show()



