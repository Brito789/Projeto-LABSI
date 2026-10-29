Robô autónomo
  O presente projeto consiste no desenvolvimento de um robô autónomo projetado para percorrer um circuito misto. O sistema combina a navegação por seguimento de linha de uma única cor com a navegação por deteção de paredes (sem recurso a linhas no piso), alternando o seu modo de funcionamento através de um sensor de cor de decisão.  
O controlo e a trajetória do robô são geridos em duas fases principais:  

 1.ª Volta — Percurso Guiado por Linha:
Comportamento: O robô utiliza a sua matriz de sensores de linha infravermelhos para seguir a cor principal da pista. O algoritmo de controlo (ex: PID) é otimizado para manter estabilidade e máxima velocidade ao longo das curvas do traçado.  
 	Gatilho de Mudança: Ao passar pelo ponto de transição, um sensor de cor dedicado faz a leitura do indicador de mudança de volta/trajeto.  

 2.ª Volta — Percurso Sem Fios / Deteção de Paredes:
 Comportamento: Ao detetar a cor correspondente, o robô desativa a navegação por linha e passa a confiar exclusivamente nos seus sensores de distância (ultrassónicos ou laser ToF).  
 Trajetória: O robô navega pela secção sem linha no piso, mantendo a distância de segurança em relação às paredes laterais e contornando os obstáculos a alta velocidade até reentrar na pista principal para finalizar o circuito.
