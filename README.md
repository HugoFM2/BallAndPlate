# Controle de um dispositivo Ball and Plate por meio de visão computacional

<p align="justify"> O presente trabalho visa o controle de um sistema Ball and Plate por meio de visão computacional. Este sistema envolve uma superfície que pode ser rotacionada nos eixos x e y, em que se utiliza três servos motores para atingir a inclinação desejada, juntamente com uma bola que pode se movimentar livremente pela superfície. O objetivo principal é controlar a bola por meio de rotações na superfície, visando posicioná-la em uma referência pré-determinada. Para atingir esse objetivo, quatro temas principais são abordados. Em primeiro lugar, a visão computacional é empregada para determinar a posição da superfície e, posteriormente, a posição da bola.A partir disso, a cinemática inversa é utilizada para converter o ângulo desejadoda superfície em angulações específicas para os servos motores. A modelagem dosistema, fundamentada na mecânica lagrangiana, é empregada para obter a plantado sistema. Por fim, o controle é realizado por meio de dois controladores PIDs,os quais são responsáveis pelo gerenciamento da posição da bola sobre a superfície,ajustando a inclinação da mesa para controlar o movimento da bola. Os resultadosobtidos indicam êxito na construção do sistema, bem como na implementação detodos os recursos utilizados. No entanto, são apresentadas propostas de melhoriasque visam aprimorar a robustez e precisão do sistema. </p>


## 🚀 Começando

Essas instruções permitirão que você obtenha uma cópia do projeto em operação na sua máquina local para fins de desenvolvimento e teste.



### 📋 Pré-requisitos

Para poder executar todos os códigos, é necessário ter as seguintes bibliotecas em suas respectivas versões:

```
imutils==0.5.4
numpy==1.24.3
opencv-contrib-python==4.7.0.72
opencv-python==4.7.0.72
PyQt5==5.15.4
scikit-learn==1.1.1
scipy==1.5.2
```

O código para o Arduino e programas auxiliares, como a calibração da câmera, podem ser encontrados na pasta **Extras**.


### 🔧 Instalação

É recomendada a criação de um ambiente virtual contendo os pacotes listados acima, para isso, ativar o ambiente e instalar as bibliotecas a partir do seguinte comando:
```
pip install -r requirements.txt
```


## 📦 Execução

Para executar o código principal, é necessário estar na pasta raíz e executar o seguinte comando:
```
python main.py
```
Para o código principal rodar, é necessário estar com a Câmera e o Arduino conectados.

### 🔧 Funcionamento

Aqui são explicados os principais arquivos do projeto.

#### Arquivos Principais
 - [main.py](https://github.com/HugoFM2/BallAndPlate/blob/main/main.py) - Responsável pela execução do programa. Inicializa a GUI e todas as classes dependentes.

#### Cinemática Inversa
 - [CinematicaInversa/Plate.py](https://github.com/HugoFM2/BallAndPlate/blob/main/CinematicaInversa/Plate.py) - Classe da superfície. Define as coordenadas da 3a Junta e permite ajustar a altura e a rotação da superfície nos eixos x e y
 - [CinematicaInversa/Servo.py](https://github.com/HugoFM2/BallAndPlate/blob/main/CinematicaInversa/Servo.py) - Classe dos Servos. Responsável por ajustar a inclinação dos 3 Servos.
 - [CinematicaInversa/BallAndPlate.py](https://github.com/HugoFM2/BallAndPlate/blob/main/CinematicaInversa/BallAndPlate.py) - Classe principal que reúne as classes Plate e Servo. Responsável por obter a posição de todas as juntas e enviar a inclinação do servo para o Arduino via comunicação Serial. 

#### Controle
 - [NoThread/Controle.py](https://github.com/HugoFM2/BallAndPlate/blob/main/NoThread/Controle.py) - Classe de Controle. Responsável pela implementação do PID e do Saturador.

#### Visão Computacional
 - [NoThread/CompVisual.py](https://github.com/HugoFM2/BallAndPlate/blob/main/NoThread/CompVisual.py) - Classe de visão computacional. Responsável pela detecção da superfície, através dos ChArUcos, e da bola.

#### Arquivos Extras
- [Extras/Arduino/servo/servo.ino](https://github.com/HugoFM2/BallAndPlate/blob/main/Extras/Arduino/servo/servo.ino) - Arquivo para carregar no Arduino que recebe os comandos via Serial do computador e envia as informações para o módulo PWM PCA9685 para o controle dos servos.
- [Extras/CameraCalibration/Calibration.py](https://github.com/HugoFM2/BallAndPlate/blob/main/Extras/Camera%20Calibration/Calibration.py) - Arquivo para calibrar a câmera. Para fazer a calibração, é necessário imprimir o Chessboard que está localizado no mesmo diretório, pasta ChessBoardGen.
  
## 🛠️ Construído com

* [QT Designer](https://doc.qt.io/qt-5/qtdesigner-manual.html) - Programa utilizado para gerar a interface


## ✒️ Autores

* **Hugo Ferreira Marques** - *Trabalho e documentação*
* **Luciano Antonio Frezzatto Santos** - *Orientação*



---
Feito com base no modelo disponibilizado por [Armstrong Lohãns](https://gist.github.com/lohhans)
