# 🇺🇸 English
# Line Follower Robot with ESP32 and PID Control

## Overview
This repository contains code for a line follower robot using an ESP32 as the main controller. It implements an advanced PID control loop with adaptive speed control to maximize performance on tight turns.

## Key Features

**High Performance**
- PID tuned for fast response
- Adaptive speed control in corners
- Optimized processing with minimal delay (5ms)
- Noise filtering for stable sensor readings

**Smart Control**
- **Proportional–Integral–Derivative (PID)**: KP=2.5, KI=0.1, KD=0.8
- **Automatic curve detection** based on error
- **Progressive speed reduction** on tight turns
- **Gradual return to normal speed** after stabilization

**Optimizations for Tight Turns**
- Configurable curve threshold (0.6)
- Minimum speed in turns (80/255)
- Search strategy when the line is not detected
- Smoothing filter to reduce oscillations

## Required Hardware

### Main Components
- **ESP32** (any model)
- **2x 6V DC motors** (rear)
- **8-channel reflectance sensor array** (QTR-8A or similar)
- **L298N motor driver** or similar
- **6V–12V battery** (depending on driver)

### Pin Connections

#### Motors
```
Left Motor:
- IN1 → GPIO 26
- IN2 → GPIO 27
- ENA → GPIO 14

Right Motor:
- IN1 → GPIO 32
- IN2 → GPIO 33
- ENB → GPIO 25
```

#### Reflectance Sensors
```
Sensor 1 → GPIO 13
Sensor 2 → GPIO 12
Sensor 3 → GPIO 14
Sensor 4 → GPIO 27
Sensor 5 → GPIO 26
Sensor 6 → GPIO 25
Sensor 7 → GPIO 33
Sensor 8 → GPIO 32
```

## Adjustable Settings

### PID Parameters
```cpp
#define KP 2.5        // Proportional — response strength
#define KI 0.1        // Integral — removes steady-state error
#define KD 0.8        // Derivative — adds stability/damping
```

### Speed Control
```cpp
#define VELOCIDADE_BASE 200      // Base speed (0-255)
#define VELOCIDADE_MIN 80        // Minimum speed in turns
#define VELOCIDADE_MAX 255       // Maximum speed
#define THRESHOLD_CURVA 0.6      // Threshold to detect curves
```

## System Operation

### 1. Sensor Reading
- Average of 3 reads to reduce noise
- Temporal smoothing filter
- Search strategy when the line is not detected

### 2. Error Calculation
- Weighted average of active sensors
- Weights from -7 to +7 for precision
- Smoothing filter for stability

### 3. PID Control
- Optimized computation of the three components
- Integral windup limiting
- Output clamping to avoid saturation

### 4. Adaptive Speed Control
- **Curve detection**: based on absolute error
- **Progressive reduction**: speed decreases as error increases
- **Gradual return**: speed returns to normal after stabilization

### 5. Motor Commands
- PWM control for variable speed
- Direction controlled by IN1/IN2 pins
- 5kHz PWM frequency for smoothness

## Competition Tuning

### For Very Tight Turns
```cpp
#define KP 3.0        // Increase responsiveness
#define VELOCIDADE_MIN 60        // Lower minimum speed
#define THRESHOLD_CURVA 0.5      // Detect curves earlier
```

### For Straight Lines (Max Speed)
```cpp
#define VELOCIDADE_BASE 220      // Increase base speed
#define KP 2.0        // Reduce response for stability
```

### For Stability on Gentle Curves
```cpp
#define KD 1.2        // Increase damping
#define KI 0.05       // Reduce integral correction
```

## Monitoring and Debug

### Serial Monitor
- Set to 115200 baud
- Debug info every 100ms
- Data: Error, PID Output, Speeds, Curve state

### Debug Functions
```cpp
debugInfo()           // Real-time information
paradaEmergencia()    // Full motor stop
calibrarSensores()    // Manual calibration (optional)
```

## Tips

### 1. First Run
- Check all connections
- Tune PID parameters as needed
- Test on straight lines before curves

### 2. Fine Tuning
- Start with low KP and increase gradually
- Adjust KD to reduce oscillations
- Use KI only if there is steady-state error

### 3. Competition Optimization
- Comment out `debugInfo()` to reduce overhead
- Adjust `VELOCIDADE_BASE` to match the track
- Test on different curve types

## Troubleshooting

### Robot Oscillates on the Line
- Lower KP
- Increase KD
- Check sensor alignment

### Cannot Handle Tight Turns
- Increase KP
- Reduce VELOCIDADE_MIN
- Adjust THRESHOLD_CURVA

### Speed Too Low
- Increase VELOCIDADE_BASE
- Check motor connections
- Ensure adequate power supply

## License
This code was developed for educational and competition purposes. Feel free to modify and adapt it to your needs.

**Built with a focus on performance for robotics competitions.**

# 🇧🇷 Português
# Robô Seguidor de Linha com ESP32 e Tecnologia PID

## Visão Geral
Este é um código para um robô seguidor de linha que utiliza um ESP32 como controlador principal. O sistema implementa controle PID avançado com controle de velocidade adaptativo para maximizar a performance em curvas fechadas.

## Características Principais

**Alta Performance**
- Sistema PID otimizado para resposta rápida
- Controle de velocidade adaptativo em curvas
- Processamento otimizado com delay mínimo (5ms)
- Filtros de ruído para leitura estável dos sensores

**Controle Inteligente**
- **PID Proporcional-Integral-Derivativo**: KP=2.5, KI=0.1, KD=0.8
- **Detecção automática de curvas** baseada no erro
- **Redução progressiva de velocidade** em curvas fechadas
- **Retorno gradual à velocidade normal** após estabilização

**Otimizações para Curvas Fechadas**
- Threshold de curva configurável (0.6)
- Velocidade mínima em curvas (80/255)
- Estratégia de busca quando linha não é detectada
- Filtro de suavização para reduzir oscilações

## Hardware Necessário

### Componentes Principais
- **ESP32** (qualquer modelo)
- **2 Motores DC 6V** (traseiros)
- **Array de 8 sensores de refletância** (QTR-8A ou similar)
- **Driver de motores L298N** ou similar
- **Bateria 6V-12V** (dependendo do driver)

### Conexões dos Pinos

#### Motores
```
Motor Esquerdo:
- IN1 → GPIO 26
- IN2 → GPIO 27
- ENA → GPIO 14

Motor Direito:
- IN1 → GPIO 32
- IN2 → GPIO 33
- ENB → GPIO 25
```

#### Sensores de Refletância
```
Sensor 1 → GPIO 13
Sensor 2 → GPIO 12
Sensor 3 → GPIO 14
Sensor 4 → GPIO 27
Sensor 5 → GPIO 26
Sensor 6 → GPIO 25
Sensor 7 → GPIO 33
Sensor 8 → GPIO 32
```

## Configurações Ajustáveis

### Parâmetros PID
```cpp
#define KP 2.5        // Proporcional - Ajuste para resposta
#define KI 0.1        // Integral - Ajuste para eliminar erro estático
#define KD 0.8        // Derivativo - Ajuste para estabilidade
```

### Controle de Velocidade
```cpp
#define VELOCIDADE_BASE 200      // Velocidade base (0-255)
#define VELOCIDADE_MIN 80        // Velocidade mínima em curvas
#define VELOCIDADE_MAX 255       // Velocidade máxima
#define THRESHOLD_CURVA 0.6      // Threshold para detectar curvas
```

## Funcionamento do Sistema

### 1. **Leitura dos Sensores**
- Média de 3 leituras para reduzir ruído
- Filtro de suavização temporal
- Estratégia de busca quando linha não detectada

### 2. **Cálculo do Erro**
- Média ponderada dos sensores ativos
- Pesos de -7 a +7 para precisão
- Filtro de suavização para estabilidade

### 3. **Controle PID**
- Cálculo otimizado das três componentes
- Limitação do erro integral (anti-windup)
- Limitação da saída para evitar saturação

### 4. **Controle de Velocidade Adaptativo**
- **Detecção de curvas**: Baseada no erro absoluto
- **Redução progressiva**: Velocidade diminui conforme erro aumenta
- **Retorno gradual**: Velocidade retorna ao normal após estabilização

### 5. **Comando dos Motores**
- Controle PWM para velocidade variável
- Direção controlada pelos pinos IN1/IN2
- Frequência PWM de 5kHz para suavidade

## Ajustes para Competição

### Para Curvas Muito Fechadas
```cpp
#define KP 3.0        // Aumentar resposta
#define VELOCIDADE_MIN 60        // Reduzir velocidade mínima
#define THRESHOLD_CURVA 0.5      // Detectar curvas mais cedo
```

### Para Linhas Retas (Velocidade Máxima)
```cpp
#define VELOCIDADE_BASE 220      // Aumentar velocidade base
#define KP 2.0        // Reduzir resposta para estabilidade
```

### Para Estabilidade em Curvas Suaves
```cpp
#define KD 1.2        // Aumentar amortecimento
#define KI 0.05       // Reduzir correção integral
```

## Monitoramento e Debug

### Serial Monitor
- Configurado para 115200 baud
- Informações de debug a cada 100ms
- Dados: Erro, Saída PID, Velocidades, Estado da curva

### Funções de Debug
```cpp
debugInfo()           // Informações em tempo real
paradaEmergencia()    // Parada total dos motores
calibrarSensores()    // Calibração manual (opcional)
```

## Dicas de Uso

### 1. **Primeira Execução**
- Verifique todas as conexões
- Ajuste os parâmetros PID conforme necessário
- Teste em linha reta antes de curvas

### 2. **Ajuste Fino**
- Comece com KP baixo e aumente gradualmente
- Ajuste KD para reduzir oscilações
- Use KI apenas se houver erro estático

### 3. **Otimização para Competição**
- Comente a função `debugInfo()` para reduzir overhead
- Ajuste `VELOCIDADE_BASE` conforme a pista
- Teste em diferentes tipos de curvas

## Solução de Problemas

### Robô Oscila na Linha
- Reduza KP
- Aumente KD
- Verifique se os sensores estão alinhados

### Não Consegue Fazer Curvas Fechadas
- Aumente KP
- Reduza VELOCIDADE_MIN
- Ajuste THRESHOLD_CURVA

### Velocidade Muito Baixa
- Aumente VELOCIDADE_BASE
- Verifique conexões dos motores
- Confirme alimentação adequada

## Licença
Este código foi desenvolvido para fins educacionais e de competição. Sinta-se livre para modificar e adaptar conforme suas necessidades.

**Desenvolvido com foco em performance para competições de robótica.**
