/*
 * Robô Seguidor de Linha com ESP32
 * Sistema PID com Controle de Velocidade Adaptativo
 * Otimizado para curvas fechadas e alta performance
 * 
 * Hardware:
 * - ESP32
 * - 2 motores DC 6V
 * - Array de 8 sensores de refletância
 * - Driver de motores L298N
 * 
 * Autor: Felipe Mattia
 * Data: 2025
 */

// Configurações dos pinos
#define MOTOR_ESQUERDA_IN1 26
#define MOTOR_ESQUERDA_IN2 27
#define MOTOR_ESQUERDA_ENA 14
#define MOTOR_DIREITA_IN1 32
#define MOTOR_DIREITA_IN2 33
#define MOTOR_DIREITA_ENB 25

// Pinos dos sensores de refletância
const int sensores[] = {13, 12, 14, 27, 26, 25, 33, 32};
const int NUM_SENSORES = 8;

// Configurações PID
#define KP 2.5        // Proporcional
#define KI 0.1        // Integral
#define KD 0.8        // Derivativo

// Configurações de velocidade
#define VELOCIDADE_BASE 200      // Velocidade base (0-255)
#define VELOCIDADE_MIN 80        // Velocidade mínima em curvas
#define VELOCIDADE_MAX 255       // Velocidade máxima
#define THRESHOLD_CURVA 0.6      // Threshold para detectar curvas

// Variáveis PID
float erro = 0;
float erro_anterior = 0;
float erro_integral = 0;
float erro_derivativo = 0;
float saida_pid = 0;

// Variáveis de controle
int velocidade_esquerda = VELOCIDADE_BASE;
int velocidade_direita = VELOCIDADE_BASE;
float erro_medio = 0;
bool curva_detectada = false;

// Configuração de PWM
#define FREQ_PWM 5000
#define RESOLUCAO_PWM 8

void setup() {
  Serial.begin(115200);
  Serial.println("Robô Seguidor de Linha - Inicializando...");
  
  // Configuração dos pinos dos motores
  pinMode(MOTOR_ESQUERDA_IN1, OUTPUT);
  pinMode(MOTOR_ESQUERDA_IN2, OUTPUT);
  pinMode(MOTOR_DIREITA_IN1, OUTPUT);
  pinMode(MOTOR_DIREITA_IN2, OUTPUT);
  
  // Configuração PWM para controle de velocidade
  ledcSetup(0, FREQ_PWM, RESOLUCAO_PWM);
  ledcSetup(1, FREQ_PWM, RESOLUCAO_PWM);
  ledcAttachPin(MOTOR_ESQUERDA_ENA, 0);
  ledcAttachPin(MOTOR_DIREITA_ENB, 1);
  
  // Configuração dos sensores
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(sensores[i], INPUT);
  }
  
  // Aguarda estabilização dos sensores
  delay(1000);
  Serial.println("Sistema inicializado com sucesso!");
  
  // Pergunta se deseja calibrar
  Serial.println("Deseja calibrar os sensores? (s/n)");
  delay(2000);
  
  if (Serial.available()) {
    char resposta = Serial.read();
    if (resposta == 's' || resposta == 'S') {
      calibrarSensores();
    }
  }
  
  Serial.println("Robô pronto para funcionar!");
}

void loop() {
  // Verifica se os sensores foram calibrados
  if (!sensores_calibrados) {
    Serial.println("Sensores não calibrados! Pressione 'c' para calibrar");
    if (Serial.available()) {
      char comando = Serial.read();
      if (comando == 'c' || comando == 'C') {
        calibrarSensores();
      }
    }
    delay(100);
    return;
  }
  
  // Leitura dos sensores com filtro de ruído
  int valores_sensores[NUM_SENSORES];
  lerSensores(valores_sensores);
  
  // Cálculo do erro com média ponderada
  calcularErro(valores_sensores);
  
  // Aplicação do PID
  aplicarPID();
  
  // Controle de velocidade adaptativo
  controlarVelocidadeAdaptativa();
  
  // Aplicação dos comandos aos motores
  aplicarComandosMotores();
  
  // Debug (opcional)
  if (millis() % 100 == 0) {
    debugInfo();
  }
  
  // Delay mínimo para estabilidade
  delay(5);
}

void lerSensores(int valores[]) {
  // Leitura com filtro de ruído e média
  for (int i = 0; i < NUM_SENSORES; i++) {
    int soma = 0;
    for (int j = 0; j < 3; j++) { // Média de 3 leituras
      soma += analogRead(sensores[i]);
      delayMicroseconds(50);
    }
    int valor_medio = soma / 3;
    
    // Converte para valor normalizado usando calibração
    if (sensores_calibrados) {
      // Mapeia o valor analógico para 0-1000 (0 = branco, 1000 = preto)
      int valor_normalizado = map(valor_medio, valores_minimos[i], valores_maximos[i], 0, 1000);
      valores[i] = constrain(valor_normalizado, 0, 1000);
    } else {
      // Se não calibrado, usa valor bruto
      valores[i] = valor_medio;
    }
  }
}

void calcularErro(int valores[]) {
  // Cálculo do erro usando média ponderada
  float soma_ponderada = 0;
  float soma_pesos = 0;
  
  for (int i = 0; i < NUM_SENSORES; i++) {
    float peso = (i - (NUM_SENSORES - 1) / 2.0) * 2.0; // Peso de -7 a 7
    
    // Detecta linha preta (valores altos = linha detectada)
    if (valores[i] > 500) { // Threshold de 500/1000
      soma_ponderada += peso;
      soma_pesos += abs(peso);
    }
  }
  
  if (soma_pesos > 0) {
    erro = erro * 0.7 + (soma_ponderada / soma_pesos) * 0.3;
  } else {
    // Linha não detectada - mantém erro anterior ou usa estratégia de busca
    if (abs(erro) > 0.5) {
      erro = (erro > 0) ? 1.0 : -1.0; // Busca na direção do último erro
    }
  }
}

void aplicarPID() {
  // Cálculo PID
  erro_integral += erro;
  erro_derivativo = erro - erro_anterior;
  
  // Limitação do erro integral para evitar windup
  if (abs(erro_integral) > 50) {
    erro_integral = (erro_integral > 0) ? 50 : -50;
  }
  
  // Cálculo da saída PID
  saida_pid = KP * erro + KI * erro_integral + KD * erro_derivativo;
  
  // Limitação da saída PID
  if (abs(saida_pid) > 255) {
    saida_pid = (saida_pid > 0) ? 255 : -255;
  }
  
  erro_anterior = erro;
}

void controlarVelocidadeAdaptativa() {
  // Detecção de curvas baseada no erro
  float erro_absoluto = abs(erro);
  
  if (erro_absoluto > THRESHOLD_CURVA) {
    curva_detectada = true;
    // Redução progressiva da velocidade baseada no erro
    float fator_reducao = map(erro_absoluto * 100, THRESHOLD_CURVA * 100, 100, 100, VELOCIDADE_MIN);
    fator_reducao = constrain(fator_reducao, VELOCIDADE_MIN, VELOCIDADE_BASE);
    
    velocidade_esquerda = VELOCIDADE_BASE - (saida_pid * fator_reducao / 255);
    velocidade_direita = VELOCIDADE_BASE + (saida_pid * fator_reducao / 255);
  } else {
    if (curva_detectada) {
      // Retorno gradual à velocidade normal
      velocidade_esquerda = VELOCIDADE_BASE - saida_pid;
      velocidade_direita = VELOCIDADE_BASE + saida_pid;
      
      // Verifica se estabilizou
      if (abs(erro) < 0.1) {
        curva_detectada = false;
      }
    } else {
      // Operação normal em linha reta
      velocidade_esquerda = VELOCIDADE_BASE - saida_pid;
      velocidade_direita = VELOCIDADE_BASE + saida_pid;
    }
  }
  
  // Limitação das velocidades
  velocidade_esquerda = constrain(velocidade_esquerda, 0, VELOCIDADE_MAX);
  velocidade_direita = constrain(velocidade_direita, 0, VELOCIDADE_MAX);
}

void aplicarComandosMotores() {
  // Motor esquerdo
  if (velocidade_esquerda > 0) {
    digitalWrite(MOTOR_ESQUERDA_IN1, HIGH);
    digitalWrite(MOTOR_ESQUERDA_IN2, LOW);
    ledcWrite(0, velocidade_esquerda);
  } else {
    digitalWrite(MOTOR_ESQUERDA_IN1, LOW);
    digitalWrite(MOTOR_ESQUERDA_IN2, HIGH);
    ledcWrite(0, abs(velocidade_esquerda));
  }
  
  // Motor direito
  if (velocidade_direita > 0) {
    digitalWrite(MOTOR_DIREITA_IN1, HIGH);
    digitalWrite(MOTOR_DIREITA_IN2, LOW);
    ledcWrite(1, velocidade_direita);
  } else {
    digitalWrite(MOTOR_DIREITA_IN1, LOW);
    digitalWrite(MOTOR_DIREITA_IN2, HIGH);
    ledcWrite(1, abs(velocidade_direita));
  }
}

void debugInfo() {
  Serial.print("Erro: ");
  Serial.print(erro, 3);
  Serial.print(" | PID: ");
  Serial.print(saida_pid, 1);
  Serial.print(" | Vel_E: ");
  Serial.print(velocidade_esquerda);
  Serial.print(" | Vel_D: ");
  Serial.print(velocidade_direita);
  Serial.print(" | Curva: ");
  Serial.println(curva_detectada ? "SIM" : "NAO");
}

// Função para parada de emergência
void paradaEmergencia() {
  digitalWrite(MOTOR_ESQUERDA_IN1, LOW);
  digitalWrite(MOTOR_ESQUERDA_IN2, LOW);
  digitalWrite(MOTOR_DIREITA_IN1, LOW);
  digitalWrite(MOTOR_DIREITA_IN2, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

// Variáveis para calibração
int valores_minimos[NUM_SENSORES];
int valores_maximos[NUM_SENSORES];
bool sensores_calibrados = false;

// Função para calibração automática dos sensores
void calibrarSensores() {
  Serial.println("=== CALIBRAÇÃO AUTOMÁTICA DOS SENSORES ===");
  Serial.println("1. Coloque o robô sobre uma superfície BRANCA");
  Serial.println("2. Pressione 'b' para calibrar BRANCO");
  Serial.println("3. Coloque o robô sobre uma superfície PRETA");
  Serial.println("4. Pressione 'p' para calibrar PRETO");
  Serial.println("5. Pressione 's' para iniciar o robô");
  
  // Inicializa arrays de calibração
  for (int i = 0; i < NUM_SENSORES; i++) {
    valores_minimos[i] = 1023;  // Valor máximo para encontrar o mínimo
    valores_maximos[i] = 0;      // Valor mínimo para encontrar o máximo
  }
  
  while (!sensores_calibrados) {
    if (Serial.available()) {
      char comando = Serial.read();
      
      switch (comando) {
        case 'b':
        case 'B':
          calibrarBranco();
          break;
        case 'p':
        case 'P':
          calibrarPreto();
          break;
        case 's':
        case 'S':
          finalizarCalibracao();
          break;
      }
    }
    
    // Mostra valores atuais dos sensores
    mostrarValoresSensores();
    delay(100);
  }
}

void calibrarBranco() {
  Serial.println("Calibrando BRANCO, mova o robô sobre a superfície branca por 3 segundos");
  
  unsigned long tempo_inicio = millis();
  while (millis() - tempo_inicio < 3000) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int valor = analogRead(sensores[i]);
      if (valor < valores_minimos[i]) {
        valores_minimos[i] = valor;
      }
    }
    delay(50);
  }
  
  Serial.println("BRANCO calibrado!");
  mostrarCalibracao();
}

void calibrarPreto() {
  Serial.println("Calibrando PRETO, mova o robô sobre a superfície preta por 3 segundos");
  
  unsigned long tempo_inicio = millis();
  while (millis() - tempo_inicio < 3000) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      int valor = analogRead(sensores[i]);
      if (valor > valores_maximos[i]) {
        valores_maximos[i] = valor;
      }
    }
    delay(50);
  }
  
  Serial.println("PRETO calibrado!");
  mostrarCalibracao();
}

void finalizarCalibracao() {
  Serial.println("=== CALIBRAÇÃO FINALIZADA ===");
  mostrarCalibracao();
  
  // Verifica se a calibração é válida
  bool calibracao_valida = true;
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (valores_maximos[i] - valores_minimos[i] < 100) {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" pode ter calibração insuficiente!");
      calibracao_valida = false;
    }
  }
  
  if (calibracao_valida) {
    sensores_calibrados = true;
    Serial.println("Calibração válida! Iniciando robô");
  } else {
    Serial.println("Recomenda-se recalibrar os sensores!");
  }
}

void mostrarCalibracao() {
  Serial.println("Valores de calibração:");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": Min=");
    Serial.print(valores_minimos[i]);
    Serial.print(" Max=");
    Serial.print(valores_maximos[i]);
    Serial.print(" Range=");
    Serial.println(valores_maximos[i] - valores_minimos[i]);
  }
}

void mostrarValoresSensores() {
  Serial.print("Valores atuais: ");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print(analogRead(sensores[i]));
    Serial.print(" ");
  }
  Serial.println();
}
