/**
 * @brief Cabeçalho responsável pelo controle do driver de motor HIP4082.
 */

#pragma once

#include <Arduino.h>

// Classe para controlar motores que utilizam o driver HIP4082
class HIP4082
{
private:
    // Atributos que armazenam os pinos de controle do driver de motor
    int _pinoAHI; // PWM A High-Side
    int _pinoBHI; // PWM B High-Side
    int _pinoALI; // PWM A Low-Side
    int _pinoBLI; // PWM B Low-Side

    // Canais PWM
    int _canalAHI;
    int _canalBHI;
    int _canalALI;
    int _canalBLI;

    // Potencia maxma calculada a partir da resolução do PWM
    int _valorMaximoDePotencia;

    // Potencia atual
    int _potencia = 0;

    bool _modoBistate = false;
    /**
     * @brief Método para inicializar o timer do MCPWM
     */
    void initTimer() {
        mcpwm_timer_config_t configTimer = {
            .group_id = 0,                           // Bloco 0 do MCPWM
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,  // Fonte de clock padrão (160MHz)
            .resolution_hz = resolucaoHz,            // 10MHz (1 tick = 0.1µs)
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = periodTicks,             // 20kHz -> 10MHz / 500 = 20kHz
        };

        mcpwm_new_timer(&configTimer, &timer);
        mcpwm_timer_enable(timer);
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    }

    /**
     * @brief Método para inicializar os operadores do MCPWM e conectá-los ao timer
     * O operador A controla os pinos AHI e ALI, enquanto o operador B controla os pinos BHI e BLI
     * 
     * @note Os operadores são divididos entre os lados, pois cada operador só consegue controlar 2 geradores,
     * por isso temos 2 operadores atrelados ao mesmo timer
     */
    void initOperators() {
        mcpwm_operator_config_t configOperatorA = {
            .group_id = 0,
        };

        mcpwm_new_operator(&configOperatorA, &operA);
        mcpwm_operator_connect_timer(operA, timer);

        mcpwm_operator_config_t configOperatorB = {
            .group_id = 0,
        };

        mcpwm_new_operator(&configOperatorB, &operB);
        mcpwm_operator_connect_timer(operB, timer);
    }

    void initGenerator() {
        // Se não estiver no modo bistate, configura os pinos do high-side
        if (!config.modoBistate) {
            mcpwm_generator_config_t configGeneratorAHI = {
                .gen_gpio_num = static_cast<int>(config.pinoAHI),
            };

            mcpwm_generator_config_t configGeneratorBHI = {
                .gen_gpio_num = static_cast<int>(config.pinoBHI),
            };

            mcpwm_gen_handle_t generatorAHI = nullptr;
            mcpwm_gen_handle_t generatorBHI = nullptr;

            mcpwm_new_generator(operA, &configGeneratorAHI, &generatorAHI);
            mcpwm_new_generator(operB, &configGeneratorBHI, &generatorBHI);
        }

        mcpwm_generator_config_t configGeneratorALI = {
            .gen_gpio_num = static_cast<int>(config.pinoALI),
        };

        mcpwm_generator_config_t configGeneratorBLI = {
            .gen_gpio_num = static_cast<int>(config.pinoBLI),
        };

        mcpwm_gen_handle_t generatorALI = nullptr;
        mcpwm_gen_handle_t generatorBLI = nullptr;

        mcpwm_new_generator(operA, &configGeneratorALI, &generatorALI);
        mcpwm_new_generator(operB, &configGeneratorBLI, &generatorBLI);
    }

public:
    bool motorInvertido = false;

    /**
     * @brief Construtor que configura os pinos e canais PWM do driver HIP4082.
     *
     * @param pinoAHI Número do pino PWM A High-Side.
     * @param pinoBHI Número do pino PWM B High-Side.
     * @param pinoALI Número do pino PWM A Low-Side.
     * @param pinoBLI Número do pino PWM B Low-Side.
     * @param canalAHI Canal PWM para o pino A High-Side.
     * @param canalBHI Canal PWM para o pino B High-Side.
     * @param canalALI Canal PWM para o pino A Low-Side.
     * @param canalBLI Canal PWM para o pino B Low-Side.
     * @param frequenciaDoSinalDePWM Frequência do sinal PWM.
     * @param resolucao Número de bits da resolução do sinal PWM (ex: 12 bits = 0 a 4095).
     * @param placa Nome da placa que será utilizada (Unificada ou Cometa)
     */
    HIP4082(
        uint8_t pinoAHI, 
        uint8_t pinoBHI, 
        uint8_t pinoALI, 
        uint8_t pinoBLI, 
        uint8_t canalAHI, 
        uint8_t canalBHI, 
        uint8_t canalALI, 
        uint8_t canalBLI, 
        uint32_t frequenciaDoSinalDePWM, 
        uint8_t resolucao, 
        bool modoBistate = false)
        :
        _modoBistate(modoBistate)
    {
        if (modoBistate)
        {
            _pinoALI = pinoALI;
            _pinoBLI = pinoBLI;
            _canalALI = canalALI;
            _canalBLI = canalBLI;

            ledcSetup(_canalALI, frequenciaDoSinalDePWM, resolucao);
            ledcSetup(_canalBLI, frequenciaDoSinalDePWM, resolucao);

            // Anexa os canais PWM aos pinos correspondentes
            ledcAttachPin(_pinoALI, _canalALI);
            ledcAttachPin(_pinoBLI, _canalBLI);
        }

        else
        {
            _pinoAHI = pinoAHI;
            _pinoBHI = pinoBHI;
            _pinoALI = pinoALI;
            _pinoBLI = pinoBLI;
            _canalAHI = canalAHI;
            _canalBHI = canalBHI;
            _canalALI = canalALI;
            _canalBLI = canalBLI;

            // Configura os canais PWM
            ledcSetup(_canalAHI, frequenciaDoSinalDePWM, resolucao);
            ledcSetup(_canalBHI, frequenciaDoSinalDePWM, resolucao);
            ledcSetup(_canalALI, frequenciaDoSinalDePWM, resolucao);
            ledcSetup(_canalBLI, frequenciaDoSinalDePWM, resolucao);

            // Anexa os canais PWM aos pinos correspondentes
            ledcAttachPin(_pinoAHI, _canalAHI);
            ledcAttachPin(_pinoBHI, _canalBHI);
            ledcAttachPin(_pinoALI, _canalALI);
            ledcAttachPin(_pinoBLI, _canalBLI);
        }

        // Define o valor máximo de PWM a partir da resolução passada
        _valorMaximoDePotencia = static_cast<int>((1 << resolucao) - 1);

        // Para o motor
        parar();
    }

    /**
     * @brief Método para alterar a velocidade do motor de acordo com o valor de potência passado.
     *
     * @param potencia Valor da potência, na qual varia no intervalo [-valorMaximoDePotencia, valorMaximoDePotencia], para aplicar no motor. Para valores positivos o motor gira em um sentido, para valores negativos o motor gira no sentido contrário.
     */
    void setPotencia(int potencia)
    {
        potencia = constrain(potencia, -_valorMaximoDePotencia, _valorMaximoDePotencia);
        potencia = motorInvertido ? -potencia : potencia; // Se o motor estiver invertido, inverte o valor da potência

        if (_modoBistate)
        {
            // MODO BISTATE (Apenas ALI e BLI controlam a ponte)
            //
            // No modo Bistate, o controle da ponte H é simplificado, usando apenas dois pinos (ALI e BLI). O chip HIP4082 faz o trabalho e entende que se ligamos a parte de baixo, ele deve desligar a de cima automaticamente (adicionando um "tempo morto" para não dar curto-circuito na bateria).
            // ---- A REGRA DO BISTATE É INVERTIDA:
            // - Sinal ALTO (Valor MÁXIMO do PWM): Conecta aquele lado do motor ao GND (Terra).
            // - Sinal BAIXO (Valor ZERO do PWM): Conecta aquele lado do motor ao VDD (Bateria).
            // * Por isso o cálculo da aceleração é invertida: para o motor girar, nós "travamos" um pino no GND (mandando o valor MÁXIMO de PWM para ele) 
            // e aplicamos a aceleração no outro pino subtraindo a potência desejada do máximo 
            // (MAX - potencia). Assim, quanto maior a potência que você pedir, mais próximo de ZERO o sinal fica, o que injeta mais energia da bateria no motor!
            
            //Olhe o gráfico do modo de funcionamento BISTATE no datasheet do HIP4082 para entender melhor o funcionamento do driver de motor nesse modo.
            // https://www.alldatasheet.com/datasheet-pdf/view/541034/INTERSIL/HIP4082.html

            
            if (potencia > 0)
            {
                // Lado B fixo no GND (BLI = MÁXIMO).
                // Lado A recebe PWM invertido (quanto maior a potência, mais tempo próximo de 0).
                ledcWrite(_canalALI, getValorMaximoDePotencia() - potencia);
                ledcWrite(_canalBLI, potencia);
            }
            
            else if (potencia < 0)
            {
                // Lado A fixo no GND (ALI = MÁXIMO).
                // Lado B recebe PWM invertido.
                ledcWrite(_canalALI, potencia);
                ledcWrite(_canalBLI, getValorMaximoDePotencia() - abs(potencia));
            }

            else
            {
                // Para frear o motor, ambos os lados recebem o valor máximo de potência (ambos os lados desconectados)(Lembra que fica invertido).
                ledcWrite(_canalALI, getValorMaximoDePotencia());
                ledcWrite(_canalBLI, getValorMaximoDePotencia());
            }
        }

        else {
            // Se a potência passada for positiva, escreve esse valor no canal do pino ALI e BHI
            if (potencia > 0)
            {
                ledcWrite(_canalAHI, 0);
                ledcWrite(_canalALI, potencia);
                ledcWrite(_canalBHI, potencia);
                ledcWrite(_canalBLI, 0);
            }
            // Senão, se a potência passada for negativa, escreve o valor absoluto desse valor nos pinos AHI e BLI
            else if (potencia < 0)
            {
                ledcWrite(_canalAHI, abs(potencia));
                ledcWrite(_canalALI, 0);
                ledcWrite(_canalBHI, 0);
                ledcWrite(_canalBLI, abs(potencia));
            }
            // Senão, se o valor de potência for 0, freia o motor escrevendo o valor 0 no canal do pino PWMH
            else
            {
                ledcWrite(_canalAHI, 0);
                ledcWrite(_canalALI, 0);
                ledcWrite(_canalBHI, 0);
                ledcWrite(_canalBLI, 0);
            }
        }

        // Atualiza o atributo da potência do motor com o valor da potência atual
        _potencia = potencia;
    }

    // Método para parar o motor
    inline void parar()
    {
        setPotencia(0);
        // Atualiza o atributo da potência do motor com o valor de potência 0
        _potencia = 0;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor máximo de potência do motor.
     *
     * @return O valor máximo de potência do motor.
     */
    int getValorMaximoDePotencia() const
    {
        return _valorMaximoDePotencia;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor da potêncial atual do motor.
     *
     * @return O valor da potência atual do motor.
     */
    int getPotencia() const
    {
        return _potencia;
    }
};