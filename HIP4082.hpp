/**
 * @brief Cabeçalho responsável pelo controle do driver de motor HIP4082.
 */

#pragma once

#include <stdint.h>

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_cmpr.h"

typedef struct {
    uint8_t pinoAHI;  // PWM A High-Side
    uint8_t pinoALI;  // PWM A L Low-Side
    uint8_t pinoBHI;  // PWM B High-Side
    uint8_t pinoBLI;  // PWM B Low-Side
    uint8_t canalAHI; // Canal A High-Side
    uint8_t canalBHI; // Canal B High-Side
    uint8_t canalALI; // Canal A Low-Side
    uint8_t canalBLI; // Canal B Low-Side
    bool modoBistate = false // Se ativo a classe atuará na versão do modo Bistate do driver
} hip_config_t;

// Classe para controlar motores que utilizam o driver HIP4082
class HIP4082 {
private:

    // struct com a configuração inicial para o driver (pinos, canais e modo)
    hip_config_t config;

    // Inicializando os handles do timer e dos operadores como nulos
    mcpwm_timer_handle_t timer = nullptr;

    mcpwm_oper_handle_t operA = nullptr; // Operador para o lado A da ponte H (AHI e ALI)
    mcpwm_oper_handle_t operB = nullptr; // Operador para o lado B da ponte H (BHI e BLI)

    mcpwm_gen_handle_t generatorAHI = nullptr;
    mcpwm_gen_handle_t generatorBHI = nullptr;
    mcpwm_gen_handle_t generatorALI = nullptr;
    mcpwm_gen_handle_t generatorBLI = nullptr;

    mcpwm_cmpr_handle_t comparatorA = nullptr;
    mcpwm_cmpr_handle_t comparatorB = nullptr;

    static uint32_t resolucaoHz = 10000000; // Resolução do timer do MCPWM [Padrão = 10MHz -> 1 tick = 0.1µs]
    static uint32_t periodTicks = 500       // Número de ticks em um período (Cálculo para frequência -> f = res / perTicks) [Padrão = 500 -> Freq = 20kHz]
    static uint32_t frequencia = resolucaoHz / periodTicks; // Frequência em Hz

    // Potencia maxma calculada a partir da resolução do PWM
    int valorMaximoDePotencia;

    // Potencia atual
    int potencia = 0;

    /* ------------------ Funções para gerarem erros ou warnings em compilação ------------------ */
    void erro_resolucao_zero() __attribute__((error("ERRO: A resolucao nao pode ser 0!")));
    void erro_frequencia_zero() __attribute__((error("ERRO: A frequencia nao pode ser 0!")));

    void warning_non_const_frequencia() __attribute__((warning("AVISO: A frequencia nao foi reconhecida pelo compilador como uma constante, verifique a funcao setFrequencia eh chamada antes do metodo begin")));
    void warning_non_const_resolucao() __attribute__((warning("AVISO: A resolucao nao foi reconhecida pelo compilador como uma constante, verifique a funcao setResolucao eh chamada antes do metodo begin")));

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

            mcpwm_new_generator(operA, &configGeneratorAHI, &generatorAHI);
            mcpwm_new_generator(operB, &configGeneratorBHI, &generatorBHI);
        }

        mcpwm_generator_config_t configGeneratorALI = {
            .gen_gpio_num = static_cast<int>(config.pinoALI),
        };

        mcpwm_generator_config_t configGeneratorBLI = {
            .gen_gpio_num = static_cast<int>(config.pinoBLI),
        };

        mcpwm_new_generator(operA, &configGeneratorALI, &generatorALI);
        mcpwm_new_generator(operB, &configGeneratorBLI, &generatorBLI);
    }

    void initComparator() {
        mcpwm_comparator_config_t configComparatorA = {
            /// @attention O update compare on timer equal zero é essencial para evitar o driver queimar com pulsos errados
            .flags.update_cmp_on_tez = true, // Atualiza o valor do comparador no início do período
        };

        mcpwm_comparator_config_t configComparatorB = {
            .flags.update_cmp_on_tez = true,
        };

        mcpwm_new_comparator(operA, &configComparatorA, &comparatorA);
        mcpwm_new_comparator(operB, &configComparatorB, &comparatorB);
    }

public:
    bool motorInvertido = false;

    HIP4082() {}

    ~HIP4082() {
        // Desabilita o timer e os operadores ao destruir a classe
        if (timer) {
            mcpwm_timer_disable(timer);
            mcpwm_del_timer(timer);
        }

        if (operA) {
            mcpwm_del_operator(operA);
        }

        if (operB) {
            mcpwm_del_operator(operB);
        }
    }

    __attribute__((always_inline)) // Atributo para forçar a função a ser inline, necessário para as verificações de tempo de compilação
    inline void setResolucao(const uint32_t novaResolucao) {
        // Checa se o valor é uma constante reconhecida em tempo de compilação
        if (!__builtin_constant_p(novaResolucao)) {
            warning_non_const_resolucao();
        }

        // Checa se o valor é diferente de 0, se não, gera erro de compilação
        if (__builtin_constant_p(novaResolucao) && novaResolucao == 0) {
            erro_resolucao_zero(); 
        }

        // Se a nova resolução já for igual a padrão, não faz nada
        if (novaResolucao == resolucaoHz) {
            return;
        }

        resolucaoHz = novaResolucao;
        periodTicks = novaResolucao / frequencia; // Atualiza o valor do periodTicks de acordo com a nova resolução
    }

    __attribute__((always_inline)) // Atributo para forçar a função a ser inline, necessário para as verificações de tempo de compilação
    inline void setFrequencia(const uint32_t novaFrequencia) {
        // Checa se o valor é uma constante reconhecida em tempo de compilação
        if (!__builtin_constant_p(novaFrequencia)) {
            warning_non_const_frequencia();
        }

        // Checa se o valor é diferente de 0, se não, gera erro de compilação
        if (__builtin_constant_p(novaFrequencia) && novaFrequencia == 0) {
            erro_frequencia_zero(); 
        }

        if (novaFrequencia == frequencia) {
            return;
        }

        frequencia = novaFrequencia;
        periodTicks = resolucaoHz / novaFrequencia;
    }

    /**
     * @brief
     */
    void begin(const hip_config_t *config) {
        // Atribuindo o struct passado por parâmetro ao struct membro da classe
        this->config = config;

        initTimer();
        initOperators();
        initGenerator();
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
        return static_cast<int>((1 << resolucao) - 1);
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

    uint32_t getFrequencia() const
    {
        return frequencia;
    }

    uint32_t getResolucao() const
    {
        return resolucaoHz;
    }
};