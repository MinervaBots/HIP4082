/**
 * @brief Biblioteca header-only para controle de motores utilizando o driver HIP4082 com o periférico MCPWM do ESP32
 * 
 * @author MinervaBots
 * 
 * @version 2.0.0
 * 
 * @see espidf v5.0 ou superior
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
    bool modoBistate = false; // Se ativo a classe atuará na versão do modo Bistate do driver
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

    mcpwm_cmpr_handle_t comparatorAHI = nullptr;
    mcpwm_cmpr_handle_t comparatorBHI = nullptr;
    mcpwm_cmpr_handle_t comparatorALI = nullptr;
    mcpwm_cmpr_handle_t comparatorBLI = nullptr;

    uint32_t resolucaoHz = 20000000; // Resolução do timer do MCPWM [Padrão = 20MHz -> 1 tick = 0,05us]
    uint32_t periodTicks = 1000;     // Número de ticks em um período (Cálculo para frequência -> f = res / perTicks) [Padrão = 1000 -> Freq = 20kHz com resolução de 20MHz]
    uint32_t frequencia = resolucaoHz / periodTicks; // Frequência em Hz

    // Potencia máximo do sinal de PWM
    uint32_t valorMaximoDePotencia = periodTicks - 1;  // O valor é decrementado pois o valor começa em 0
    int32_t valorMinimoDePotencia = -valorMaximoDePotencia;

    // Potencia atual
    int32_t potencia = 0;

    /* ------------------ Funções para gerarem erros ou warnings em compilação ------------------ */
    void erro_resolucao_zero() __attribute__((error("ERRO: A resolucao nao pode ser 0!")));
    void erro_frequencia_zero() __attribute__((error("ERRO: A frequencia nao pode ser 0!")));

    void warning_non_const_frequencia() __attribute__((warning("AVISO: A frequencia nao foi reconhecida pelo compilador como uma constante, verifique a funcao setFrequencia eh chamada antes do metodo begin")));
    void warning_non_const_resolucao() __attribute__((warning("AVISO: A resolucao nao foi reconhecida pelo compilador como uma constante, verifique a funcao setResolucao eh chamada antes do metodo begin")));

    /**
     * @brief Método para inicializar o timer do MCPWM
     * O timer é responsável por gerar a base de tempo para o sinal PWM
     * Um timer consegue controlar até 4 geradores de PWM
     */
    void initTimer() {
        mcpwm_timer_config_t configTimer = {
            .group_id = 0,                           // Bloco 0 do MCPWM
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,  // Fonte de clock padrão (160MHz)
            .resolution_hz = resolucaoHz,            // 1 tick = 1 segundo / resolucaoHz
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = periodTicks,             // Frequência do PWM = resolucaoHz / periodTicks (periodTicks é o número de ticks para completar um período do PWM)
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

    /**
     * @brief Método para inicializar os comparadores do MCPWM
     * O comparador é responsável por comparar o valor do timer com um valor de referência para determinar quando o sinal PWM deve mudar de estado (HIGH ou LOW)
     * 
     * @note O update compare on timer equal zero é essencial para evitar o driver queimar com pulsos errados, 
     * pois garante que o valor do comparador seja atualizado no início de cada período do PWM, 
     * evitando que o sinal fique preso em um estado por mais tempo do que o desejado.
     */
    void initComparators() {
        mcpwm_comparator_config_t configComparatorALI = {
            /// @attention O update compare on timer equal zero é essencial para evitar o driver queimar com pulsos errados
            .flags = {
                .update_cmp_on_tez = true, // Atualiza o valor do comparador no início do período
            }
        };

        mcpwm_comparator_config_t configComparatorBLI = {
            .flags = {
                .update_cmp_on_tez = true,
            }
        };

        mcpwm_new_comparator(operA, &configComparatorALI, &comparatorALI);
        mcpwm_new_comparator(operB, &configComparatorBLI, &comparatorBLI);

        if (!config.modoBistate) {
            mcpwm_comparator_config_t configComparatorAHI = {
                /// @attention O update compare on timer equal zero é essencial para evitar o driver queimar com pulsos errados
                .flags = {
                    .update_cmp_on_tez = true, // Atualiza o valor do comparador no início do período
                }
            };

            mcpwm_comparator_config_t configComparatorBHI = {
                .flags = {
                    .update_cmp_on_tez = true,
                }
            };

            mcpwm_new_comparator(operA, &configComparatorAHI, &comparatorAHI);
            mcpwm_new_comparator(operB, &configComparatorBHI, &comparatorBHI);
        }
    }

    /**
     * @brief Método para inicializar os geradores do MCPWM e configurá-los para gerar o sinal PWM de acordo com o valor do comparador
     * O gerador é responsável por gerar o sinal PWM, configurando as ações que devem ser tomadas quando o timer atinge certos eventos 
     * (como contar até zero ou atingir o valor do comparador)
     */
    void initGenerators() {
        mcpwm_generator_config_t configGeneratorALI = {
            .gen_gpio_num = static_cast<int>(config.pinoALI),
        };

        mcpwm_generator_config_t configGeneratorBLI = {
            .gen_gpio_num = static_cast<int>(config.pinoBLI),
        };

        mcpwm_new_generator(operA, &configGeneratorALI, &generatorALI); // Cria o gerador para o pino ALI e conecta ao operador A
        mcpwm_new_generator(operB, &configGeneratorBLI, &generatorBLI); // Cria o gerador para o pino BLI e conecta ao operador B

        // Configura as ações dos geradores para o ALI, quando o timer contar até zero, o sinal de PWM vai para HIGH, 
        // e quando o valor do comparador for atingido, o sinal de PWM vai para LOW
        mcpwm_generator_set_action_on_timer_event(generatorALI, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(generatorALI, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorALI, MCPWM_GEN_ACTION_LOW));

        // Configuração das ações dos geradores para o BLI
        mcpwm_generator_set_action_on_timer_event(generatorBLI, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(generatorBLI, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorBLI, MCPWM_GEN_ACTION_LOW));

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

            mcpwm_generator_set_action_on_timer_event(generatorAHI, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
            mcpwm_generator_set_action_on_compare_event(generatorAHI, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorAHI, MCPWM_GEN_ACTION_LOW));

            mcpwm_generator_set_action_on_timer_event(generatorBHI, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
            mcpwm_generator_set_action_on_compare_event(generatorBHI, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorBHI, MCPWM_GEN_ACTION_LOW));
        }
    }

public:
    bool motorInvertido = false; // Variável para inverter o sentido de rotação do motor

    /// @brief Construtor da classe (vazio)
    HIP4082() {}

    /// @brief Destrutor da classe, responsável por deletar os geradores, comparadores, operadores e timer criados
    ~HIP4082() {
        // Deleta os geradores, comparadores, operadores e timer criados, verificando se eles foram criados antes de tentar deletar
        if (generatorAHI) mcpwm_del_generator(generatorAHI);
        if (generatorBHI) mcpwm_del_generator(generatorBHI);
        if (generatorALI) mcpwm_del_generator(generatorALI);
        if (generatorBLI) mcpwm_del_generator(generatorBLI);

        if (comparatorAHI) mcpwm_del_comparator(comparatorAHI);
        if (comparatorBHI) mcpwm_del_comparator(comparatorBHI);
        if (comparatorALI) mcpwm_del_comparator(comparatorALI);
        if (comparatorBLI) mcpwm_del_comparator(comparatorBLI);

        if (operA) mcpwm_del_operator(operA);
        if (operB) mcpwm_del_operator(operB);

        if (timer) mcpwm_del_timer(timer);
    }

    /**
     * @brief Método para alterar a resolução do timer do MCPWM
     * 
     * @param novaResolucao Nova resolução do timer em Hz (1 tick = 1 segundo / novaResolucao)
     * A resolução é a base de tempo do PWM, quanto maior a resolução, mais preciso é o controle da potência, 
     * mas também aumenta o uso de CPU e pode limitar a frequência máxima do PWM.
     * [Padrão = 20MHz]
     * 
     * @note A função setFrequencia deve ser chamada após essa função para atualizar o valor do periodTicks de acordo com a nova resolução
     * @note O valor da resolução deve ser uma constante reconhecida em tempo de compilação para evitar warnings, e deve ser diferente de 0 para evitar erros de compilação.
     */
    inline void setResolucao(const uint32_t novaResolucao) __attribute__((always_inline)) {
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

    /**
     * @brief Método para alterar a frequência do PWM
     * 
     * @param novaFrequencia Nova frequência do PWM em Hz
     * A frequência do PWM determina a rapidez com que o sinal de controle é atualizado, o que pode afetar a resposta do motor e a eficiência
     * 
     * @note A frequência depende da resolução do timer, se você deseja uma frequência específica, verifique a resolução atual do timer e calcule o valor do 
     * periodTicks de acordo com a fórmula: periodTicks = resolucaoHz / frequencia. Se a frequência desejada não for atingível com a resolução atual, 
     * considere alterar a resolução do timer com o método setResolucao()
     * @note O valor da frequência deve ser uma constante reconhecida em tempo de compilação para evitar warnings, e deve ser diferente de 0 para evitar erros de compilação.
     */
    inline void setFrequencia(const uint32_t novaFrequencia) __attribute__((always_inline)) {
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
     * @brief Método para inicializar o driver de motor HIP4082 com a configuração passada por parâmetro
     * 
     * @param *config Struct com a configuração inicial para o driver (pinos, canais e modo)
     * Parâmetros de configuração:
     * @param config.pinoAHI: Pino conectado ao A High-Side do driver
     * @param config.pinoALI: Pino conectado ao A Low-Side do driver
     * @param config.pinoBHI: Pino conectado ao B High-Side do driver
     * @param config.pinoBLI: Pino conectado ao B Low-Side do driver
     * @param config.modoBistate: Se true, a classe atuará no modo Bistate do driver
     */
    void begin(const hip_config_t &config) {
        // Atribuindo o struct passado por parâmetro ao struct membro da classe
        this->config = config;

        initTimer();
        initOperators();
        initComparators();
        initGenerators();
    }

    /**
     * @brief Método para alterar a velocidade do motor de acordo com o valor de potência passado.
     *
     * @param potencia Valor da potência, na qual varia no intervalo [-valorMaximoDePotencia, valorMaximoDePotencia], para aplicar no motor. 
     * Para valores positivos o motor gira em um sentido, para valores negativos o motor gira no sentido contrário.
     * Na resolução padrão de 20MHz e frequência de 20kHz, o valor varia no intervalo [-999, 999] com 0 representando o motor parado.
     */
    void setPotencia(int32_t potencia) {
        potencia = (potencia > valorMaximoDePotencia) ? valorMaximoDePotencia : potencia; // Limita a potência ao valor máximo permitido
        potencia = (potencia < -valorMaximoDePotencia) ? -valorMaximoDePotencia : potencia; // Limita a potência ao valor mínimo permitido
        potencia = motorInvertido ? -potencia : potencia; // Se o motor estiver invertido, inverte o valor da potência

        if (config.modoBistate)
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

            
            if (potencia > 0) {
                // Lado B fixo no GND (BLI = MÁXIMO).
                // Lado A recebe PWM invertido (quanto maior a potência, mais tempo próximo de 0).
                mcpwm_comparator_set_compare_value(comparatorALI, valorMaximoDePotencia - potencia);
                mcpwm_comparator_set_compare_value(comparatorBLI, potencia);
            }
            
            else if (potencia < 0) {
                // Lado A fixo no GND (ALI = MÁXIMO).
                // Lado B recebe PWM invertido.
                mcpwm_comparator_set_compare_value(comparatorALI, potencia);
                mcpwm_comparator_set_compare_value(comparatorBLI, valorMaximoDePotencia - abs(potencia));
            }

            else {
                // Para frear o motor, ambos os lados recebem o valor máximo de potência (ambos os lados desconectados)(Lembra que fica invertido).
                mcpwm_comparator_set_compare_value(comparatorALI, valorMaximoDePotencia);
                mcpwm_comparator_set_compare_value(comparatorBLI, valorMaximoDePotencia);
            }
        }

        else {
            // Reseta o estado forçado dos geradores (se a potência anterior for 0, ele fica com os geradores forçados em low)
            if (potencia != 0) {
                mcpwm_generator_set_force_level(generatorAHI, -1, true);
                mcpwm_generator_set_force_level(generatorALI, -1, true);
                mcpwm_generator_set_force_level(generatorBHI, -1, true);
                mcpwm_generator_set_force_level(generatorBLI, -1, true);
            }

            // Se a potência passada for positiva, escreve esse valor no canal do pino ALI e BHI
            if (potencia > 0) {
                mcpwm_comparator_set_compare_value(comparatorAHI, 0);
                mcpwm_comparator_set_compare_value(comparatorALI, potencia);
                mcpwm_comparator_set_compare_value(comparatorBHI, potencia);
                mcpwm_comparator_set_compare_value(comparatorBLI, 0);
            }

            // Senão, se a potência passada for negativa, escreve o valor absoluto desse valor nos pinos AHI e BLI
            else if (potencia < 0) {
                mcpwm_comparator_set_compare_value(comparatorAHI, abs(potencia));
                mcpwm_comparator_set_compare_value(comparatorALI, 0);
                mcpwm_comparator_set_compare_value(comparatorBHI, 0);
                mcpwm_comparator_set_compare_value(comparatorBLI, abs(potencia));
            }
            
            else {
                mcpwm_generator_set_force_level(generatorAHI, 0, true);
                mcpwm_generator_set_force_level(generatorALI, 0, true);
                mcpwm_generator_set_force_level(generatorBHI, 0, true);
                mcpwm_generator_set_force_level(generatorBLI, 0, true);
            }
        }

        // Atualiza o atributo da potência do motor com o valor da potência atual
        this->potencia = potencia;
    }

    /**
     * @brief Método para utilizar a potência em 8 bits (-127 a 127) para controlar o motor
     * O valor será convertido para a resolução atual do timer do MCPWM
     * 
     * @param potencia Valor da potência em 8 bits, no intervalo [-127, 127]. 
     * Para valores positivos o motor gira em um sentido, para valores negativos o motor gira no sentido contrário.
     */
    inline void setPotencia8bits(int8_t potencia) {
        // Converte a potência de 8 bits para a resolução atual
        int32_t potenciaConvertida = (static_cast<int32_t>(potencia) * static_cast<int32_t>(valorMaximoDePotencia)) / INT8_MAX;

        setPotencia(potenciaConvertida); // Chamada do método setPotencia com o valor convertido para a resolução atual
    }

    /// @brief Método para parar o motor, setando a potência para 0
    inline void parar() {
        setPotencia(0);
        // Atualiza o atributo da potência do motor com o valor de potência 0
        this->potencia = 0;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor máximo de potência do motor
     * 
     * @param potencia8bits Se true, o valor máximo de potência será retornado para a resolução de 8 bits (127), 
     * caso contrário, será retornado o valor máximo de potência para a resolução atual do timer do MCPWM (valorMaximoDePotencia).
     * [Padrão = false]
     *
     * @return O valor máximo de potência do motor.
     */
    uint32_t getValorMaximoDePotencia(bool potencia8bits = false) const {
        return potencia8bits ? static_cast<uint32_t>(INT8_MAX) : valorMaximoDePotencia;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor mínimo de potência do motor
     * 
     * @param potencia8bits Se true, o valor mínimo de potência será retornado para a resolução de 8 bits (-127), 
     * caso contrário, será retornado o valor mínimo de potência para a resolução atual do timer do MCPWM (valorMinimoDePotencia).
     * [Padrão = false]
     * 
     * @return O valor mínimo de potência do motor.
     */
    int32_t getValorMinimoDePotencia(bool potencia8bits = false) const {
        return potencia8bits ? static_cast<int32_t>(INT8_MIN) : valorMinimoDePotencia;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor da potêncial atual do motor.
     *
     * @return O valor da potência atual do motor.
     */
    int32_t getPotencia() const {
        return this->potencia;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena a frequência atual do PWM.
     * 
     * @return O valor da frequência atual do PWM em Hz.
     */
    uint32_t getFrequencia() const {
        return frequencia;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena a resolução atual do timer do MCPWM.
     * 
     * @return O valor da resolução atual do timer do MCPWM em Hz.
     */
    uint32_t getResolucao() const {
        return resolucaoHz;
    }
};