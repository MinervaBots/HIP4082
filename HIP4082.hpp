/*
Cabeçalho responsável pelo controle do driver de motor HIP4082.
*/

#pragma once

#include <Arduino.h>

enum Placa
{
    Unificada,
    Cometa
};

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

    Placa _placa;

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
    HIP4082(int pinoAHI, int pinoBHI, int pinoALI, int pinoBLI, int canalAHI, int canalBHI, int canalALI, int canalBLI, int frequenciaDoSinalDePWM, int resolucao, Placa placa)
    {
        // Atribui os valores passados nos atributos
        _placa = placa;
        if (placa == Cometa)
        {
            _pinoALI = pinoALI;
            _pinoBLI = pinoBLI;
            _canalALI = canalALI;
            _canalBLI = canalBLI;
        }

        else if (placa == Unificada)
        {
            _pinoAHI = pinoAHI;
            _pinoBHI = pinoBHI;
            _pinoALI = pinoALI;
            _pinoBLI = pinoBLI;
            _canalAHI = canalAHI;
            _canalBHI = canalBHI;
            _canalALI = canalALI;
            _canalBLI = canalBLI;
        }

        // Define o valor máximo de PWM a partir da resolução passada
        _valorMaximoDePotencia = (int)(pow(2, resolucao) - 1);

        if (placa == Unificada)
        {
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

        else if (placa == Cometa)
        {
            ledcSetup(_canalALI, frequenciaDoSinalDePWM, resolucao);
            ledcSetup(_canalBLI, frequenciaDoSinalDePWM, resolucao);

            // Anexa os canais PWM aos pinos correspondentes
            ledcAttachPin(_pinoALI, _canalALI);
            ledcAttachPin(_pinoBLI, _canalBLI);
        }

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
        if (abs(potencia) <= getValorMaximoDePotencia())
        {
            // Se o motor estiver invertido, inverte o valor da potência
            if (motorInvertido)
            {
                potencia = -potencia;
            }
        }

        if (_placa == Unificada)
        {
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

        if (_placa == Cometa)
        {
            // Se a potência passada for positiva, escreve esse valor no canal do pino ALI e passa o seu complemento no BLI
            // Quando o ALI estiver mandando HIGH, o BLI estará mandando LOW
            // Isso seria a mesma coisa que colocar uma porta NOT em um dos pinos
            if (potencia > 0)
            {
                ledcWrite(_canalALI, potencia);
                ledcWrite(_canalBLI, getValorMaximoDePotencia() - potencia);
            }

            else if (potencia < 0)
            {
                ledcWrite(_canalALI, getValorMaximoDePotencia() - potencia);
                ledcWrite(_canalBLI, potencia);
            }

            else
            {
                ledcWrite(_canalALI, 0);
                ledcWrite(_canalBLI, 0);
            }
        }
        // Atualiza o atributo da potência do motor com o valor da potência atual
        _potencia = potencia;
    }

    // Método para parar o motor
    void parar()
    {
        if (_placa == Unificada)
        {
            // Freia o motor escreven+do o valor 0 no canal do pino PWMH
            ledcWrite(_canalAHI, 0);
            ledcWrite(_canalALI, getValorMaximoDePotencia());
            ledcWrite(_canalBHI, 0);
            ledcWrite(_canalBLI, getValorMaximoDePotencia());
        }

        else if (_placa == Cometa)
        {
            // Freia o motor escrevendo o valor 0 no canal do pino PWMH
            // Testar diferença com ambos os pinos em HIGH
            ledcWrite(_canalALI, 0);
            ledcWrite(_canalBLI, 0);
        }

        // Atualiza o atributo da potência do motor com o valor de potência 0
        _potencia = 0;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor máximo de potência do motor.
     *
     * @return O valor máximo de potência do motor.
     */
    int getValorMaximoDePotencia()
    {
        return _valorMaximoDePotencia;
    }

    /**
     * @brief Método para obter o valor do atributo que armazena o valor da potêncial atual do motor.
     *
     * @return O valor da potência atual do motor.
     */
    int getPotencia()
    {
        return _potencia;
    }
};