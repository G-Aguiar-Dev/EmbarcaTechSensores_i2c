//-------------------------------------------Bibliotecas-------------------------------------------
#include <stdio.h>                  // Biblioteca padrão de entrada e saída
#include <string.h>                 // Biblioteca padrão de manipulação de strings
#include <ctype.h>                  // Biblioteca padrão de manipulação de caracteres
#include <math.h>                   // Biblioteca padrão de funções matemáticas

#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico
#include "pico/bootrom.h"           // Biblioteca com recursos para trabalhar com o bootrom da Raspberry Pi Pico
#include "pico/multicore.h"         // Biblioteca para suporte a múltiplos núcleos na Raspberry Pi Pico

#include "FreeRTOS.h"               // Biblioteca de FreeRTOS
#include "task.h"                   // Biblioteca de tasks

#include "lwip/tcp.h"               // Biblioteca de LWIP para manipulação de TCP/IP

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/i2c.h"           // Biblioteca de hardware de I2C

#include "aht20.h"                  // Biblioteca para o sensor AHT20 (Temperatura e Umidade)
#include "bmp280.h"                 // Biblioteca para o sensor BMP280 (Pressão e Temperatura)

//-------------------------------------------Definições-------------------------------------------
#define WIFI_SSID ""                            // Nome da rede Wi-Fi
#define WIFI_PASS ""                            // Senha da rede Wi-Fi

#define I2C_PORT_BMP i2c0                       // i2c0 pinos 0 e 1
#define I2C_SDA_BMP 0                           // SDA do BMP280
#define I2C_SCL_BMP 1                           // SCL do BMP280
#define I2C_ADDR_BMP 0x76                       // Endereço do BMP280 (Sensor de Pressão e Temperatura)
#define SEA_LEVEL_PRESSURE 101325.0             // Pressão ao nível do mar em Pa
struct bmp280_calib_param params;               // Estrutura para armazenar os parâmetros de calibração do BMP280

#define I2C_PORT_AHT i2c1                       // i2c1 pinos 2 e 3
#define I2C_SDA_AHT 2                           // SDA do AHT20
#define I2C_SCL_AHT 3                           // SCL do AHT20
#define I2C_ADDR_AHT 0x38                       // Endereço do AHT10 (Sensor de Temperatura e Umidade)

#define DEBOUNCE_MS 500                         // Tempo de debounce para os botões     
                               // Pino do botão A (GPIO 5)
#define BOTAO_B 6                               // Pino do botão B (BOOTSEL)
#define LED_BLUE 12                             // Pino do LED Azul

//-------------------------------------------Variáveis Globais-------------------------------------------

// HTML do template da página (gráficos de temperatura, umidade e altitude)
const char HTML_BODY[] =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Sistema de Monitoramento de Sensores</title>"
    "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>"
    "<style>"
    "body { font-family: sans-serif; text-align: center; padding: 10px; margin: 0; background: #f9f9f9; }"
    "canvas { max-width: 90%; margin: 10px auto; }"
    "</style>"
    "<script>"
    "let tempChart, humChart, altChart;"
    "function initCharts() { const timeLabels=[];"
    "const ctx1=document.getElementById('tempChart').getContext('2d'); tempChart=new Chart(ctx1,{type:'line',data:{labels:timeLabels,datasets:[{label:'Temperatura BMP (°C)',data:[],borderColor:'red',fill:false}]},options:{animation:false,scales:{x:{display:false},y:{suggestedMin:0,suggestedMax:50}}}});"
    "const ctx2=document.getElementById('humChart').getContext('2d'); humChart=new Chart(ctx2,{type:'line',data:{labels:timeLabels,datasets:[{label:'Umidade (%)',data:[],borderColor:'blue',fill:false}]},options:{animation:false,scales:{x:{display:false},y:{suggestedMin:0,suggestedMax:100}}}});"
    "const ctx3=document.getElementById('altChart').getContext('2d'); altChart=new Chart(ctx3,{type:'line',data:{labels:timeLabels,datasets:[{label:'Altitude (m)',data:[],borderColor:'green',fill:false}]},options:{animation:false,scales:{x:{display:false}}}}); }"
    "function atualizar(){fetch('/estado').then(r=>r.json()).then(d=>{let t=new Date().toLocaleTimeString();"
    "document.getElementById('temp_val').innerText=d.str_tmp1;document.getElementById('temp2_val').innerText=d.str_tmp2;"
    "document.getElementById('alt_val').innerText=d.str_alt;document.getElementById('hum_val').innerText=d.str_umi;"
    "[tempChart,humChart,altChart].forEach(c=>{if(c.data.labels.length>=20)c.data.labels.shift();c.data.labels.push(t);});"
    "tempChart.data.datasets[0].data.push(parseFloat(d.str_tmp1));if(tempChart.data.datasets[0].data.length>20)tempChart.data.datasets[0].data.shift();"
    "humChart.data.datasets[0].data.push(parseFloat(d.str_umi));if(humChart.data.datasets[0].data.length>20)humChart.data.datasets[0].data.shift();"
    "altChart.data.datasets[0].data.push(parseFloat(d.str_alt));if(altChart.data.datasets[0].data.length>20)altChart.data.datasets[0].data.shift();"
    "tempChart.update();humChart.update();altChart.update();});}"
    "window.onload=initCharts;setInterval(atualizar,1000);"
    "</script></head><body>"
    "<h1>Sistema de Monitoramento de Sensores</h1>"
    "<p>Temperatura BMP: <span id='temp_val'>--</span></p><canvas id='tempChart' width='400' height='200'></canvas>"
    "<p>Temperatura AHT: <span id='temp2_val'>--</span></p>"
    "<p>Umidade: <span id='hum_val'>--</span></p><canvas id='humChart' width='400' height='200'></canvas>"
    "<p>Altitude Estimada: <span id='alt_val'>--</span></p><canvas id='altChart' width='400' height='200'></canvas>"
    "</body></html>";

char str_tmp1[5];  // Buffer para armazenar a string
char str_alt[5];   // Buffer para armazenar a string  
char str_tmp2[5];  // Buffer para armazenar a string
char str_umi[5];   // Buffer para armazenar a string 

static uint32_t lastIrqTime = 0;                // Registra o tempo da ultima interrupcao

struct http_state                               // Struct para manter o estado da conexão HTTP
{
    char response[4096];
    size_t len;
    size_t sent;
    size_t offset; // bytes já enfileirados para envio
};

//---------------------------------------------Protótipos---------------------------------------------

// Função de configuração inicial
void setup(void);

// Função de interrupção para lidar com os botões
void gpio_irq_handler(uint gpio, uint32_t events);

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure);

// Função de callback para enviar dados HTTP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

// Função de callback para receber dados HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Função de callback para aceitar novas conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função para iniciar o servidor HTTP
static void start_http_server(void);

//-----------------------------------------------Tasks------------------------------------------------

// Task de polling para manter a conexão Wi-Fi ativa
void vPollingTask(void *pvParameters)
{
    while (true)
    {
        cyw43_arch_poll(); // Polling do Wi-Fi para manter a conexão ativa
        vTaskDelay(1000);  // Aguarda 1 segundo antes de repetir
    }
}

//Task de Leitura dos sensores i2c
void vLeituraSensoresTask(void *pvParameters)
{
    AHT20_Data data;                                                                        // Estrutura para armazenar os dados do AHT20
    int32_t raw_temp_bmp;                                                                   // Variável para armazenar os dados de temperatura do BMP280
    int32_t raw_pressure;                                                                   // Variável para armazenar os dados de pressão do BMP280

    while (true)
    {
        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT_BMP, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);                   // Conversão da temperatura do BMP280
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);    // Conversão da pressão do BMP280

        // Cálculo da altitude
        double altitude = calculate_altitude(pressure);

        // Exibe os dados do BMP 280 no console
        printf("Core 0: Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Core 0: Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Core 0: Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT_AHT, &data))
        {
            // Exibe os dados do AHT20 no console
            printf("Core 0: Temperatura AHT: %.2f C\n", data.temperature);
            printf("Core 0: Umidade: %.2f %%\n\n\n", data.humidity);
        }
        else
        {
            printf("Core 0: Erro na leitura do AHT10!\n\n\n");
        }

        multicore_fifo_push_blocking(&temperature, sizeof(temperature));            // Envia temperatura do BMP280 para o outro núcleo
        multicore_fifo_push_blocking(&altitude, sizeof(altitude));                  // Envia altitude para o outro núcleo
        multicore_fifo_push_blocking(&data.temperature, sizeof(data.temperature));  // Envia temperatura para o outro núcleo
        multicore_fifo_push_blocking(&data.humidity, sizeof(data.humidity));        // Envia umidade para o outro núcleo
        printf("Core 0: Dados enviados para o núcleo 1\n");

        sprintf(str_tmp1, "%.1fC", temperature / 100.0);    // Variável para armazenar a temperatura do BMP280
        sprintf(str_alt, "%.0fm", altitude);                // Variável para armazenar a altitude
        sprintf(str_tmp2, "%.1fC", data.temperature);       // Variável para armazenar a temperatura do AHT20
        sprintf(str_umi, "%.1f%%", data.humidity);          // Variável para armazenar a umidade

        vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo antes de repetir
    }
}

//------------------------------------------------MAIN------------------------------------------------
int main()
{
    stdio_init_all();               // Inicializa a saída padrão (UART)
    sleep_ms(2000);                 // Aguarda 2 segundos para estabilização

    setup();                        // Configurações iniciais

    if (cyw43_arch_init())          // Inicializa o Wi-fi
    {
        return 1;
    }

    cyw43_arch_enable_sta_mode();   // Habilita o modo Station do Wi-Fi

    // Verifica se o Wi-Fi está conectado
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000))
    {
        printf("Falha na conexao Wi-Fi\n");
        return 1;
    }

    // Variáveis para armazenar o endereço IP
    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    printf("Conectado ao Wi-Fi %s\n", WIFI_SSID);   // Exibe o nome da rede Wi-Fi
    printf("Endereço IP: %s\n", ip_str);            // Exibe o endereço IP

    start_http_server();            // Inicia o servidor HTTP

    // Inicia o segundo núcleo para lidar com a leitura dos sensores
    // Inserir o nome da função que será executada no segundo núcleo no parenteses abaixo
    // Utilizar a FIFO para comunicação entre os núcleos (Task dos sensores)
    //multicore_launch_core1();

    // Tasks
    xTaskCreate(vPollingTask, "Polling Task", 256, NULL, 1, NULL);
    xTaskCreate(vLeituraSensoresTask, "Leitura Sensores Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();          // Inicia o escalonador do FreeRTOS
    panic_unsupported();            // Se o escalonador falhar, entra em pânico
}

//----------------------------------------------Funções------------------------------------------------
// Função de configuração inicial
void setup(void)
{
    gpio_init(BOTAO_B);                                                                         // Inicializa o pino do botão B
    gpio_set_dir(BOTAO_B, GPIO_IN);                                                             // Configura o pino do botão B como entrada
    gpio_pull_up(BOTAO_B);                                                                      // Habilita o pull-up no pino do botão B
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);   // Configura a interrupção para o botão B

    gpio_init(LED_BLUE);                                                                        // Inicializa o pino do LED Azul
    gpio_set_dir(LED_BLUE, GPIO_OUT);                                                           // Configura o pino do LED Azul como saída
    gpio_put(LED_BLUE, 0);                                                                      // Desliga o LED Azul

    // Inicializa o I2C
    printf("Inicializando I2C para BMP280...\n");
    i2c_init(I2C_PORT_BMP, 400 * 1000);
    gpio_set_function(I2C_SDA_BMP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_BMP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_BMP);
    gpio_pull_up(I2C_SCL_BMP);

    printf("Inicializando I2C para AHT20...\n");

    i2c_init(I2C_PORT_AHT, 400 * 1000);
    gpio_set_function(I2C_SDA_AHT, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_AHT, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_AHT);
    gpio_pull_up(I2C_SCL_AHT);

    // Inicializa o BMP280
    printf("Inicializando BMP280...\n");
    bmp280_init(I2C_PORT_BMP);
    bmp280_get_calib_params(I2C_PORT_BMP, &params);
    printf("BMP280 inicializado com sucesso!\n");

    // Inicializa o AHT20
    printf("Inicializando AHT20...\n");
    aht20_reset(I2C_PORT_AHT);
    aht20_init(I2C_PORT_AHT);
    printf("AHT20 inicializado com sucesso!\n");
}

// Interrupcao para o botão com debounce
void gpio_irq_handler(uint gpio, uint32_t events)
{
    // Variável para debounce
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (now - lastIrqTime < DEBOUNCE_MS)
        return; // Debounce
    lastIrqTime = now;          // Atualiza o tempo da última interrupção

    if (gpio == BOTAO_B)        // Se o botão B for pressionado
    {
        reset_usb_boot(0, 0);   // Reinicia o dispositivo em modo BOOTSEL
    }
}

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

#define CHUNK_SIZE 1024

// Envia próximo pedaço de resposta se houver espaço na janela
static void send_next_chunk(struct tcp_pcb *tpcb, struct http_state *hs)
{
    if (hs->offset >= hs->len) return;
    size_t remaining = hs->len - hs->offset;
    size_t to_send = remaining > CHUNK_SIZE ? CHUNK_SIZE : remaining;
    if (tcp_sndbuf(tpcb) < to_send) return; // Aguardar janela
    err_t err = tcp_write(tpcb, hs->response + hs->offset, to_send, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK)
    {
        hs->offset += to_send;
        tcp_output(tpcb);
    }
    else if (err != ERR_MEM)
    {
        // log erro irreversível
        printf("tcp_write fatal: %d\n", err);
    }
}

// Função de callback para enviar dados HTTP
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len)
    {
        tcp_close(tpcb);
        free(hs);
    }
    else
    {
        send_next_chunk(tpcb, hs);
    }
    return ERR_OK;
}

// Função de callback para receber dados HTTP
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p)
    {
        tcp_close(tpcb);
        return ERR_OK;
    }
    tcp_recved(tpcb, p->tot_len);

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs)
    {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;
    hs->offset = 0;

    // Endpoint JSON para /estado
    if (strstr(req, "GET /estado"))
    {
        char json[256];
        int jsonlen = snprintf(json, sizeof(json),
            "{\"str_tmp1\":\"%s\",\"str_tmp2\":\"%s\",\"str_alt\":\"%s\",\"str_umi\":\"%s\"}",
            str_tmp1, str_tmp2, str_alt, str_umi);
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            jsonlen, json);
        tcp_arg(tpcb, hs);
        tcp_sent(tpcb, http_sent);
        send_next_chunk(tpcb, hs);
        pbuf_free(p);
        return ERR_OK;
    }

    else
    {
        hs->len = snprintf(hs->response, sizeof(hs->response),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/html\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n"
                           "\r\n"
                           "%s",
                           (int)strlen(HTML_BODY), HTML_BODY);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    // Inicia envio por chunks
    send_next_chunk(tpcb, hs);

    pbuf_free(p);
    return ERR_OK;
}

// Função de callback para aceitar novas conexões TCP
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Função para iniciar o servidor HTTP
static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb)
    {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK)
    {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}
